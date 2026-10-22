#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A733 桌宠下位机 —— PC 端链路联调工具（对应 M1 / M2 / M2.5 验收）

用途：
    在没有 A7Z 的情况下，用 PC 串口直接验证下位机固件。
    也可以作为将来 A7Z 侧 C 代码的协议参考实现。

依赖：
    pip install pyserial

用法：
    # 交互式（推荐先跑这个）
    python pc_link_test.py COM4

    # 一键自检（依次跑 PING / GET_CONFIG / SET_CONFIG / 坏 CRC）
    python pc_link_test.py COM4 --selftest

    # 只发一次 PING 并打印应答
    python pc_link_test.py COM4 --ping

    # 波特率可改（默认 115200）
    python pc_link_test.py COM4 --baud 115200

协议（与 MCU 侧 Protocol/link_proto.c 一致）：
    +0  SOF      1B  0xA5
    +1  TYPE     1B   0x01 命令 / 0x02 应答 / 0x03 事件
    +2  SEQ      2B   小端
    +4  OP       1B
    +5  LEN      1B   0..200
    +6  PAYLOAD  LEN B
    +6+LEN CRC16 2B   小端，CCITT-FALSE(0x1021, init 0xFFFF)，覆盖 SOF..PAYLOAD
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial：请先执行  pip install pyserial")
    sys.exit(2)

# ------------------------------------------------------------------ 常量
SOF = 0xA5
TYPE_CMD, TYPE_ACK, TYPE_EVT = 0x01, 0x02, 0x03

OP_MOVE, OP_SERVO, OP_TTS = 0x01, 0x02, 0x03
OP_TOF_CFG, OP_CLEAR_FAULT, OP_PING = 0x04, 0x05, 0x06
OP_SET_CONFIG, OP_GET_CONFIG = 0x07, 0x08
OP_STEPPER_STAT = 0x09
OP_LINK_STAT = 0x0A
OP_STEPPER_TEST = 0x0B
OP_MOTOR_STOP = 0x0C

OP_ACK, OP_NAK, OP_PONG, OP_CONFIG = 0x81, 0x82, 0x86, 0x87
OP_EVT_FAULT, OP_EVT_CONFIG_CHANGED = 0x93, 0x96
OP_EVT_MOTION_DONE = 0x94

STATUS = {
    0x00: "OK",
    0x01: "CLAMPED",
    0x02: "BUSY",
    0x03: "FAULT_LATCHED",
    0x04: "CRC_ERROR",
    0x05: "UNKNOWN_OP",
    0x06: "FEATURE_DISABLED",
    0x07: "FLASH_FAIL",
    0x08: "NOT_IMPLEMENTED",
}

OPNAME = {
    OP_MOVE: "MOVE", OP_SERVO: "SERVO", OP_TTS: "TTS", OP_TOF_CFG: "TOF_CFG",
    OP_CLEAR_FAULT: "CLEAR_FAULT", OP_PING: "PING",
    OP_SET_CONFIG: "SET_CONFIG", OP_GET_CONFIG: "GET_CONFIG",
    OP_STEPPER_STAT: "STEPPER_STAT",
    OP_LINK_STAT: "LINK_STAT",
    OP_STEPPER_TEST: "STEPPER_TEST",
    OP_MOTOR_STOP: "MOTOR_STOP",
    OP_ACK: "ACK", OP_NAK: "NAK", OP_PONG: "PONG", OP_CONFIG: "CONFIG",
    OP_EVT_FAULT: "EVT_FAULT", OP_EVT_CONFIG_CHANGED: "EVT_CONFIG_CHANGED",
    OP_EVT_MOTION_DONE: "MOTION_DONE",
}

FAULT = {
    0x00: "NONE", 0x01: "HEARTBEAT_LOST", 0x02: "EDGE", 0x03: "OBSTACLE",
    0x04: "THERMAL", 0x05: "TOF_INIT", 0x0A: "NO_EDGE_PROTECT(BLIND)",
}

CFG_BITS = [
    (1 << 0, "SERVO"), (1 << 1, "TTS"), (1 << 2, "BT"),
    (1 << 3, "TOF0_bottom_front"), (1 << 4, "TOF1_bottom_left"),
    (1 << 5, "TOF2_bottom_right"), (1 << 6, "TOF3_front_obstacle"),
]


# ------------------------------------------------------------------ CRC
def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE：poly 0x1021, init 0xFFFF, 不反转。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(ftype: int, seq: int, op: int, payload: bytes = b"") -> bytes:
    if len(payload) > 200:
        raise ValueError("payload 超过 200 字节")
    head = struct.pack("<BBHBB", SOF, ftype, seq, op, len(payload))
    body = head + payload
    return body + struct.pack("<H", crc16_ccitt_false(body))


# ------------------------------------------------------------------ 解析
class FrameParser:
    """与 MCU 侧同构的逐字节状态机，含帧间超时。"""

    def __init__(self, timeout_s: float = 0.1):
        self.buf = bytearray()
        self.timeout_s = timeout_s
        self.last_rx = 0.0
        self.bad = 0

    def feed(self, data: bytes):
        out = []
        for b in data:
            now = time.monotonic()
            if self.buf and (now - self.last_rx) > self.timeout_s:
                # 半帧超时：丢弃，防止假 SOF 吞数据
                self.buf.clear()
                self.bad += 1
            self.last_rx = now

            self.buf.append(b)
            # 尝试在缓冲里找到第一个完整且 CRC 正确的帧
            while True:
                idx = self.buf.find(bytes([SOF]))
                if idx < 0:
                    self.buf.clear()
                    break
                if idx > 0:
                    del self.buf[:idx]
                if len(self.buf) < 6:
                    break
                length = self.buf[5]
                if length > 200:
                    del self.buf[:1]
                    self.bad += 1
                    continue
                total = 6 + length + 2
                if len(self.buf) < total:
                    break
                cand = bytes(self.buf[:total])
                calc = crc16_ccitt_false(cand[:-2])
                stored = struct.unpack("<H", cand[-2:])[0]
                if calc == stored:
                    out.append(parse_frame(cand))
                    del self.buf[:total]
                else:
                    del self.buf[:1]
                    self.bad += 1
        return out

    def feed_str(self, data: bytes):
        """同 feed，但把帧转成可读文本。"""
        lines = []
        for f in self.feed(data):
            lines.append(describe(f))
        return lines


def parse_frame(raw: bytes) -> dict:
    ftype, seq, op, length = struct.unpack("<BHBB", raw[1:6])
    return {"type": ftype, "seq": seq, "op": op,
            "payload": raw[6:6 + length], "raw": raw}


def describe(f: dict) -> str:
    op = f["op"]
    name = OPNAME.get(op, "OP?0x%02X" % op)
    p = f["payload"]
    parts = ["TYPE=%d SEQ=%d OP=0x%02X(%s) LEN=%d"
             % (f["type"], f["seq"], op, name, len(p))]

    if op == OP_PONG and len(p) >= 9:
        # 固件发的 uptime 单位是【毫秒】；早期版本这里误标成 s，已修正
        up_ms, fault = struct.unpack("<IB", p[:5])
        cfg = struct.unpack("<I", p[5:9])[0]
        parts.append("uptime=%d.%03ds (%dms)"
                     % (up_ms // 1000, up_ms % 1000, up_ms))
        parts.append("fault=0x%02X(%s)" % (fault, FAULT.get(fault, "?")))
        parts.append("config=0x%08X %s" % (cfg, cfg_str(cfg)))
        if len(p) >= 10:
            flags = p[9]
            src = "FLASH" if (flags & 0x01) else "DEFAULT"
            blind = "blind" if (flags & 0x02) else "normal"
            parts.append("flags=0x%02X src=%s %s" % (flags, src, blind))
        if len(p) >= 11:
            impl = p[10]
            parts.append("impl=0x%02X %s" % (impl, impl_str(impl)))
        else:
            parts.append("impl=?? [旧固件：PONG 无 impl 字节]")
    elif op == OP_ACK and len(p) >= 1:
        parts.append("status=0x%02X(%s)" % (p[0], STATUS.get(p[0], "?")))
    elif op == OP_NAK and len(p) >= 2:
        parts.append("status=0x%02X(%s)" % (p[0], STATUS.get(p[0], "?")))
        parts.append("detail=0x%02X(%s)" % (p[1], OPNAME.get(p[1], "")))
    elif op == OP_CONFIG and len(p) >= 4:
        cfg = struct.unpack("<I", p[:4])[0]
        parts.append("config=0x%08X %s" % (cfg, cfg_str(cfg)))
    elif op == OP_EVT_CONFIG_CHANGED and len(p) >= 4:
        cfg = struct.unpack("<I", p[:4])[0]
        parts.append("config=0x%08X %s" % (cfg, cfg_str(cfg)))
    elif op == OP_EVT_FAULT and len(p) >= 1:
        parts.append("fault=0x%02X(%s)" % (p[0], FAULT.get(p[0], "?")))
    elif op == OP_EVT_MOTION_DONE and len(p) >= 5:
        motor = "L" if p[0] == 0 else "R"
        pos = struct.unpack("<i", p[1:5])[0]
        parts.append("motor=%s position=%d" % (motor, pos))
    elif p:
        parts.append("payload=" + p.hex(" "))
    return "  ".join(parts)


def cfg_str(cfg: int) -> str:
    on = [name for bit, name in CFG_BITS if cfg & bit]
    return "[%s]" % (", ".join(on) if on else "all off")


# 功能实现位图（与固件 app_config.h 的 IMPL_* 一致）
IMPL_BITS = [
    (1 << 0, "SERVO"), (1 << 1, "MOVE"), (1 << 2, "TTS"), (1 << 3, "TOF"),
    (1 << 4, "CONFIG"), (1 << 5, "CLEAR_FAULT"),
]


def impl_str(mask: int) -> str:
    on = [name for bit, name in IMPL_BITS if mask & bit]
    return "[%s]" % (", ".join(on) if on else "none")


# ------------------------------------------------------------------ 通信
class Link:
    def __init__(self, port: str, baud: int, verbose: bool = True):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.parser = FrameParser()
        self.seq = 0
        self.verbose = verbose
        # 这些操作码不打印（动作期间的保活 PING/PONG 会刷屏）
        self.silent_ops = set()

    def close(self):
        self.ser.close()

    def next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq

    def send(self, ftype: int, op: int, payload: bytes = b"", seq=None) -> int:
        if seq is None:
            seq = self.next_seq()
        frame = build_frame(ftype, seq, op, payload)
        self.ser.write(frame)
        if self.verbose and op not in self.silent_ops:
            print("  --> TX %s" % frame.hex(" "))
        return seq

    def recv(self, timeout: float = 1.0):
        """收集一段时间内的所有应答/事件帧。"""
        deadline = time.monotonic() + timeout
        frames = []
        while time.monotonic() < deadline:
            data = self.ser.read(256)
            if data:
                got = self.parser.feed(data)
                if got:
                    if self.verbose:
                        for f in got:
                            if f["op"] not in self.silent_ops:
                                print("  <-- RX " + describe(f))
                    frames.extend(got)
                    # 收到足够就提前返回
            else:
                time.sleep(0.005)
        return frames

    def drain(self):
        self.ser.reset_input_buffer()


# ------------------------------------------------------------------ 测试步骤
def ask_ping(link: Link):
    print("\n[PING] 保活并读取状态")
    link.drain()
    seq = link.send(TYPE_CMD, OP_PING)
    frames = link.recv(1.0)
    for f in frames:
        if f["op"] == OP_PONG and f["seq"] == seq:
            return True
    print("  ✗ 未收到 PONG")
    return False


def ask_get_config(link: Link):
    print("\n[GET_CONFIG] 读取当前功能开关")
    link.drain()
    seq = link.send(TYPE_CMD, OP_GET_CONFIG)
    frames = link.recv(1.0)
    for f in frames:
        if f["op"] == OP_CONFIG and f["seq"] == seq:
            return struct.unpack("<I", f["payload"][:4])[0]
    print("  ✗ 未收到 CONFIG")
    return None


def ask_set_config(link: Link, cfg: int):
    print("\n[SET_CONFIG] 写入 0x%08X %s" % (cfg, cfg_str(cfg)))
    link.drain()
    seq = link.send(TYPE_CMD, OP_SET_CONFIG, struct.pack("<I", cfg))
    frames = link.recv(1.5)
    ok = False
    for f in frames:
        if f["op"] == OP_ACK and f["seq"] == seq:
            ok = (f["payload"][0] == 0x00)
        if f["op"] == OP_NAK and f["seq"] == seq:
            print("  ✗ NAK")
            return False
    if not ok:
        print("  ✗ 未收到 ACK")
    return ok


def ask_bad_crc(link: Link):
    """坏 CRC 必须被拒。

    ⚠️ 注意：判定只看【应答帧 TYPE=2 且 SEQ 匹配】。
    因为 PC 端每条命令之间会 idle 1 秒以上，而下位机心跳超时正好是 1000 ms，
    期间会主动上报 EVT_FAULT（TYPE=3），那是正常的心跳事件，
    不能当成"坏帧的应答"——早期版本就是在这里误判的。

    另外：先发一个正常 PING 把心跳计时器复位，让窗口内不出现心跳事件。
    """
    print("\n[坏 CRC] 应被拒（不产生 ACK/NAK）")

    # 1) 先正常 PING 一次，重置下位机的心跳计时，避免 EVT_FAULT 干扰
    link.drain()
    link.send(TYPE_CMD, OP_PING)
    link.recv(0.4)
    link.drain()

    # 2) 发一个 CRC 被破坏的帧
    frame = bytearray(build_frame(TYPE_CMD, 0x9999, OP_PING))
    frame[4] ^= 0xFF                      # 破坏 OP，CRC 不再匹配
    link.ser.write(bytes(frame))
    if link.verbose:
        print("  --> TX %s  (故意损坏)" % bytes(frame).hex(" "))

    frames = link.recv(0.6)

    # 3) 只关心"针对 seq=0x9999 的应答帧"
    bad_replies = [f for f in frames
                   if f["type"] == TYPE_ACK and f["seq"] == 0x9999]
    evts = [f for f in frames if f["type"] == TYPE_EVT]

    if bad_replies:
        print("  ✗ 坏帧竟产生了应答：%s"
              % [OPNAME.get(f["op"]) for f in bad_replies])
        return False

    if evts:
        print("  （窗口内收到 %d 条 EVT，属正常心跳事件，已忽略）" % len(evts))

    print("  ✓ 无 ACK/NAK 应答（正确）")
    return True


def ask_not_implemented(link: Link):
    print("\n[MOVE] 本阶段应返回 NAK(NOT_IMPLEMENTED)")
    link.drain()
    seq = link.send(TYPE_CMD, OP_MOVE, bytes([0, 5, 0x00, 0x00]))
    frames = link.recv(1.0)
    for f in frames:
        if f["seq"] == seq and f["op"] == OP_NAK:
            if f["payload"][0] == 0x08:
                print("  ✓ 返回 NOT_IMPLEMENTED（符合 M0-M2.5 阶段预期）")
                return True
            print("  ? 返回了其它状态 0x%02X" % f["payload"][0])
            return True
    print("  ✗ 未收到 NAK")
    return False


def ask_clear_fault(link: Link, ack: bool):
    print("\n[CLEAR_FAULT] blind_ack=%s" % ("0x5A" if ack else "缺失"))
    link.drain()
    payload = bytes([0x03]) + (bytes([0x5A]) if ack else b"")
    seq = link.send(TYPE_CMD, OP_CLEAR_FAULT, payload)
    frames = link.recv(1.0)
    for f in frames:
        if f["seq"] == seq:
            return f["op"] in (OP_ACK, OP_NAK)
    return False


def ask_servo(link: Link, deg: int, ms: int = 0):
    """M3：舵机角度指令。

    payload = deg(int8, 有符号) + ms(uint16 LE)，共 3 字节。
    返回 (ok, clamped)：
      clamped=True 表示角度被下位机钳位（status == ST_CLAMPED）
    """
    print("\n[SERVO] deg=%d ms=%d" % (deg, ms))
    if not (-128 <= deg <= 127):
        print("  ✗ deg=%d 超出 int8 范围（协议里 deg 是有符号单字节）" % deg)
        print("    想测限幅请用 100 或 -100（>45 就会被钳位）")
        return (False, False)

    link.drain()
    payload = struct.pack("<bH", deg, ms)
    seq = link.send(TYPE_CMD, OP_SERVO, payload)
    frames = link.recv(1.0)

    for f in frames:
        if f["seq"] != seq or f["type"] != TYPE_ACK:
            continue
        if f["op"] == OP_ACK:
            st = f["payload"][0]
            if st == 0x00:
                print("  ✓ ACK(OK)")
                return (True, False)
            if st == 0x01:
                print("  ✓ ACK(CLAMPED) — 角度被钳到 ±45°")
                return (True, True)
            print("  ? ACK(status=0x%02X %s)" % (st, STATUS.get(st, "?")))
            return (True, False)
        if f["op"] == OP_NAK:
            st = f["payload"][0]
            print("  ✗ NAK(status=0x%02X %s)"
                  % (st, STATUS.get(st, "?")))
            if st == 0x06:
                print("    提示：CFG_SERVO_EN=0。先发 s 0000007B 打开舵机位")
            elif st == 0x08:
                impl = probe_impl(link)
                print("    提示：板子里的固件【没有实现 SERVO 命令】")
                if impl is None:
                    print("    → PONG 里没有 impl 字节 = 还是 M0~M2.5 版固件")
                else:
                    print("    → 当前固件实现集 = 0x%02X %s"
                          % (impl, impl_str(impl)))
                print("    → 请在 Keil 里 **Rebuild + Load** 后重试")
                print("    → 编译输出里应能看到 `compiling servo.c...`")
            return (False, False)

    print("  ✗ 未收到 ACK/NAK")
    return (False, False)


def probe_impl(link: Link):
    """读一次 PONG，返回 impl 位图；旧固件（无该字节）返回 None。"""
    link.drain()
    seq = link.send(TYPE_CMD, OP_PING)
    frames = link.recv(0.8)
    for f in frames:
        if f["op"] == OP_PONG and f["seq"] == seq:
            p = f["payload"]
            return p[10] if len(p) >= 11 else None
    return None


# ------------------------------------------------------------------ M4/M5 运动
MOVE_DIRS = {
    0: "FORWARD", 1: "BACKWARD", 2: "TURN_LEFT", 3: "TURN_RIGHT", 4: "STOP",
}
SPEED_HZ = {0: 120, 1: 200, 2: 300, 3: 600}


def ask_move(link: Link, direction: int, speed: int = 2, steps: int = 200):
    """M4/M5：履带运动。

    payload = dir(1) + speed(1) + steps(2 LE) = 4 字节
      dir   : 0 前进 / 1 后退 / 2 左转 / 3 右转 / 4 停止
      speed : 0..3 -> 120/200/300/600 Hz
      steps : 半步数；0xFFFF = 持续运动

    ⚠️ 动作期间必须持续发 PING！
       固件的心跳超时是 1000 ms，超时后安全联锁会立刻 Stepper_StopAll()。
       如果不保活，任何超过 1 秒的动作都会被腰斩（表现为"只跑 1 秒就停"）。
       真实的 A7Z 也是每 500 ms 发一次 PING，本函数按 300 ms 保活。
    """
    name = MOVE_DIRS.get(direction, "?")
    hz = SPEED_HZ.get(speed, 300)

    if direction == 4:
        print("\n[MOVE] STOP")
        est_s = 0.0
    else:
        est_s = steps / float(hz)
        print("\n[MOVE] %s  speed=%d(%dHz)  steps=%d  → 约 %.1f s"
              % (name, speed, hz, steps, est_s))

    if not (-1 <= direction <= 4):
        print("  ✗ dir 必须是 0..4")
        return False
    if not (0 <= steps <= 0xFFFF):
        print("  ✗ steps 超出 uint16")
        return False

    link.drain()
    payload = struct.pack("<BBH", direction, speed & 0xFF, steps)
    seq = link.send(TYPE_CMD, OP_MOVE, payload)

    # 保活期间的 PING/PONG 不打印，避免刷屏
    saved_silent = link.silent_ops
    link.silent_ops = {OP_PING, OP_PONG}

    ack = None
    done_mask = 0
    last_ping = time.monotonic()
    start = time.monotonic()
    deadline = start + max(2.0, est_s * 2.0 + 4.0)

    try:
        while time.monotonic() < deadline:
            # ---- 心跳保活 ----
            if time.monotonic() - last_ping > 0.3:
                link.send(TYPE_CMD, OP_PING, seq=link.next_seq())
                last_ping = time.monotonic()

            for fr in link.recv(0.08):
                if fr["type"] == TYPE_ACK and fr["seq"] == seq:
                    ack = fr
                elif fr["op"] == OP_EVT_MOTION_DONE and len(fr["payload"]) >= 5:
                    done_mask |= (1 << fr["payload"][0])

            if ack is not None:
                if ack["op"] == OP_NAK:
                    break
                if direction == 4:
                    break
                if done_mask == 0x03:      # 左右都报完成
                    break
            elif time.monotonic() - start > 2.0:
                break
    finally:
        link.silent_ops = saved_silent

    if ack is None:
        print("  ✗ 未收到 ACK/NAK")
        return False

    if ack["op"] == OP_NAK:
        st = ack["payload"][0]
        detail = ack["payload"][1] if len(ack["payload"]) >= 2 else 0
        print("  ✗ NAK(status=0x%02X %s, detail=0x%02X)"
              % (st, STATUS.get(st, "?"), detail))
        if st == 0x08:
            impl = probe_impl(link)
            print("    提示：固件没有实现 MOVE 命令")
            if impl is None:
                print("    → PONG 无 impl 字节 = 旧固件，请 Rebuild + Load")
            else:
                print("    → 当前实现集 = 0x%02X %s" % (impl, impl_str(impl)))
        elif st == 0x02:
            print("    提示：电机正在运动，先发 st 停止")
        elif st == 0x03:
            print("    提示：热保护冷却中，等 2 秒再试")
        return False

    st = ack["payload"][0]
    dist = ack["payload"][1] if len(ack["payload"]) >= 2 else None
    msg = "  ✓ ACK(%s)" % STATUS.get(st, "0x%02X" % st)
    if dist is not None and direction != 4:
        if dist == 0xFF:
            msg += "  预计距离 > 25 mm"
        elif dist > 0:
            msg += "  预计距离 %.1f mm" % (dist / 10.0)
    if direction != 4:
        msg += "  [完成 L=%s R=%s]" % (
            "Y" if done_mask & 1 else "-", "Y" if done_mask & 2 else "-")
    print(msg)
    return True


def run_endurance(link: Link, cycles: int = 10, steps: int = 2000,
                  speed: int = 2, gap: float = 0.5) -> int:
    """耐久测试：连发 N 次前进 MOVE，逐轮核对【位置寄存器】。

    与早期版本的区别（关键）：
        旧版依赖 MOTION_DONE 事件判断完成，事件一旦丢帧就完全瞎掉，
        会把"事件没收到"误报成"丢步 55%"。
        新版用 STEPPER_STAT 轮询 remaining/position —— 位置寄存器不会丢，
        这才是真值。

    同时检查两件事：
        1) 左右位置是否相等  —— 不等说明有一侧没启动（差速分叉）
        2) remaining 是否为 0 —— 非 0 说明动作被截断（热保护/联锁）

    固件热保护（看启动横幅里的 thermal 行）：
        12V: run<=8000ms overlap<=3000ms  -> 单次别超 900 步 @300Hz
         5V: run<=60000ms overlap<=60000ms -> 基本不受限
    """
    hz = SPEED_HZ.get(speed, 300)
    secs = steps / float(hz)

    print("=" * 66)
    print(" 耐久测试：%d 次 × %d 半步 @ %d Hz" % (cycles, steps, hz))
    print(" 单次约 %.1f s，累计约 %.1f s，总距离约 %.1f mm"
          % (secs, secs * cycles, steps * cycles * 94.2 / 4076.0))
    print("=" * 66)

    st0 = read_stepper_stat(link)
    if st0 is None:
        print(" 固件不支持 STEPPER_STAT（需要 Rebuild）")
        return 1
    base_l = st0["L"]["pos"]
    base_r = st0["R"]["pos"]
    base_trips = st0["trips"]
    print(" 起始：L=%d R=%d trips=%d fault=0x%02X"
          % (base_l, base_r, base_trips, st0["fault"]))
    print("-" * 66)

    fails = 0
    bad_desync = 0
    bad_trunc = 0
    last = st0

    for i in range(1, cycles + 1):
        link.drain()
        seq = link.send(TYPE_CMD, OP_MOVE,
                        struct.pack("<BBH", 0, speed & 0xFF, steps))

        # ---- 等 ACK ----
        ack = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5 and ack is None:
            for fr in link.recv(0.05):
                if fr["type"] == TYPE_ACK and fr["seq"] == seq:
                    ack = fr
        if ack is None:
            print("  第 %2d/%d  未收到 ACK" % (i, cycles))
            fails += 1
            continue
        if ack["op"] == OP_NAK:
            st = ack["payload"][0]
            print("  第 %2d/%d  NAK(0x%02X %s)"
                  % (i, cycles, st, STATUS.get(st, "?")))
            fails += 1
            time.sleep(2.0)
            continue

        # ---- 保活 + 轮询真值 ----
        saved_silent = link.silent_ops
        link.silent_ops = {OP_PING, OP_PONG, OP_STEPPER_STAT}
        st = None
        try:
            deadline = time.monotonic() + secs * 2.5 + 5.0
            last_ping = 0.0
            t_dot = time.monotonic()
            print("      ", end="", flush=True)
            while time.monotonic() < deadline:
                if time.monotonic() - last_ping > 0.35:
                    link.send(TYPE_CMD, OP_PING, seq=link.next_seq())
                    last_ping = time.monotonic()
                if time.monotonic() - t_dot > 1.0:
                    print(".", end="", flush=True)   # 活着就点一个点
                    t_dot = time.monotonic()

                time.sleep(0.15)          # 别把串口刷爆
                st = read_stepper_stat(link)
                if st is not None:
                    if (st["L"]["rem"] == 0 and st["R"]["rem"] == 0
                            and not (st["L"]["flags"] & 0x01)
                            and not (st["R"]["flags"] & 0x01)):
                        break
            print()
        finally:
            link.silent_ops = saved_silent

        if st is None:
            print("  第 %2d/%d  状态读取失败" % (i, cycles))
            fails += 1
            continue

        last = st
        exp = steps * i
        got = st["L"]["pos"] - base_l
        delta = got - exp
        tags = []
        if st["L"]["pos"] != st["R"]["pos"]:
            bad_desync += 1
            tags.append("左右差%d" % (st["L"]["pos"] - st["R"]["pos"]))
        if st["L"]["rem"] != 0 or st["R"]["rem"] != 0:
            bad_trunc += 1
            tags.append("截断(rem=%d/%d)" % (st["L"]["rem"], st["R"]["rem"]))
        if st["trips"] != base_trips:
            tags.append("热跳闸+%d" % (st["trips"] - base_trips))

        print("  第 %2d/%d  L=%7d R=%7d  期望=%7d  偏差=%+d  %s"
              % (i, cycles, st["L"]["pos"], st["R"]["pos"], base_l + exp,
                 delta, ("  << " + ", ".join(tags)) if tags else ""))

        time.sleep(gap)

    print("-" * 66)
    exp_total = steps * cycles
    got_total = last["L"]["pos"] - base_l
    delta = got_total - exp_total
    pct = 100.0 * delta / exp_total if exp_total else 0.0

    print(" 累计：期望 %d，实际 %d，偏差 %+d (%.2f%%)"
          % (exp_total, got_total, delta, pct))
    print(" 左侧累计 = %d   右侧累计 = %d   (差 %d)"
          % (last["L"]["pos"] - base_l, last["R"]["pos"] - base_r,
             (last["L"]["pos"] - base_l) - (last["R"]["pos"] - base_r)))
    print(" 热跳闸 %d 次   左右分叉 %d 次   动作截断 %d 次"
          % (last["trips"] - base_trips, bad_desync, bad_trunc))
    print()

    if bad_desync:
        print(" [XX] 左右分叉：有一侧没启动。检查 Stepper_MoveBoth 的回滚逻辑")
    elif bad_trunc:
        print(" [XX] 动作被截断：看启动横幅的 thermal 行，确认是否还在 12V 阈值")
    elif abs(pct) < 0.5:
        print(" [OK] 几乎无丢步，左右同步 —— 机械与电源都没问题")
    elif abs(pct) < 3.0:
        print(" [!!] 轻微丢步：降一档复测，或把 STEPPER_ACCEL_HZ_PER_MS 调小")
    else:
        print(" [XX] 明显丢步：先降档（fwd 200 1）复测，再查")
        print("      1) 电机电源 5V 是否真的到位  2) 机械卡阻  3) 相序线接触")

    print("=" * 66)
    return 1 if (fails or bad_desync or bad_trunc) else 0

def run_move_selftest(link: Link) -> int:
    """M4 验收：单电机精确步数 + 加减速 + 停止。"""
    print("=" * 60)
    print(" M4 / M5 运动自检")
    print("=" * 60)

    fails = 0

    print("\n--- 1) 前进 200 半步（约 4.6 mm）---")
    if not ask_move(link, 0, 2, 200):
        fails += 1
    time.sleep(2.0)

    print("\n--- 2) 后退 200 半步（应回到起点）---")
    if not ask_move(link, 1, 2, 200):
        fails += 1
    time.sleep(2.0)

    print("\n--- 3) 原地左转 300 半步 ---")
    if not ask_move(link, 2, 2, 300):
        fails += 1
    time.sleep(2.5)

    print("\n--- 4) 原地右转 300 半步（应转回来）---")
    if not ask_move(link, 3, 2, 300):
        fails += 1
    time.sleep(2.5)

    print("\n--- 5) 低速档 100 半步 ---")
    if not ask_move(link, 0, 0, 100):
        fails += 1
    time.sleep(2.0)

    print("\n--- 6) 运动中发 STOP，应立即停 ---")
    ask_move(link, 0, 2, 2000)
    time.sleep(0.5)
    ask_move(link, 4, 0, 0)
    time.sleep(0.3)

    print("\n--- 7) 运动中再发 MOVE 应被拒（BUSY）或已停 ---")
    ask_move(link, 0, 2, 2000)
    time.sleep(0.3)
    ask_move(link, 0, 2, 200)   # 期望 NAK(BUSY)
    ask_move(link, 4, 0, 0)
    time.sleep(0.5)

    print("\n" + "=" * 60)
    if fails == 0:
        print(" 运动自检完成")
        print(" 人工确认：")
        print("   · 前进/后退是否走了再退回原处")
        print("   · 左右转是否方向正确（若反了，改 STEPPER_x_DIR_SIGN）")
        print("   · 是否有丢步的咔咔声")
        print("   · ULN2003 芯片温度是否可接受")
    else:
        print(" 有 %d 项失败" % fails)
    print("=" * 60)
    return fails


def _read_lstat_raw(link: Link):
    """静默读一次 LINK_STAT，返回 (uptime_ms, good, bad, rx, ovf)。"""
    seq = link.send(TYPE_CMD, OP_LINK_STAT)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        for f in link.recv(0.05):
            if f["op"] == OP_LINK_STAT and f["seq"] == seq:
                p = f["payload"]
                if len(p) < 26:
                    return None
                return (struct.unpack("<I", p[0:4])[0],
                        struct.unpack("<I", p[4:8])[0],
                        struct.unpack("<I", p[8:12])[0],
                        struct.unpack("<I", p[12:16])[0],
                        struct.unpack("<I", p[16:20])[0])
    return None


def ask_motor_stop(link: Link, clear_cooldown: bool = False) -> bool:
    """独立停止命令：立即切断两路步进。

    与 MOVE(dir=4) 的区别：
      这是独立操作码，不依赖上位机把 dir 算对；
      无条件生效 —— 不论是否在运动、是否处于故障/冷却状态。

    clear_cooldown=True 会同时清除热保护冷却（台架调试用）。
    """
    print("\n[MOTOR_STOP] 立即停止%s"
          % ("（并清除热保护冷却）" if clear_cooldown else ""))

    link.drain()
    payload = bytes([0x01 if clear_cooldown else 0x00])
    seq = link.send(TYPE_CMD, OP_MOTOR_STOP, payload)
    frames = link.recv(0.8)

    for f in frames:
        if f["type"] == TYPE_ACK and f["seq"] == seq:
            if f["op"] == OP_ACK:
                print("  ✓ ACK(OK)  两路步进已断电")
                return True
            if f["op"] == OP_NAK:
                st = f["payload"][0]
                print("  ✗ NAK(0x%02X %s)" % (st, STATUS.get(st, "?")))
                return False
    print("  ✗ 未收到应答（固件可能不支持 OP_MOTOR_STOP，需 Rebuild）")
    return False


def ask_single_motor(link: Link, motor: int, steps: int) -> bool:
    """只驱动一侧电机，用【原始相位方向】（不经过 DIR_SIGN 换算）。

    这是方向标定唯一可靠的手段：
      fwd/lft/rgt 都是双电机复合运动，任一符号错就看不出是哪一处；
      单侧驱动时"正数让它往哪走"是唯一确定的事实。

    motor: 0=左  1=右
    steps: 有符号，正 = 原始相位递增方向
    """
    side = "左" if motor == 0 else "右"
    print("\n[单侧] %s电机  steps=%+d（原始方向，200Hz）" % (side, steps))

    link.drain()
    seq = link.send(TYPE_CMD, OP_STEPPER_TEST,
                    struct.pack("<Bi", motor, steps))

    saved = link.silent_ops
    link.silent_ops = {OP_PING, OP_PONG}
    ack = None
    done = False
    secs = abs(steps) / 200.0
    t0 = time.monotonic()
    deadline = t0 + secs * 2.0 + 4.0
    last_ping = t0
    try:
        while time.monotonic() < deadline:
            if time.monotonic() - last_ping > 0.35:
                link.send(TYPE_CMD, OP_PING, seq=link.next_seq())
                last_ping = time.monotonic()
            for fr in link.recv(0.08):
                if fr["type"] == TYPE_ACK and fr["seq"] == seq:
                    ack = fr
                if (fr["op"] == OP_EVT_MOTION_DONE
                        and len(fr["payload"]) >= 5
                        and fr["payload"][0] == motor):
                    done = True
            if ack is not None and done:
                break
            if ack is not None and time.monotonic() - t0 > 2.0 + secs:
                break
    finally:
        link.silent_ops = saved

    if ack is None:
        print("  ✗ 未收到应答")
        return False
    if ack["op"] == OP_NAK:
        st = ack["payload"][0]
        print("  ✗ NAK(0x%02X %s)" % (st, STATUS.get(st, "?")))
        return False
    print("  ✓ 完成%s" % ("" if done else "（未收到完成事件）"))
    return True


def run_dir_calibration(link: Link, steps: int = 400) -> int:
    """单侧方向标定：4 条命令，每条只有一侧在动，观察毫无歧义。"""
    print("=" * 66)
    print(" 方向标定（单侧驱动）—— 每条只有一侧电机在动")
    print("=" * 66)
    print(" 请站在机器人【正后方】，和它朝同一方向看。")
    print(" 『前』= 远离你，『后』= 靠近你。")
    print()

    plan = [
        (0, +steps, "左电机 原始方向【正】"),
        (0, -steps, "左电机 原始方向【负】"),
        (1, +steps, "右电机 原始方向【正】"),
        (1, -steps, "右电机 原始方向【负】"),
    ]
    for motor, stp, desc in plan:
        print("-" * 66)
        print(">>> %s  （%+d 步，约 %.1f 秒）" % (desc, stp, abs(stp) / 200.0))
        ask_single_motor(link, motor, stp)
        time.sleep(1.2)

    print()
    print("=" * 66)
    print(" 请照实回我这 4 行（填『前』或『后』）：")
    print("   左电机 +  →  ___")
    print("   左电机 -  →  ___")
    print("   右电机 +  →  ___")
    print("   右电机 -  →  ___")
    print()
    print(" 有了这 4 个事实，我就能一次把两个 DIR_SIGN 定准，不用再试。")
    print("=" * 66)
    return 0


def run_calibration(link: Link, steps: int = 500, gap: float = 2.5) -> int:
    """方向标定：依次跑四个方向，每条之间停顿，方便你逐个观察。

    判据（对着机器人看）：
      fwd  两履带同向，机器人【直线前进】
      bck  两履带同向，机器人【直线后退】
      lft  左履带后退 + 右履带前进，机器人【原地左转】
      rgt  左履带前进 + 右履带后退，机器人【原地右转】

    任何一条不符，把 4 条的实际表现告诉我，我据此定符号。
    """
    plan = [
        (0, "fwd", "两履带同向 → 机器人直线【前进】"),
        (1, "bck", "两履带同向 → 机器人直线【后退】"),
        (2, "lft", "左履带后退 + 右履带前进 → 原地【左转】"),
        (3, "rgt", "左履带前进 + 右履带后退 → 原地【右转】"),
    ]

    print("=" * 64)
    print(" 方向标定：每个方向 %d 半步，间隔 %.1f 秒" % (steps, gap))
    print(" 请盯住【左履带】和【右履带】各自往哪走")
    print("=" * 64)

    for d, name, expect in plan:
        print("\n>>> 下一条：%s" % name)
        print("    预期：%s" % expect)
        input("    按回车执行...") if False else None
        ask_move(link, d, 2, steps)
        # 等动作跑完再问（500 步约 1.7 秒）
        time.sleep(gap - 1.7 if gap > 1.7 else 0.5)
        print("    ↑ 这条实际是：", end="", flush=True)

    print("\n")
    print("=" * 64)
    print(" 请按这个格式回我（照实填）：")
    print("   fwd → 前进 / 后退 / 两轮反向 / 不转")
    print("   bck → 前进 / 后退 / 两轮反向 / 不转")
    print("   lft → 原地左转 / 原地右转 / 直线 / 不动")
    print("   rgt → 原地左转 / 原地右转 / 直线 / 不动")
    print("=" * 64)
    return 0


def run_tick_test(link: Link, seconds: float = 10.0) -> int:
    """测量下位机 1 ms 节拍是否准。

    原理：下位机用 TIM3 产生 1 ms 节拍并累加成 uptime。
          用 PC 的高精度时钟等固定时间，比较 uptime 增量。
          误差超过 ±2% 就说明 TIM3 的 PSC/ARR 或 APB1 倍频有问题。

    为什么这个很重要：
          心跳超时、热保护限时、加减速斜坡全都基于这个 tick。
          tick 快 2 倍 -> 心跳 1000ms 变成 500ms 真实时间 ->
          "明明每 0.4 秒发一次 PING 却报心跳丢失"。
    """
    print("=" * 60)
    print(" 时基校准：测量下位机 1 ms tick 是否真实")
    print("=" * 60)

    a = _read_lstat_raw(link)
    if a is None:
        print(" 读不到 LINK_STAT（固件需 Rebuild）")
        return 1

    t0 = time.monotonic()
    print(" 开始计时 %.1f 秒，请稍等..." % seconds)
    time.sleep(seconds)
    t1 = time.monotonic()

    b = _read_lstat_raw(link)
    if b is None:
        print(" 第二次读取失败")
        return 1

    real_ms = (t1 - t0) * 1000.0
    dev_ms = float(b[0] - a[0])

    # 串口往返有几十 ms 误差，误差预算按 2% 或 300ms 取大
    err_pct = 100.0 * (dev_ms - real_ms) / real_ms

    print()
    print(" PC 实测时间    : %.1f ms" % real_ms)
    print(" 下位机 tick    : %.1f ms" % dev_ms)
    print(" 偏差           : %+.1f ms  (%+.2f%%)" % (dev_ms - real_ms, err_pct))
    print(" 等效 tick 频率 : %.1f Hz  (应为 1000.0)" % (dev_ms / (real_ms / 1000.0)))
    print()

    if abs(err_pct) < 2.0:
        print(" [OK] 时基准确（误差 < 2%）")
        rc = 0
    elif err_pct > 0:
        print(" [XX] 下位机 tick 跑【快】了 %.1f 倍" % (dev_ms / real_ms))
        print("      影响：心跳 1000ms 实际只有 %.0f ms 真实时间"
              % (1000.0 * real_ms / dev_ms))
        print("      -> PING 间隔必须 < %.0f ms 才不会被误判掉线"
              % (1000.0 * real_ms / dev_ms))
        print("      排查：TIM3 的 PSC/ARR，以及 RCC_CFGR 的 PPRE1 分频")
        rc = 1
    else:
        print(" [XX] 下位机 tick 跑【慢】了 %.1f 倍" % (real_ms / dev_ms))
        print("      影响：所有限时（热保护/加减速）都会拉长")
        rc = 1

    print("=" * 60)
    return rc


def ask_link_stat(link: Link) -> bool:
    """诊断：链路健康计数器。

    用来判断"上位机在发帧，下位机却说心跳丢失"到底卡在哪一层：
      rx_bytes 涨 / good 不涨   -> 帧被 CRC 或解析丢掉
      rx_overflow 涨            -> 主循环太慢，接收环形缓冲溢出
      last_frame_age 一直很大   -> 下位机根本没收到
    """
    print("\n[LINK_STAT] 链路健康")
    link.drain()
    seq = link.send(TYPE_CMD, OP_LINK_STAT)
    frames = link.recv(1.0)

    resp = None
    for f in frames:
        if f["op"] == OP_LINK_STAT and f["seq"] == seq:
            resp = f
            break
    if resp is None:
        print("  ✗ 未收到 LINK_STAT（固件可能不支持，需 Rebuild）")
        return False

    p = resp["payload"]
    if len(p) < 26:
        print("  ✗ 响应长度异常：%d" % len(p))
        return False

    up      = struct.unpack("<I", p[0:4])[0]
    good    = struct.unpack("<I", p[4:8])[0]
    bad     = struct.unpack("<I", p[8:12])[0]
    rx      = struct.unpack("<I", p[12:16])[0]
    ovf     = struct.unpack("<I", p[16:20])[0]
    age     = struct.unpack("<I", p[20:24])[0]
    fault   = p[24]
    lost    = p[25]

    print("  uptime        : %d.%03d s" % (up // 1000, up % 1000))
    print("  好帧 / 坏帧   : %d / %d" % (good, bad))
    print("  收到字节      : %d" % rx)
    print("  环形缓冲溢出  : %d" % ovf)
    if age == 0xFFFFFFFF:
        print("  距上一帧      : 从未收到过！")
    else:
        print("  距上一帧      : %d ms" % age)
    print("  fault         : 0x%02X(%s)   comm_lost=%d"
          % (fault, FAULT.get(fault, "?"), lost))
    return True


def read_stepper_stat(link: Link):
    """读一次电机状态，返回 dict（静默，不打印）。

    这是耐久测试的【真值来源】：位置寄存器不会丢，
    而 MOTION_DONE 事件可能因为时序/丢帧而收不到。
    """
    seq = link.send(TYPE_CMD, OP_STEPPER_STAT)
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        for f in link.recv(0.05):
            if f["op"] == OP_STEPPER_STAT and f["seq"] == seq:
                p = f["payload"]
                if len(p) < 24:
                    return None
                return {
                    "L": {"pos": struct.unpack("<i", p[0:4])[0],
                          "rem": struct.unpack("<i", p[4:8])[0],
                          "hz": struct.unpack("<H", p[8:10])[0],
                          "flags": p[10]},
                    "R": {"pos": struct.unpack("<i", p[11:15])[0],
                          "rem": struct.unpack("<i", p[15:19])[0],
                          "hz": struct.unpack("<H", p[19:21])[0],
                          "flags": p[21]},
                    "trips": p[22],
                    "fault": p[23],
                }
    return None


def ask_stepper_stat(link: Link) -> bool:
    """诊断：读回两个电机的实时状态。

    用途：区分"发 MOVE 后电机只动一下"到底是什么原因：
      - remaining > 0 且 running=0  -> 被热保护/联锁停在中途
      - remaining = 0 且 position 没涨 -> 根本没启动
      - cooling=1 -> 还在冷却
      - trips 增长 -> 触发了多少次热跳闸
    """
    print("\n[STEPPER_STAT] 电机实时状态")
    link.drain()
    seq = link.send(TYPE_CMD, OP_STEPPER_STAT)
    frames = link.recv(1.0)

    resp = None
    for f in frames:
        if f["op"] == OP_STEPPER_STAT and f["seq"] == seq:
            resp = f
            break
    if resp is None:
        print("  ✗ 未收到 STEPPER_STAT（固件可能不支持，需 Rebuild）")
        return False

    p = resp["payload"]
    if len(p) < 24:
        print("  ✗ 响应长度异常：%d" % len(p))
        return False

    for i, side in ((0, "L"), (1, "R")):
        off = i * 11
        pos = struct.unpack("<i", p[off:off + 4])[0]
        rem = struct.unpack("<i", p[off + 4:off + 8])[0]
        hz = struct.unpack("<H", p[off + 8:off + 10])[0]
        fl = p[off + 10]
        print("  %s: position=%7d  remaining=%7d  hz=%3d  %s%s"
              % (side, pos, rem, hz,
                 "RUNNING" if fl & 0x01 else "stopped",
                 "  COOLING" if fl & 0x02 else ""))

    print("  thermal_trips=%d   fault=0x%02X(%s)"
          % (p[22], p[23], FAULT.get(p[23], "?")))
    return True


def show_caps(link: Link):
    """打印板子里固件实际实现了哪些命令。

    这是排查"我改了代码但怎么没反应"最快的手段：
    如果 impl 里没有 SERVO，那就是固件没刷进去，跟接线无关。
    """
    print("\n[CAPS] 查询固件能力")
    link.drain()
    seq = link.send(TYPE_CMD, OP_PING)
    frames = link.recv(1.0)

    pong = None
    for f in frames:
        if f["op"] == OP_PONG and f["seq"] == seq:
            pong = f
            break
    if pong is None:
        print("  ✗ 未收到 PONG")
        return

    p = pong["payload"]
    up_ms = struct.unpack("<I", p[:4])[0]
    cfg = struct.unpack("<I", p[5:9])[0]

    print("  uptime : %d.%03d s" % (up_ms // 1000, up_ms % 1000))
    print("  config : 0x%08X %s" % (cfg, cfg_str(cfg)))

    if len(p) < 10:
        print("  flags  : ??   ← PONG 只有 %d 字节" % len(p))
    else:
        flags = p[9]
        print("  flags  : 0x%02X  src=%s  blind=%s"
              % (flags,
                 "FLASH" if flags & 0x01 else "DEFAULT",
                 "YES" if flags & 0x02 else "NO"))

    if len(p) < 11:
        print("  impl   : ??   ← 旧固件（M0~M2.5 或更早），PONG 无 impl 字节")
        print()
        print("  → 板子里的固件很可能是旧的，请在 Keil 里 Rebuild + Load")
        return

    impl = p[10]
    print("  impl   : 0x%02X %s" % (impl, impl_str(impl)))
    print()
    print("  逐项：")
    for bit, name in IMPL_BITS:
        mark = "✔" if (impl & bit) else "✘"
        print("    %s %-12s (bit%d)" % (mark, name, bit.bit_length() - 1))

    print()
    if impl & (1 << 0):
        print("  → SERVO 已实现，可执行 sv <deg> [ms]")
    else:
        print("  → SERVO 未实现：固件是旧的，先 Rebuild + Load")


def servo_sweep(link: Link):
    """舵机扫描：0 -> -45 -> +45 -> 0，用来肉眼确认行程和缓动。"""
    print("\n[SERVO 扫描] 0 -> -45 -> +45 -> 0（每段 600 ms 缓动）")
    for deg in (0, -45, 45, 0):
        ask_servo(link, deg, 600)
        time.sleep(0.75)
    print("  扫描完成")


def run_selftest(link: Link) -> int:
    """按 M1/M2/M2.5 的判据逐项自检。返回失败项数。"""
    print("=" * 60)
    print(" M1 / M2 / M2.5 自检")
    print("=" * 60)

    fails = 0
    if not ask_ping(link):
        fails += 1

    cur = ask_get_config(link)
    if cur is None:
        fails += 1
        cur = None

    if cur is not None:
        # 直接调 ask_set_config（它自己会打印标题，不重复打印）
        newcfg = cur ^ (1 << 1)          # 翻转 CFG_TTS_EN
        print("\n（翻转 TTS 位，用来验证 Flash 持久化）")
        if not ask_set_config(link, newcfg):
            fails += 1
        back = ask_get_config(link)
        if back != newcfg:
            print("  ✗ 回读不一致：期望 0x%08X 实际 %s" % (newcfg, back))
            fails += 1
        else:
            print("  ✓ 回读一致")
        # 恢复原值
        print("\n（恢复原配置）")
        if not ask_set_config(link, cur):
            fails += 1

    if not ask_not_implemented(link):
        fails += 1
    if not ask_bad_crc(link):
        fails += 1
    if not ask_clear_fault(link, ack=True):
        fails += 1

    print("\n" + "=" * 60)
    if fails == 0:
        print(" 自检全部通过（M1 / M2 / M2.5 的 PC 侧判据满足）")
        print(" 还需要人工确认：")
        print("   · M0：板载 LED 是否 1 Hz 闪烁")
        print("   · M2：断开串口 1 秒后 LED 是否变快闪")
        print("   · M2.5：**拔掉 USB 重新上电后 GET_CONFIG 是否仍是新值**")
    else:
        print(" 自检有 %d 项失败，请回传上方完整输出" % fails)
    print("=" * 60)
    return fails


def repl(link: Link):
    print("\n交互模式。命令：")
    print("  p                 PING")
    print("  v                 查看固件能力（实现了哪些命令）★ 刷没刷固件一看就知")
    print("  g                 GET_CONFIG")
    print("  s <hex32>         SET_CONFIG，例如 s 0000007A")
    print("  sv <deg> [ms]     SERVO 舵机（deg 有符号 -45..+45，ms 省略=立即）")
    print("  sc                舵机扫描 -45 -> +45 -> 0（体验缓动）")
    print("  mv <dir> [spd] [steps]  MOVE  0进 1退 2左转 3右转 4停")
    print("  fwd [steps] [spd]       前进（默认 200 半步 @300Hz）")
    print("  bck [steps] [spd]       后退")
    print("  lft [steps] [spd]       原地左转")
    print("  rgt [steps] [spd]       原地右转")
    print("  st                      立即停止（独立停止命令，无条件生效）")
    print("  stc                     立即停止 + 清除热保护冷却")
    print("  mtsel                   跑一遍 M4/M5 运动自检")
    print("  end [次数] [步数]       耐久测试（连发 MOVE，核对位置丢步）")
    print("  sst                     读回电机实时状态（诊断卡住问题）")
    print("  lstat                   读回链路健康计数器（诊断丢帧/心跳）")
    print("  tick                    测量 1ms 时基是否准确（10 秒）")
    print("  cal                     方向标定：依次跑 fwd/bck/lft/rgt")
    print("  dcal                    方向标定（单侧驱动，最可靠，推荐）")
    print("  ml <steps>              只驱动左电机（原始方向，有符号）")
    print("  mr <steps>              只驱动右电机（原始方向，有符号）")
    print("  c [ack]           CLEAR_FAULT")
    print("  t <text>          TTS 原样转发（当前阶段返回 NAK）")
    print("  raw <hex...>      直接发原始字节")
    print("  hb               开始 500ms 心跳（Ctrl+C 退出）")
    print("  q                 退出")
    while True:
        try:
            line = input("\ncmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd == "q":
                return
            elif cmd == "p":
                ask_ping(link)
            elif cmd == "v":
                show_caps(link)
            elif cmd == "g":
                ask_get_config(link)
            elif cmd == "s" and len(parts) >= 2:
                ask_set_config(link, int(parts[1], 16))
            elif cmd == "sv" and len(parts) >= 2:
                deg = int(parts[1])
                ms = int(parts[2]) if len(parts) >= 3 else 0
                ask_servo(link, deg, ms)
            elif cmd == "sc":
                servo_sweep(link)
            elif cmd == "mv" and len(parts) >= 2:
                d = int(parts[1])
                sp = int(parts[2]) if len(parts) >= 3 else 2
                stp = int(parts[3]) if len(parts) >= 4 else 200
                ask_move(link, d, sp, stp)
            elif cmd in ("fwd", "bck", "lft", "rgt"):
                dmap = {"fwd": 0, "bck": 1, "lft": 2, "rgt": 3}
                stp = int(parts[1]) if len(parts) >= 2 else 200
                sp = int(parts[2]) if len(parts) >= 3 else 2
                ask_move(link, dmap[cmd], sp, stp)
            elif cmd == "st":
                ask_motor_stop(link, clear_cooldown=False)
            elif cmd == "stc":
                ask_motor_stop(link, clear_cooldown=True)
            elif cmd == "mtsel":
                run_move_selftest(link)
            elif cmd == "sst":
                ask_stepper_stat(link)
            elif cmd == "lstat":
                ask_link_stat(link)
            elif cmd == "tick":
                run_tick_test(link)
            elif cmd == "cal":
                run_calibration(link)
            elif cmd == "dcal":
                run_dir_calibration(link)
            elif cmd == "ml" and len(parts) >= 2:
                ask_single_motor(link, 0, int(parts[1]))
            elif cmd == "mr" and len(parts) >= 2:
                ask_single_motor(link, 1, int(parts[1]))
            elif cmd == "end":
                n = int(parts[1]) if len(parts) >= 2 else 10
                s = int(parts[2]) if len(parts) >= 3 else 2400
                run_endurance(link, n, s)
            elif cmd == "m":
                ask_not_implemented(link)
            elif cmd == "c":
                ask_clear_fault(link, ack=(len(parts) > 1 and parts[1] == "ack"))
            elif cmd == "t" and len(parts) >= 2:
                text = " ".join(parts[1:]).encode("utf-8")
                seq = link.send(TYPE_CMD, OP_TTS, text)
                link.recv(1.0)
            elif cmd == "hb":
                print("心跳中，Ctrl+C 停止")
                try:
                    while True:
                        seq = link.send(TYPE_CMD, OP_PING, seq=link.seq)
                        link.recv(0.3)
                        time.sleep(0.5)
                except KeyboardInterrupt:
                    print("停止心跳")
            elif cmd == "raw":
                data = bytes(int(x, 16) for x in parts[1:])
                link.ser.write(data)
                print("  --> TX %s" % data.hex(" "))
                link.recv(0.5)
            else:
                print("未知命令")
        except ValueError as e:
            print("参数错误：%s" % e)


def verify_boot(link: Link) -> int:
    """判断"配置是否真的从 Flash 加载"。

    背景：早期判据靠"看开机横幅"或"uptime 归零"，很容易误判——
    例如只拔了 USB-TTL 的 USB 线，而 MCU 由 ST-Link 供电时，
    板子根本没重启，uptime 会继续累加，看起来"配置保住了"其实是 RAM 里的值。

    本命令用 PONG 的 flags bit0（固件在 Config_Load 后置位）来判定，
    并把 uptime 一起打出来作为交叉验证。
    """
    print("=" * 60)
    print(" 启动状态核对（判断配置是否来自 Flash）")
    print("=" * 60)

    link.drain()
    seq = link.send(TYPE_CMD, OP_PING)
    frames = link.recv(1.0)

    pong = None
    for f in frames:
        if f["op"] == OP_PONG and f["seq"] == seq:
            pong = f
            break
    if pong is None:
        print("  ✗ 未收到 PONG")
        return 1

    p = pong["payload"]
    up_ms = struct.unpack("<I", p[:4])[0]
    cfg = struct.unpack("<I", p[5:9])[0]
    has_flags = len(p) >= 10
    flags = p[9] if has_flags else None
    impl = p[10] if len(p) >= 11 else None

    print("  uptime   : %d.%03d s" % (up_ms // 1000, up_ms % 1000))
    print("  config   : 0x%08X %s" % (cfg, cfg_str(cfg)))
    if impl is None:
        print("  impl     : ??  [旧固件：PONG 无 impl 字节]")
    else:
        print("  impl     : 0x%02X %s" % (impl, impl_str(impl)))

    fails = 0

    if not has_flags:
        print("  ⚠ 固件仍是旧版（PONG 无 flags 字节），无法自动判定")
        print("    请重新编译烧录最新固件后再试")
        return 1

    print("  flags    : 0x%02X" % flags)
    print("    bit0 配置来源 : %s" % ("Flash" if flags & 0x01 else "默认值"))
    print("    bit1 盲走模式 : %s" % ("YES" if flags & 0x02 else "NO"))
    print()

    # 判据 1：uptime 必须很小，否则说明根本没重启，测试无效
    if up_ms > 60000:
        print("  ✗ uptime 已达 %d s，说明板子【没有重启过】" % (up_ms // 1000))
        print("    → 请按板上的 RESET 键，或断开 MCU 的【全部】电源后重新上电")
        print("    → 只拔 USB-TTL 的 USB 线不会让 MCU 断电")
        fails += 1
    else:
        print("  ✓ uptime 较小（%d.%03d s），看起来是刚启动"
              % (up_ms // 1000, up_ms % 1000))

    # 判据 2：配置必须来自 Flash
    if flags & 0x01:
        print("  ✓ 配置来自 Flash → 持久化成功")
    else:
        print("  ✗ 配置来自默认值 → Flash 持久化失败")
        print("    → 若这颗 MCU 是 32 KB 假片，0x0800FC00 超出物理 Flash")
        fails += 1

    print()
    if fails == 0:
        print(" 结论：FLASH 持久化 通过")
    else:
        print(" 结论：判定失败（%d 项）" % fails)
    print("=" * 60)
    return fails


def main():
    ap = argparse.ArgumentParser(
        description="A733 桌宠下位机 PC 端链路联调工具")
    ap.add_argument("port", help="串口名，如 COM4 或 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    ap.add_argument("--selftest", action="store_true", help="一键自检（M1/M2/M2.5）")
    ap.add_argument("--ping", action="store_true", help="只发一次 PING")
    ap.add_argument("--dcal", action="store_true",
                    help="方向标定：单侧驱动（最可靠）")
    ap.add_argument("--cal", action="store_true",
                    help="方向标定：依次跑四个方向")
    ap.add_argument("--tick-test", action="store_true",
                    help="测量下位机 1ms 时基是否准确（10 秒）")
    ap.add_argument("--endurance", type=int, default=0, metavar="N",
                    help="耐久测试：连发 N 次 MOVE 并核对位置")
    ap.add_argument("--move-selftest", action="store_true",
                    help="M4/M5 运动自检")
    ap.add_argument("--verify-boot", action="store_true",
                    help="核对是否真的重启过、配置是否来自 Flash")
    ap.add_argument("--quiet", action="store_true", help="不打印原始 hex")
    args = ap.parse_args()

    try:
        link = Link(args.port, args.baud, verbose=not args.quiet)
    except Exception as e:
        print("打开串口失败：%s" % e)
        print("常见原因：串口被串口助手 / Keil Debug 等程序独占，先关掉它们")
        sys.exit(2)

    try:
        if args.ping:
            ask_ping(link)
        elif args.verify_boot:
            sys.exit(0 if verify_boot(link) == 0 else 1)
        elif args.move_selftest:
            sys.exit(0 if run_move_selftest(link) == 0 else 1)
        elif args.dcal:
            sys.exit(run_dir_calibration(link))
        elif args.cal:
            sys.exit(run_calibration(link))
        elif args.tick_test:
            sys.exit(run_tick_test(link))
        elif args.endurance:
            sys.exit(0 if run_endurance(link, args.endurance) == 0 else 1)
        elif args.selftest:
            sys.exit(0 if run_selftest(link) == 0 else 1)
        else:
            repl(link)
    finally:
        link.close()


if __name__ == "__main__":
    main()
