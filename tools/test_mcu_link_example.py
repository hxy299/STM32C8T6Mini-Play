#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/mcu_link_example.py 的端到端回归测试。

做法：
  用 pty 造一对虚拟串口，让 C 示例连 slave 端，
  本脚本在 master 端扮演 MCU，真的按协议收发。

  这样能在没有硬件的情况下验证：
    · 组帧 / CRC 是否与固件一致
    · SEQ 配对是否正确（查询命令的应答会回显请求 OP）
    · 心跳节奏是否 ≈300 ms
    · MOVE 的应答解析（两个字节：status + 距离）
    · 事件（TOF_DATA / MOTION_DONE）能否被正确打印

只依赖 Python3 标准库（pty / select / subprocess），不需要 pyserial。

用法：
    gcc -std=c99 -Wall -Wextra -O2 -o /tmp/mcu_link tools/mcu_link_example.c
    python3 tools/test_mcu_link_example.py /tmp/mcu_link
"""
import os
import pty
import re
import select
import subprocess
import sys
import time

# ---------------------------------------------------------------- 协议
SOF = 0xA5
TYPE_CMD, TYPE_ACK, TYPE_EVT = 0x01, 0x02, 0x03

OP_MOVE, OP_SERVO, OP_TTS, OP_TOF_CFG = 0x01, 0x02, 0x03, 0x04
OP_CLEAR_FAULT, OP_PING, OP_SET_CONFIG, OP_GET_CONFIG = 0x05, 0x06, 0x07, 0x08
OP_STEPPER_STAT, OP_LINK_STAT, OP_STEPPER_TEST, OP_MOTOR_STOP = (
    0x09, 0x0A, 0x0B, 0x0C)

OP_ACK, OP_NAK, OP_PONG, OP_CONFIG = 0x81, 0x82, 0x86, 0x87
OP_EVT_EDGE, OP_EVT_TOF_DATA, OP_EVT_FAULT = 0x91, 0x92, 0x93
OP_EVT_MOTION_DONE, OP_EVT_OBSTACLE = 0x94, 0x95

CONFIG_2TOF = 0x4E
IMPL_ALL = 0x3F


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(ftype, seq, op, payload=b""):
    body = bytes([SOF, ftype, seq & 0xFF, (seq >> 8) & 0xFF, op, len(payload)]) + payload
    crc = crc16_ccitt_false(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class Parser:
    """极简拆帧器，只接受 TYPE_CMD（因为测的是上位机发出来的帧）。"""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0
        self.good = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(bytes([SOF]))
            if i < 0:
                self.buf.clear()
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < 6:
                break
            ln = self.buf[5]
            total = 8 + ln
            if ln > 200:
                self.bad += 1
                del self.buf[:1]
                continue
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            crc = crc16_ccitt_false(frame[:6 + ln])
            if crc != (frame[6 + ln] | (frame[7 + ln] << 8)):
                self.bad += 1
                del self.buf[:1]
                continue
            del self.buf[:total]
            self.good += 1
            out.append({
                "type": frame[1],
                "seq": frame[2] | (frame[3] << 8),
                "op": frame[4],
                "payload": frame[6:6 + ln],
            })
        return out


# ---------------------------------------------------------------- 假 MCU
class MockMcu:
    def __init__(self, fd):
        self.fd = fd
        self.parser = Parser()
        self.pings = []          # 每次 PING 的到达时间
        self.moves = []
        self.other = []
        self.motion_sent = False
        self.motion_at = None

    def reply(self, seq, op, payload=b""):
        os.write(self.fd, build(TYPE_ACK, seq, op, payload))

    def event(self, op, payload=b""):
        os.write(self.fd, build(TYPE_EVT, 0, op, payload))

    def handle(self, f):
        t = time.monotonic()
        op, seq = f["op"], f["seq"]

        if op == OP_PING:
            self.pings.append(t)
            # uptime(4) fault(1) config(4) flags(1) impl(1)
            body = bytes([0x10, 0x27, 0x00, 0x00,      # uptime = 10000
                          0x00,                          # fault = NONE
                          CONFIG_2TOF, 0, 0, 0,
                          0x00,                          # flags
                          IMPL_ALL])
            self.reply(seq, OP_PONG, body)

        elif op == OP_GET_CONFIG:
            self.reply(seq, OP_CONFIG, bytes([CONFIG_2TOF, 0, 0, 0]))

        elif op == OP_MOVE:
            self.moves.append((t, f["payload"]))
            if f["payload"][0] == 0x04:
                self.reply(seq, OP_ACK, bytes([0x00]))
            else:
                # status + 预计距离(mm/10)，2000 步会饱和成 0xFF
                self.reply(seq, OP_ACK, bytes([0x00, 0xFF]))
                if self.motion_at is None:
                    # 故意延到 1.2 秒：让心跳跑满几轮，才能验间隔
                    self.motion_at = t + 1.2
                # 紧跟着再推一条 TOF 数据。
                # 这两帧很可能被一次 read() 读进同一批 —— 正好用来验证
                # 上位机不会"返回第一帧就把这批里剩下的丢掉"。
                self.event(OP_EVT_TOF_DATA,
                           bytes([0x09]) +
                           bytes([25, 0]) + bytes([0xFF, 0xFF]) +
                           bytes([0xFF, 0xFF]) + bytes([0x2C, 0x01]))

        elif op == OP_MOTOR_STOP:
            self.reply(seq, OP_ACK, bytes([0x00]))

        else:
            self.other.append(op)
            self.reply(seq, OP_NAK, bytes([0x05, op]))

    def tick(self):
        """到点就主动推一条 MOTION_DONE。"""
        if self.motion_at and not self.motion_sent and time.monotonic() >= self.motion_at:
            self.motion_sent = True
            # motor(1) + position(int32 LE) = 2000
            self.event(OP_EVT_MOTION_DONE, bytes([0]) + bytes([0xD0, 0x07, 0x00, 0x00]))

    def pump(self, timeout):
        self.tick()
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return
        try:
            data = os.read(self.fd, 4096)
        except OSError:
            return
        for f in self.parser.feed(data):
            if f["type"] == TYPE_CMD:
                self.handle(f)


# ---------------------------------------------------------------- 跑一次
def run_case(binary, mode, deadline_s, expect_motion, expect_heartbeat):
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    proc = subprocess.Popen([binary, path, mode],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    mcu = MockMcu(master)
    t_end = time.monotonic() + deadline_s

    while time.monotonic() < t_end:
        mcu.pump(0.02)
        if proc.poll() is not None:
            break
    if proc.poll() is None:
        proc.kill()

    out = proc.stdout.read().decode("utf-8", "replace")
    rc = proc.wait()
    os.close(master)
    os.close(slave)

    checks = []

    def chk(cond, name):
        checks.append((bool(cond), name))

    chk(rc == 0, "进程退出码 0（实际 %s）" % rc)
    chk("PONG uptime=10000" in out, "解析出 PONG 的 uptime")
    chk("config=0x0000004E" in out, "解析出 config=0x4E")
    chk("impl=0x3F" in out, "解析出 impl=0x3F")
    chk(mcu.parser.good > 0 and mcu.parser.bad == 0,
        "MCU 侧收到的帧全部 CRC 正确（good=%d bad=%d）"
        % (mcu.parser.good, mcu.parser.bad))
    chk(len(mcu.pings) >= 1, "至少发了 1 次 PING（实际 %d）" % len(mcu.pings))
    chk(not mcu.other, "没有未知操作码（实际 %s）" % (mcu.other or "无"))

    # 心跳间隔：允许一定抖动，但中位数必须在 300ms 附近。
    # 只在长跑用例里检查 —— --probe 两个命令就结束了，本来就用不着心跳。
    if expect_heartbeat and len(mcu.pings) >= 3:
        gaps = sorted(b - a for a, b in zip(mcu.pings, mcu.pings[1:]))
        mid = gaps[len(gaps) // 2]
        chk(0.26 <= mid <= 0.40,
            "心跳间隔中位数 %.0f ms（目标 300）" % (mid * 1000))
    elif expect_heartbeat:
        chk(False, "心跳次数不足 3 次，无法检查间隔（实际 %d）"
            % len(mcu.pings))
    else:
        chk(True, "短命令用例，跳过心跳间隔检查")

    if expect_motion:
        chk(len(mcu.moves) >= 1, "收到了 MOVE")
        if mcu.moves:
            p = mcu.moves[0][1]
            chk(p[0] == 0x00, "MOVE 方向=前进")
            chk(p[1] == 0x02, "MOVE 档位=2")
            chk((p[2] | (p[3] << 8)) == 2000, "MOVE 步数=2000")
        chk("预计距离 > 25.4 mm" in out, "正确识别距离饱和")
        chk("TOF mask=0x09" in out, "收到并解析 TOF_DATA 事件")
        chk("d0=25" in out, "TOF d0 读数正确")
        chk("d1=-1" in out, "TOF d1（未启用）显示为 -1")
        chk(mcu.motion_sent, "MCU 侧确实发出了 MOTION_DONE")
        chk("走完" in out, "上位机收到并打印 MOTION_DONE")
        chk("已停止" in out, "退出前调用了 MOTOR_STOP")

    return checks, out, mcu


def main():
    if len(sys.argv) < 2:
        print("用法: %s <mcu_link 可执行文件>" % sys.argv[0])
        return 2

    binary = sys.argv[1]
    total = 0
    failed = 0

    for mode, deadline, motion, hb in (("--probe", 6.0, False, False),
                                       ("--demo", 12.0, True, True)):
        print("=" * 62)
        print(" 用例: %s" % mode)
        print("=" * 62)
        checks, out, mcu = run_case(binary, mode, deadline, motion, hb)
        for ok, name in checks:
            total += 1
            if ok:
                print("  [PASS] %s" % name)
            else:
                failed += 1
                print("  [FAIL] %s" % name)
        if failed:
            print("\n  ---- 程序输出 ----")
            for line in out.splitlines():
                print("  | %s" % line)
        print()

    print("-" * 62)
    print(" 结果: %d PASS / %d FAIL" % (total - failed, failed))
    print("-" * 62)
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
