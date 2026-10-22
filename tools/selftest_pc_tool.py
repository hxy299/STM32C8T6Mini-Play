# -*- coding: utf-8 -*-
"""pc_link_test.py 的自检：核对组帧与固件是否逐字节一致（不需要硬件）。

用法：python tools/selftest_pc_tool.py
"""
import importlib.util
import struct
import sys
import os

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "pc_link_test", os.path.join(HERE, "pc_link_test.py"))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

CASES = [
    ("PING seq=7",          m.build_frame(1, 7, 6, b""),
     "A50107000600E3FD"),
    ("GET_CONFIG seq=0",    m.build_frame(1, 0, 8, bytes([1, 2, 3, 4])),
     "A5010000080401020304149C"),
    ("RT seq=0x1234 op=6",  m.build_frame(1, 0x1234, 6, bytes([1, 2, 3, 4, 5])),
     "A50134120605010203040547AB"),
]

ok = True
print("=== 组帧向量核对（对照 STM32 侧 link_proto.c 的实测输出）===")
for name, got, want in CASES:
    g = got.hex().upper()
    good = (g == want)
    ok = ok and good
    print("  %-22s %s  %s" % (name, "PASS" if good else "FAIL", g))
    if not good:
        print("  %-22s       %s  <- expected" % ("", want))

print()
print("=== CRC16-CCITT-FALSE 标准检验值 ===")
crc = m.crc16_ccitt_false(b"123456789")
good = (crc == 0x29B1)
ok = ok and good
print("  CRC16(\"123456789\") = 0x%04X  %s (expect 0x29B1)"
      % (crc, "PASS" if good else "FAIL"))

print()
print("=== PONG 解析（uptime 单位换算）===")
pl = struct.pack("<IB", 191651, 0) + struct.pack("<I", 0x7A)
frame = {"type": 2, "seq": 1, "op": m.OP_PONG, "payload": pl, "raw": b""}
print("  " + m.describe(frame))
good = "191.651s" in m.describe(frame)
ok = ok and good
print("  uptime 换算 %s" % ("PASS" if good else "FAIL"))

print()
print("结果:", "ALL PASS" if ok else "HAS FAILURE")
sys.exit(0 if ok else 1)
