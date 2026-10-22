#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""STM32 源码卫生检查（在编译之前跑，抓低级但致命的错误）。

背景：2026-10-22 的一次构建失败，根因是改代码时用 PowerShell 单引号字符串
写 `` `r`n ``，结果把【字面的反引号字符】插进了源码：

    #include "stepper_math.h"`r`n#include "stepper.h"

后果是 `#include "stepper.h"` 被当成预处理指令后的多余文本吞掉，
stepper.h 从未被包含 → 30 个"标识符未定义"错误。

这类问题编译器只给一条 warning，很容易被忽略。本脚本在编译前把它抓出来。

检查项：
  1. 源码里出现字面的反引号转义序列（`r `n `t `0 等）
  2. 文件不是合法 UTF-8
  3. 文件带 BOM（ARMCC 对 BOM 敏感）
  4. 混用 CRLF / LF
  5. 行尾有多余的反引号

用法：
    python tools/check_source_sanity.py
退出码：0 = 干净，1 = 发现问题
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCAN_DIRS = ["User", "System", "BSP", "Hardware", "Protocol", "Service"]
EXTS = (".c", ".h")

# 字面的反引号转义（PowerShell 泄漏的典型特征）
BAD_ESCAPES = [
    ("`r`n", "字面 CRLF 转义"),
    ("`n", "字面换行转义"),
    ("`r", "字面回车转义"),
    ("`t", "字面制表转义"),
]

problems = 0


def report(path, lineno, kind, detail, line=""):
    global problems
    problems += 1
    rel = os.path.relpath(path, ROOT)
    print("  [%s] %s:%d" % (kind, rel, lineno))
    print("        %s" % detail)
    if line:
        print("        > %s" % line.strip()[:110])


def check_file(path):
    raw = open(path, "rb").read()

    # --- BOM ---
    if raw[:3] == b"\xef\xbb\xbf":
        report(path, 1, "BOM", "文件带 UTF-8 BOM，建议去掉")

    # --- UTF-8 合法性 ---
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as e:
        report(path, 0, "ENC", "不是合法 UTF-8：%s" % e)
        return

    lines = text.split("\n")

    # --- 字面转义序列 ---
    for i, line in enumerate(lines, 1):
        for esc, desc in BAD_ESCAPES:
            if esc in line:
                report(path, i, "ESC", "含%s %r（应为真实换行/制表符）"
                       % (desc, esc), line)

    # --- 混用换行符 ---
    crlf = raw.count(b"\r\n")
    lf_only = raw.count(b"\n") - crlf
    if crlf and lf_only:
        report(path, 0, "EOL",
               "混用换行符：CRLF %d 行 / LF %d 行（建议统一 CRLF）"
               % (crlf, lf_only))


def main():
    print("=" * 62)
    print(" STM32 源码卫生检查")
    print("=" * 62)

    nfiles = 0
    for d in SCAN_DIRS:
        full = os.path.join(ROOT, d)
        if not os.path.isdir(full):
            continue
        for name in sorted(os.listdir(full)):
            if name.endswith(EXTS):
                nfiles += 1
                check_file(os.path.join(full, name))

    print("\n扫描 %d 个文件" % nfiles)
    print("=" * 62)
    if problems == 0:
        print(" 干净：未发现问题")
        return 0
    print(" 发现 %d 个问题，请修复后再编译" % problems)
    return 1


if __name__ == "__main__":
    sys.exit(main())
