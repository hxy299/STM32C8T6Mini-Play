#!/usr/bin/env bash
# ============================================================================
#  tools/check_firmware_syntax.sh
#
#  用最小 SPL 桩在 PC 上做【纯语法检查】，不需要 Keil、不需要 ARM 工具链。
#
#  它不能替代 Keil 编译（不产生目标码、不检查链接），
#  但能在打开 Keil 之前挡掉绝大多数低级错误：
#    拼写错误、未声明函数、结构体字段写错、漏分号、类型不匹配。
#
#  用法：  bash tools/check_firmware_syntax.sh
#  退出码：失败文件数
# ============================================================================
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT" || exit 1

CC="${CC:-gcc}"
INC="-I tools/stub -I User -I Hardware -I Protocol -I Service -I BSP -I System"
FLAGS="-std=c99 -Wall -Wextra -Wno-unused-parameter -fsyntax-only"

FILES="
User/main.c
User/stm32f10x_it.c
BSP/bsp.c
Hardware/serial.c
Hardware/servo.c
Hardware/stepper.c
Hardware/i2c1.c
Hardware/tof.c
Hardware/tts.c
Protocol/link_proto.c
Service/config.c
Service/link_service.c
Service/tof_service.c
"

FAIL=0
OK=0

echo "============================================================"
echo " 固件语法检查（最小 SPL 桩，不等价于 Keil 编译）"
echo "============================================================"

for f in $FILES; do
  if [ ! -f "$f" ]; then
    printf '  %-32s \033[33m跳过（文件不存在）\033[0m\n' "$f"
    continue
  fi
  if $CC $FLAGS $INC "$f" 2> /tmp/stm_syntax.err; then
    printf '  OK  %-28s\n' "$f"
    OK=$((OK + 1))
  else
    printf '  \033[31mERR\033[0m %-28s\n' "$f"
    sed 's/^/        /' /tmp/stm_syntax.err | head -12
    FAIL=$((FAIL + 1))
  fi
done

echo "------------------------------------------------------------"
printf ' %d 通过 / %d 失败\n' "$OK" "$FAIL"
echo "------------------------------------------------------------"
[ "$FAIL" -eq 0 ] && echo " 全部通过" || echo " 有文件未通过"
exit "$FAIL"
