#!/usr/bin/env bash
# ============================================================================
#  tools/run_host_tests.sh
#
#  在 PC 上编译并运行全部纯逻辑单元测试（不需要 STM32 硬件）。
#
#  用法：  bash tools/run_host_tests.sh
#  依赖：  gcc（WSL / Linux / MSYS2 均可）
#
#  说明：
#    · 只测不碰寄存器的纯函数（*_math.h、cmd_router.h 这类），
#      因为它们才是"编译通过但上板就错"的高发区。
#    · 每个测试是独立 TU，失败不影响其它测试继续跑。
#    · 退出码 = 失败用例总数，方便 CI 直接判。
# ============================================================================
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT" || exit 1

CC="${CC:-gcc}"
CFLAGS="-std=c99 -Wall -Wextra -Werror -O2 -I tools/hosttest -I Hardware -I User -I Protocol -I Service -I BSP"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

TOTAL_PASS=0
TOTAL_FAIL=0
BUILD_FAIL=0
RAN=0

run_one() {
  local src="$1"
  local extra="${2:-}"
  local name
  name="$(basename "$src" .c)"
  RAN=$((RAN + 1))

  if ! $CC $CFLAGS "$src" $extra -o "$OUT/$name" 2>"$OUT/$name.err"; then
    printf '  %-32s \033[31m编译失败\033[0m\n' "$src"
    sed 's/^/      /' "$OUT/$name.err" | head -20
    BUILD_FAIL=$((BUILD_FAIL + 1))
    return
  fi

  local log
  log="$("$OUT/$name" 2>&1)"
  local p f
  p="$(printf '%s\n' "$log" | sed -n 's/.*结果: \([0-9]\+\) PASS.*/\1/p' | tail -1)"
  f="$(printf '%s\n' "$log" | sed -n 's/.*PASS \/ \([0-9]\+\) FAIL.*/\1/p' | tail -1)"
  p="${p:-0}"; f="${f:-0}"

  if [ "$f" -eq 0 ]; then
    printf '  %-32s 结果: %s PASS / %s FAIL\n' "$src" "$p" "$f"
  else
    printf '  %-32s \033[31m结果: %s PASS / %s FAIL\033[0m\n' "$src" "$p" "$f"
    printf '%s\n' "$log" | grep -E '\[FAIL\]' | sed 's/^/      /'
  fi

  TOTAL_PASS=$((TOTAL_PASS + p))
  TOTAL_FAIL=$((TOTAL_FAIL + f))
}

# 需要额外链接实现文件的测试（测的是真实实现，不只是纯头文件）
extra_sources_for() {
  case "$(basename "$1")" in
    test_link_proto.c) echo "Protocol/link_proto.c" ;;
    *)                 echo "" ;;
  esac
}

echo "============================================================"
echo " STM32 下位机 —— 主机纯逻辑单测"
echo " root: $ROOT"
echo " cc  : $($CC --version 2>/dev/null | head -1)"
echo "============================================================"

for src in tools/hosttest/test_*.c; do
  [ -e "$src" ] || continue
  run_one "$src" "$(extra_sources_for "$src")"
done

echo "------------------------------------------------------------"
printf ' 共 %d 个测试文件：%d PASS / %d FAIL' "$RAN" "$TOTAL_PASS" "$TOTAL_FAIL"
[ "$BUILD_FAIL" -gt 0 ] && printf ' / %d 编译失败' "$BUILD_FAIL"
echo
echo "------------------------------------------------------------"

if [ "$TOTAL_FAIL" -ne 0 ] || [ "$BUILD_FAIL" -ne 0 ]; then
  echo " 结果: FAIL"
  exit $((TOTAL_FAIL + BUILD_FAIL))
fi
echo " 结果: ALL PASS"
exit 0
