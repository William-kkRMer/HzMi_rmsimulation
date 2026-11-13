#!/usr/bin/env bash
# 在一个"能力集正常"的用户会话里跑 net_diag_livox.sh
#
# 背景：如果当前终端是从受限会话（CapEff=0）里起的，ethtool 会报
#   "netlink error: Operation not permitted"
# 这类读数就不干净。用 systemd-run --user 起一个独立实例可绕开（不需要 sudo）。
#
# 用法:
#   ./net_diag_via_systemd.sh                 # 默认网卡自动探测
#   ./net_diag_via_systemd.sh enx00e04c0b2110 192.168.1.178
#
set -uo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIAG="$DIR/net_diag_livox.sh"
OUT="${LIVOX_DIAG_OUT:-$PWD/livox_net_diag.txt}"
UNIT="livox-diag-$$"

[[ -x "$DIAG" ]] || { echo "找不到可执行脚本: $DIAG"; exit 1; }

echo "== 当前会话能力集 =="
grep -E "^Cap(Eff|Bnd)" /proc/self/status | sed 's/^/  /'
echo "== 对照：用 systemd-run --user 起独立实例 =="
systemctl --user show-environment >/dev/null 2>&1 || { echo "  systemd --user 不可用，退回直接运行"; exec "$DIAG" "$@"; }

# 用 sh -c 是为了把输出重定向到我们要的路径（systemd-run 默认收进 journal）
systemd-run --user --unit="$UNIT" --collect --wait --pipe \
  --setenv=HOME="$HOME" \
  /bin/sh -c "'$DIAG' $* > '$OUT' 2>&1; cat '$OUT'" || {
    echo "  systemd-run 失败，退回直接运行（可能仍带 netlink 报错）"
    exec "$DIAG" "$@"
  }

echo
echo "== 已保存到: $OUT =="
