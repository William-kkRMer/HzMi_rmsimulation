#!/usr/bin/env bash
# 在雷达网段里找 Livox 雷达（不依赖配置里写死的 IP）
#
# 用途：链路 up 之后，若 MID360_config.json 里的雷达 IP（默认 192.168.1.178）与实际不符，
#       用本脚本扫出真实 IP/MAC，再回填配置。
#
# 用法:
#   ./find_livox.sh                 # 扫 192.168.1.0/24
#   ./find_livox.sh 192.168.2       # 扫别的 /24 前缀
#   ./find_livox.sh 192.168.1 192.168.1.50   # 指定前缀与本机源地址
#
set -uo pipefail

PREFIX="${1:-192.168.1}"
SRC_IP="${2:-${PREFIX}.50}"
IFACE="$(ip -o -4 addr show 2>/dev/null | awk -v ip="$SRC_IP" '$4 ~ "^"ip"/"{print $2; exit}')"

c_ok()   { printf '\033[32m%s\033[0m\n' "$*"; }
c_warn() { printf '\033[33m%s\033[0m\n' "$*"; }
c_err()  { printf '\033[31m%s\033[0m\n' "$*"; }

echo "网段前缀 : $PREFIX.0/24"
echo "本机源IP : $SRC_IP"
echo "网卡     : ${IFACE:-（未找到持有 $SRC_IP 的网卡）}"

if [[ -z "$IFACE" ]]; then
  c_err "本机没有 $SRC_IP，先在网卡上配好它再扫。"
  exit 1
fi

state="$(cat "/sys/class/net/$IFACE/operstate" 2>/dev/null)"
echo "链路状态 : $state"
if [[ "$state" != "up" ]]; then
  c_err "链路不是 up（物理层没连上）⇒ 扫不出任何东西。先解决网线/供电/对端网口。"
  echo "  排查：ethtool $IFACE | grep -i 'link detected'；换一根网线/换一个 USB 口再试"
  exit 1
fi
c_ok "链路 up，开始扫描"

# 用 ping 广播式地填 ARP 表（-b 只对本机网段有效；失败就退化为普通 ping 扫）
echo "正在扫描 $PREFIX.0/24 …"
for i in $(seq 1 254); do
  ping -c1 -W1 -I "$IFACE" "$PREFIX.$i" >/dev/null 2>&1 &
done
wait

echo
echo "== 有响应的主机（ARP 表）=="
ip neigh show dev "$IFACE" | grep -vE "FAILED|INCOMPLETE" | sed 's/^/  /' || echo "  （一个都没有）"

echo
echo "== 疑似的 Livox 设备（MAC 前缀 47:xx / 00:xx 里带 livox 常见 OUI，或 ARP 可达）=="
# Livox 设备 MAC 常见以 47: 开头（本地管理位）；这里两种都列出来供人工确认
ip neigh show dev "$IFACE" | grep -iE "^$PREFIX\.|47:" | sed 's/^/  /' || true

echo
echo "== 若上面列出了 IP，回填到配置 =="
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
echo "  $WS_DIR/src/driver/rm_driver/livox_ros_driver2/src/config/MID360_config.json"
echo "  改 lidar_configs[0].ip 后重跑 start_livox_mid360.sh"
