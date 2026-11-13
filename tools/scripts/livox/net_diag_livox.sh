#!/usr/bin/env bash
# Livox 雷达网口链路 一键取证：一条命令打印第 1~3 层全部读数
#
# 用途：不用来回问"网卡通不通"，直接跑这个，把输出整段贴出来即可定位。
#
# 用法:
#   ./net_diag_livox.sh                       # 自动找 USB 有线网卡
#   ./net_diag_livox.sh enx00e04c0b2110       # 指定网卡
#   ./net_diag_livox.sh enx00e04c0b2110 192.168.1.178   # 指定网卡 + 雷达 IP
#
set -uo pipefail

IFACE="${1:-}"
LIDAR_IP="${2:-192.168.1.178}"

hdr() { printf '\n\033[1;36m===== %s =====\033[0m\n' "$*"; }
kv()  { printf '  %-34s %s\n' "$1" "$2"; }

# ---------- 找网卡 ----------
if [[ -z "$IFACE" ]]; then
  for n in /sys/class/net/*; do
    i="$(basename "$n")"
    [[ "$i" == lo || "$i" == docker* || "$i" == br-* || "$i" == veth* || "$i" == tun* ]] && continue
    [[ -e "$n/device" ]] || continue
    drv="$(basename "$(readlink -f "$n/device/driver" 2>/dev/null)" 2>/dev/null)"
    [[ "$drv" == iwlwifi || "$drv" == ath* || "$drv" == rtw* ]] && continue
    IFACE="$i"; break
  done
fi
if [[ -z "$IFACE" ]]; then echo "没有找到有线网卡（USB 网卡插了吗？）"; exit 1; fi

hdr "0. 基本信息"
kv "时间" "$(date '+%F %T')"
kv "网卡" "$IFACE"
kv "驱动" "$(basename "$(readlink -f /sys/class/net/$IFACE/device/driver 2>/dev/null)" 2>/dev/null)"
kv "雷达 IP（待查）" "$LIDAR_IP"

# ---------- 第 1 层：物理链路 ----------
hdr "1. 第 1 层：物理链路（这一层不通，后面全都不用看）"
kv "operstate" "$(cat /sys/class/net/$IFACE/operstate 2>/dev/null)   ← 内核文件，最可信"
kv "carrier" "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null)   ← 内核文件，最可信（1=有信号）"
kv "speed / duplex（内核）" "$(cat /sys/class/net/$IFACE/speed 2>/dev/null) Mbps / $(cat /sys/class/net/$IFACE/duplex 2>/dev/null)"
if command -v ethtool >/dev/null 2>&1; then
  et="$(ethtool "$IFACE" 2>&1)"
  if grep -qi "netlink error" <<<"$et"; then
    echo "  ⚠ ethtool 报 'netlink error: Operation not permitted' —— 这是**受限会话的权限问题**，"
    echo "    与雷达无关；它的 'Link detected' 仍有效，速度/双工请以上面内核文件为准。"
    echo "    （想要完全干净的 ethtool 输出：从桌面新开一个终端跑，或 sudo ethtool …）"
  fi
  grep -iE "Speed:|Duplex:|Link detected|Auto-negotiation|Port:" <<<"$et" | sed 's/^\s*/  /'
  echo "  ── 判据：'Link detected: no' ⇒ 网卡在 RJ45 侧收不到对端信号（网线/雷达端/USB 口/网卡）"
else
  echo "  （未安装 ethtool：sudo apt install ethtool）"
fi
kv "收发计数(rx/tx bytes)" "$(cat /sys/class/net/$IFACE/statistics/rx_bytes 2>/dev/null) / $(cat /sys/class/net/$IFACE/statistics/tx_bytes 2>/dev/null)"
kv "rx_errors / tx_errors" "$(cat /sys/class/net/$IFACE/statistics/rx_errors 2>/dev/null) / $(cat /sys/class/net/$IFACE/statistics/tx_errors 2>/dev/null)"

# ---------- 第 2 层：地址与邻居 ----------
hdr "2. 第 2/3 层：本机地址、路由、ARP"
kv "本机 IPv4" "$(ip -4 -o addr show dev "$IFACE" 2>/dev/null | awk '{print $4}' | paste -sd, -)"
ip route show | grep -E "$IFACE" | sed 's/^/  路由: /'
echo "  邻居表:"; ip neigh show dev "$IFACE" 2>/dev/null | sed 's/^/    /' || echo "    （空）"

# ---------- 连通性 ----------
hdr "3. 连通性探测（$LIDAR_IP）"
if [[ "$(cat /sys/class/net/$IFACE/operstate 2>/dev/null)" == "up" ]]; then
  if ping -c 2 -W 1 -I "$IFACE" "$LIDAR_IP" >/dev/null 2>&1; then
    echo "  ping $LIDAR_IP ：通"
  else
    echo "  ping $LIDAR_IP ：不通（MID-360 可能不回 ICMP，看 ARP 更准）"
  fi
  echo "  扫一遍 $LIDAR_IP 所在 /24 找邻居（填 ARP 表）…"
  pfx="${LIDAR_IP%.*}"
  for i in $(seq 1 254); do ping -c1 -W1 -I "$IFACE" "$pfx.$i" >/dev/null 2>&1 & done
  wait
  echo "  扫描后有响应的主机："
  ip neigh show dev "$IFACE" 2>/dev/null | grep -vE "FAILED|INCOMPLETE" | sed 's/^/    /' || echo "    （一个都没有 ⇒ 雷达不在这个网段/没上线）"
else
  echo "  链路 down ⇒ 跳过（先解决第 1 层）"
fi

# ---------- USB 层 ----------
hdr "4. USB 层（网卡是不是稳定挂着）"
if command -v lsusb >/dev/null 2>&1; then
  lsusb | grep -iE "realtek|asix|ethernet|8153|8152" | sed 's/^/  /' || echo "  （没有 USB 网卡）"
  echo "  总线上位置/速率:"; lsusb -t 2>/dev/null | grep -iE "r8152|cdc_ether|ax88|asix" | sed 's/^/    /'
fi
echo "  内核最近的 USB/网卡消息（需要 sudo dmesg 才全）:"
sudo -n dmesg 2>/dev/null | grep -iE "r8152|usb.*eth|over-current|reset" | tail -8 | sed 's/^/    /' || echo "    （无权限读取，可手动 sudo dmesg | tail -30）"

# ---------- 结论 ----------
hdr "5. 结论速判"
link="$(ethtool "$IFACE" 2>/dev/null | grep -i 'link detected' | awk -F: '{print $2}' | tr -d ' ')"
if [[ "$link" == "no" ]]; then
  echo "  ✗ 物理层没通（Link detected: no）⇒ 按顺序试："
  echo "     1) 换一个 USB 口重插网卡（r8152 的 PHY 会卡死）"
  echo "     2) 把网卡插到已知能用的路由器/交换机口，看 link 是否变 yes（切开网卡与雷达两端）"
  echo "     3) 换一根网线；检查雷达那端水晶头/航空插头是否到底"
  echo "     4) 确认雷达在运行态（9–27V 独立供电，网口不支持 PoE；绿灯常亮）"
else
  echo "  ✓ 物理层已通（Link detected: yes）"
  if ip neigh show dev "$IFACE" 2>/dev/null | grep -qE "REACHABLE|STALE|DELAY"; then
    echo "  ✓ 邻居表里有设备 —— 对比 $LIDAR_IP 是否需要改配置（或跑 find_livox.sh）"
  else
    echo "  ✗ 邻居表里没有对端 ⇒ 雷达 IP 不在本网段，或雷达未起（跑 find_livox.sh 扫）"
  fi
fi
echo
echo "  把上面整段输出贴回来即可。"
