#!/usr/bin/env bash
# USB 网卡链路「分岔测试」：判定问题在【网卡/线】还是【雷达端】
#
# 用法：把 USB 网卡插到**已知能用的网口**（路由器/交换机 LAN 口，或另一台电脑的网口），
#       然后运行本脚本；再把网卡插回雷达，再运行一次。
#
#   ./link_test.sh          # 读一次链路状态并判定
#   ./link_test.sh 5        # 连续监控 5 秒，能看到插拔瞬间的变化
#
set -uo pipefail

IFACE="${2:-}"
SECS="${1:-1}"
[[ "$SECS" =~ ^[0-9]+$ ]] || { echo "用法: $0 [监控秒数] [网卡名]"; exit 2; }

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
[[ -n "$IFACE" ]] || { echo "没找到有线网卡"; exit 1; }

c_ok()   { printf '\033[32m%s\033[0m\n' "$*"; }
c_warn() { printf '\033[33m%s\033[0m\n' "$*"; }
c_err()  { printf '\033[31m%s\033[0m\n' "$*"; }

read_link() {
  # 只用内核文件，不需要任何权限
  local carrier oper
  carrier="$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null)"
  oper="$(cat "/sys/class/net/$IFACE/operstate" 2>/dev/null)"
  echo "$carrier|$oper"
}

echo "网卡: $IFACE"
if [[ "$SECS" -gt 1 ]]; then
  echo "监控 ${SECS}s（插拔网线时可看到变化）…"
  for ((t=0; t<SECS; t++)); do
    IFS='|' read -r c o <<<"$(read_link)"
    printf '\r  t=%2ds  carrier=%-4s operstate=%-8s' "$t" "${c:-?}" "${o:-?}"
    sleep 1
  done
  echo
fi

IFS='|' read -r carrier oper <<<"$(read_link)"
echo
echo "== 结果 =="
echo "  carrier   = ${carrier:-未知}   (1=RJ45 侧有信号, 0=没有)"
echo "  operstate = ${oper:-未知}"
if command -v ethtool >/dev/null 2>&1; then
  echo "  ethtool   : $(ethtool "$IFACE" 2>/dev/null | grep -i 'link detected' | tr -d '\t')"
fi
echo
if [[ "$carrier" == "1" ]]; then
  c_ok "  ✓ 链路已建立 ⇒ 这块网卡 + 这根网线是好的"
  echo "     ⇒ 结论：问题在【雷达那一侧】：雷达本体网口 / 供电 / 网线另一端 / 中间设备"
  echo "     ⇒ 下一步：跑 ./find_livox.sh 扫出雷达真实 IP；再跑 ./start_livox_mid360.sh"
else
  c_err "  ✗ 仍然没有链路（carrier=0）"
  echo "     ⇒ 如果你现在插的是【已知能用的路由器/另一台电脑】："
  echo "         说明问题在【这块 USB 网卡或这根网线】⇒ 先换网线，再换网卡（RTL8153 第三方适配器有"
  echo "         '能枚举、固件正常、就是不起链路' 的已知毛病；可换 AX88179 / 原厂 8153 适配器）"
  echo "     ⇒ 如果你现在插的是【雷达】：还不能定论，请先做上面的'已知能用的网口'对照"
fi
