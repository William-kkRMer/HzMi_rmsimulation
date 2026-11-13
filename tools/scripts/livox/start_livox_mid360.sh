#!/usr/bin/env bash
# 无 GUI 查看器时的替代方案：用 ROS2 驱动 + RViz 看 Livox MID-360 实时点云
#
# 原因：Livox 官方 LivoxViewer2（Linux 版 2.3.0 / 2.6.2）在本机 Ubuntu 22.04 桌面上
#       启动即 SIGSEGV（构造主界面控件时空指针解引用），两个版本/两个引擎都一样。
#       详见 docs/livox_viewer2_launch_notes.md
#
# 用法:
#   ./start_livox_mid360.sh              # 配网卡(如需) + 启动驱动 + 启动 RViz
#   ./start_livox_mid360.sh --check      # 只做连通性/话题自检，不开 RViz
#   ./start_livox_mid360.sh --no-rviz    # 只启动驱动（自己另开 RViz / rosbag）
#   ./start_livox_mid360.sh --iface enx7c8bca1b2d3   # 指定网卡
#   ./start_livox_mid360.sh --host-ip 192.168.1.50   # 覆盖本机在雷达网段的 IP
#
set -uo pipefail

# ---------- 与仓库配置对齐（改这里就要同步改 MID360_config.json）----------
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LIVOX_CFG="$WS_DIR/src/driver/rm_driver/livox_ros_driver2/src/config/MID360_config.json"
HOST_IP="192.168.1.50"        # MID360_config.json 的 host_net_info.*_ip
LIDAR_IP="192.168.1.178"      # MID360_config.json 的 lidar_configs[0].ip（实测值 2026-10-08：雷达 MAC 0c:9a:e6:03:fc:67）
NETMASK_PREFIX=24
ROS_DOMAIN_ID_DEFAULT=0

MODE="full"
IFACE=""
SKIP_NET=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --check)    MODE="check"; shift ;;
    --no-rviz)  MODE="driver"; shift ;;
    --skip-net) SKIP_NET=1; shift ;;   # 跳过改网卡（已配好，或只想验证 ROS 侧链路）
    --iface)    IFACE="${2:-}"; shift 2 ;;
    --host-ip)  HOST_IP="${2:-}"; shift 2 ;;
    -h|--help)  sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "未知参数: $1（用 --help 看用法）"; exit 2 ;;
  esac
done

c_ok()   { printf '\033[32m%s\033[0m\n' "$*"; }
c_warn() { printf '\033[33m%s\033[0m\n' "$*"; }
c_err()  { printf '\033[31m%s\033[0m\n' "$*"; }
c_head() { printf '\n\033[1;36m== %s ==\033[0m\n' "$*"; }

# ---------- 0. 配置一致性自检 ----------
c_head "0. 读取仓库里的 MID360 配置"
if [[ ! -f "$LIVOX_CFG" ]]; then
  c_err "找不到配置文件: $LIVOX_CFG"; exit 1
fi
cfg_host_ip="$(grep -oP '"cmd_data_ip"\s*:\s*"\K[^"]+' "$LIVOX_CFG" | head -1)"
cfg_lidar_ip="$(grep -oP '"ip"\s*:\s*"\K[^"]+' "$LIVOX_CFG" | head -1)"
echo "  配置文件      : $LIVOX_CFG"
echo "  本机(host) IP : $cfg_host_ip   (脚本用: $HOST_IP)"
echo "  雷达(lidar) IP: $cfg_lidar_ip  (脚本用: $LIDAR_IP)"
if [[ -n "$cfg_host_ip" && "$cfg_host_ip" != "$HOST_IP" ]]; then
  c_warn "  ⚠ 脚本的 --host-ip 与配置文件不一致：网卡必须配成配置里那个 IP，否则雷达回包发不到本机"
  HOST_IP="$cfg_host_ip"
fi
if [[ -n "$cfg_lidar_ip" ]]; then LIDAR_IP="$cfg_lidar_ip"; fi

# ---------- 1. 找到有线网卡 ----------
c_head "1. 查找有线网卡"
if [[ -z "$IFACE" ]]; then
  # 物理有线网卡：有 /sys/class/net/<if>/device 且驱动不是 wifi
  for n in /sys/class/net/*; do
    i="$(basename "$n")"
    [[ "$i" == "lo" || "$i" == docker* || "$i" == br-* || "$i" == veth* || "$i" == tun* ]] && continue
    [[ -e "$n/device" ]] || continue
    drv="$(basename "$(readlink -f "$n/device/driver" 2>/dev/null)" 2>/dev/null)"
    [[ "$drv" == "iwlwifi" || "$drv" == "ath"* || "$drv" == "rtw"* ]] && continue
    IFACE="$i"; break
  done
fi

if [[ -z "$IFACE" ]]; then
  c_err "没有找到有线网卡。"
  echo "  MID-360 是网口雷达，必须有一块有线网卡（USB 转网口/扩展坞都行）。"
  echo "  插上后重新运行本脚本；确认是否识别： ip -br link"
  exit 1
fi
c_ok "  使用网卡: $IFACE"

# ---------- 2. 网卡 IP 配置 ----------
c_head "2. 网卡地址"
cur_ip="$(ip -4 -o addr show dev "$IFACE" 2>/dev/null | awk '{print $4}' | head -1)"
echo "  当前: ${cur_ip:-（未配置 IPv4）}"
link_up="$(cat "/sys/class/net/$IFACE/operstate" 2>/dev/null || echo unknown)"
echo "  链路: $link_up"
if [[ "$link_up" != "up" ]]; then
  c_warn "  ⚠ 网卡链路不是 up —— 网线没插、或雷达没上电。先插好线再继续。"
fi

if [[ "$SKIP_NET" == "1" ]]; then
  c_warn "  --skip-net：跳过网卡地址配置"
  c_ok "  本机在雷达网段: ${cur_ip:-（未配置）} ← 按 --skip-net 不做检查"
elif [[ "$cur_ip" != "$HOST_IP/$NETMASK_PREFIX" ]]; then
  c_warn "  需要把 $IFACE 配成 $HOST_IP/$NETMASK_PREFIX（会调用 sudo 改 NetworkManager 连接）"
  echo "  执行: nmcli con add/modify ... ipv4.addresses $HOST_IP/$NETMASK_PREFIX"
  if sudo -n true 2>/dev/null; then SUDO="sudo"; else SUDO="sudo"; fi   # 需要交互输密码
  con="$(nmcli -t -f NAME,DEVICE con show 2>/dev/null | awk -F: -v d="$IFACE" '$2==d{print $1; exit}')"
  if [[ -n "$con" ]]; then
    $SUDO nmcli con mod "$con" ipv4.method manual ipv4.addresses "$HOST_IP/$NETMASK_PREFIX" ipv4.never-default yes && \
    $SUDO nmcli con up "$con" || { c_err "  配置失败"; exit 1; }
  else
    $SUDO nmcli con add type ethernet ifname "$IFACE" con-name "livox-$IFACE" \
      ipv4.method manual ipv4.addresses "$HOST_IP/$NETMASK_PREFIX" ipv4.never-default yes && \
    $SUDO nmcli con up "livox-$IFACE" || { c_err "  配置失败"; exit 1; }
  fi
  sleep 1
  cur_ip="$(ip -4 -o addr show dev "$IFACE" | awk '{print $4}' | head -1)"
  echo "  现在: ${cur_ip:-（仍然没有 IP，请手动排查）}"
  c_ok "  本机在雷达网段: $cur_ip"
fi

# ---------- 3. 连通性 ----------
c_head "3. 与雷达的连通性 ($LIDAR_IP)"
if [[ "$SKIP_NET" == "1" ]]; then
  c_warn "  --skip-net：跳过"
else
  if ping -c 2 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then
    c_ok "  ping 通"
  else
    c_warn "  ping 不通（MID-360 有时不回 ICMP，下面继续看数据面）"
  fi
  if command -v arping >/dev/null 2>&1; then
    if arping -c 2 -I "$IFACE" "$LIDAR_IP" >/dev/null 2>&1; then
      c_ok "  ARP 能解析到雷达（说明雷达在线且在同一网段）"
    else
      c_warn "  ARP 未解析到 $LIDAR_IP（检查雷达上电/网线/网段/雷达自身 IP）"
    fi
  else
    c_warn "  未安装 arping，跳过 ARP 探测（可 apt install iputils-arping）"
  fi
  echo "  ARP 表:"; ip neigh show dev "$IFACE" | sed 's/^/    /' || true
fi

# ---------- 3.5 雷达 IP 是否与配置一致（实测最容易踩的坑）----------
c_head "3.5 配置里的雷达 IP ($LIDAR_IP) 是否真的在线"
if [[ "$SKIP_NET" != "1" ]]; then
  neigh="$(ip neigh show dev "$IFACE" 2>/dev/null | awk -v ip="$LIDAR_IP" '$1==ip{print $0}')"
  if grep -qE "REACHABLE|STALE|DELAY|PROBE" <<<"$neigh"; then
    c_ok "  在线（ARP: $(awk '{print $1, $NF}' <<<"$neigh" | head -1)）"
  else
    c_warn "  ⚠ 配置里的 $LIDAR_IP 不在 ARP 表里 —— 雷达很可能换了 IP！"
    echo "    已知症状：驱动能起、日志狂刷 'Can not get index / Storage point data failed'（数据被整包丢弃），"
    echo "    RViz 报 'Frame [livox_frame] does not exist'（因为一帧都没发布）。"
    echo "    正在扫 $IFACE 上同网段的设备（可能有另一台 Livox 雷达/别的设备）…"
    pfx="${LIDAR_IP%.*}"
    for i in $(seq 1 254); do ping -c1 -W1 -I "$IFACE" "$pfx.$i" >/dev/null 2>&1 & done
    wait
    found="$(ip neigh show dev "$IFACE" 2>/dev/null | grep -vE "FAILED|INCOMPLETE" | grep -v " $LIDAR_IP " || true)"
    if [[ -n "$found" ]]; then
      echo "    扫到这些在线设备（Livox 是 DJI 旗下，MAC 前缀常见 0c:9a:e6 / 47:xx）："
      sed 's/^/      /' <<<"$found"
      c_warn "    ⇒ 若其中某个是雷达，把配置里的 lidar_configs[0].ip 改成它，再重跑本脚本。"
    else
      c_warn "    ⇒ 没扫到任何设备。检查雷达供电/网线（见 net_diag_livox.sh）"
    fi
  fi
fi

# ---------- 4. ROS 环境 ----------
c_head "4. ROS 环境"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u; source /opt/ros/humble/setup.bash; set -u
  c_ok "  已 source /opt/ros/humble/setup.bash"
else
  c_err "  找不到 /opt/ros/humble/setup.bash"; exit 1
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  set +u; source "$WS_DIR/install/setup.bash"; set -u
  c_ok "  已 source $WS_DIR/install/setup.bash"
else
  c_warn "  ⚠ 工作区未构建（缺 install/setup.bash），livox_ros_driver2 可能找不到"
fi
if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  c_err "  找不到 livox_ros_driver2 包，先构建：cd $WS_DIR && colcon build --packages-select livox_ros_driver2"
  exit 1
fi

# 代理会干扰 DDS 的本地发现，这里清掉
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY 2>/dev/null || true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$ROS_DOMAIN_ID_DEFAULT}"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# ---------- 5. 启动驱动 ----------
c_head "5. 启动 livox_ros_driver2（PointCloud2 模式）"
# 硬前置：Livox SDK 会把本机 socket 绑到配置里的 host IP，该 IP 不在本机就必然 "bind failed"
if ! ip -4 -o addr show 2>/dev/null | grep -qw "$HOST_IP"; then
  c_err "  本机没有 $HOST_IP 这个地址，Livox SDK 会 bind failed（实测报错：'bind failed / Init lds lidar fail'）"
  echo "  先把网卡配成它（配置文件的 host_net_info.*_ip 就是 $HOST_IP）："
  echo "    sudo nmcli con mod \"<连接名>\" ipv4.method manual ipv4.addresses $HOST_IP/$NETMASK_PREFIX ipv4.never-default yes"
  echo "    sudo nmcli con up \"<连接名>\""
  echo "  或确认雷达/网段不同时，改 $LIVOX_CFG 后加 --host-ip 重跑。"
  exit 1
fi
LOG_DIR="${LIVOX_LOG_DIR:-$WS_DIR/.tmp_roslog_driver}"
mkdir -p "$LOG_DIR" 2>/dev/null || LOG_DIR="$HOME"
LOG="$LOG_DIR/livox_driver_$(date +%Y%m%d_%H%M%S).log"
ros2 launch livox_ros_driver2 rviz_MID360_launch.py >"$LOG" 2>&1 &
DRV_PID=$!
sleep 4
if ! kill -0 "$DRV_PID" 2>/dev/null; then
  c_err "  驱动启动失败，日志尾部："; tail -25 "$LOG" | sed 's/^/    /'; exit 1
fi
if grep -q "bind failed\|Init lds lidar fail" "$LOG" 2>/dev/null; then
  c_err "  驱动起来了但 SDK 初始化失败（日志见下）："; grep -E "bind failed|Failed to init|Init lds lidar fail" "$LOG" | sed 's/^/    /'
  kill "$DRV_PID" 2>/dev/null
  exit 1
fi
c_ok "  驱动已启动 (pid $DRV_PID)，日志: $LOG"

# ---------- 6. 数据面验证 ----------
c_head "6. 话题自检"
sleep 2
echo "  话题列表（livox 相关）:"; ros2 topic list 2>/dev/null | grep -i livox | sed 's/^/    /' || true
hz="$(timeout 8 ros2 topic hz /livox/lidar 2>/dev/null | grep -m1 -oE 'average rate: [0-9.]+' || true)"
if [[ -n "$hz" ]]; then
  c_ok "  /livox/lidar $hz  ← 雷达有数据"
else
  c_warn "  /livox/lidar 没测到频率：雷达没数据/还没连上（检查第 3 步）"
fi
imu="$(timeout 8 ros2 topic hz /livox/imu 2>/dev/null | grep -m1 -oE 'average rate: [0-9.]+' || true)"
[[ -n "$imu" ]] && c_ok "  /livox/imu $imu" || c_warn "  /livox/imu 没测到频率"

if [[ "$MODE" == "check" ]]; then
  c_head "自检模式结束"
  kill "$DRV_PID" 2>/dev/null
  wait "$DRV_PID" 2>/dev/null
  exit 0
fi

# ---------- 7. RViz ----------
if [[ "$MODE" == "full" ]]; then
  c_head "7. 启动 RViz（Fixed Frame=livox_frame, Topic=/livox/lidar）"
  RVCFG="$(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2/config/display_point_cloud.rviz"
  [[ -f "$RVCFG" ]] || RVCFG="$WS_DIR/src/driver/rm_driver/livox_ros_driver2/src/config/display_point_cloud.rviz"
  if [[ -f "$RVCFG" ]]; then
    rviz2 -d "$RVCFG" >/dev/null 2>&1 &
    c_ok "  RViz 已启动 (pid $!)，配置: $RVCFG"
  else
    rviz2 >/dev/null 2>&1 &
    c_warn "  没找到 rviz 配置，用默认配置启动（手动把 Fixed Frame 设成 livox_frame、加 PointCloud2 → /livox/lidar）"
  fi
fi

c_head "就绪"
echo "  驱动 pid=$DRV_PID，日志: $LOG"
echo "  按 Ctrl+C 结束（会一并停掉驱动）"
trap 'echo; echo "停止驱动…"; kill "$DRV_PID" 2>/dev/null' INT TERM
wait "$DRV_PID"
