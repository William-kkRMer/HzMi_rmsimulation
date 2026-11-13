#!/usr/bin/env bash
# 验证 /global_costmap/costmap 与 /plan_mask 的**取值分布**（决定 mask 桥的阈值对不对）
#
# 背景（2026-11-12 实测）：`[tdt] 动态掩码并入: 命中 0 格` 出现 15/15 次，
# `L0感知:DEGRADED(mask_hit=0` 出现 243 次 ⇒ **mask 一直是空的**，
# 也就是说 T-DT 的动态避障可能从来没生效过。必须确认是
#   ①nav2 发布的是代价原值 (253/254) 还是占用缩放值 (99/100)，还是
#   ②STVL 在这张图里根本没标记。

# ★ 2026-11-13：**自定位仓库根**（原来写死 /home/weicheng/...，换个目录就废）。
#   从脚本所在目录逐级上溯，找到 .git 即为仓库根。
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
while [ ! -d "$REPO/.git" ] && [ "$REPO" != "/" ]; do REPO="$(dirname "$REPO")"; done

set +u
ROOT="$REPO"; cd "$ROOT" || exit 1
OUT="$ROOT/.tmp_diag/cmprobe_$(date +%H%M%S)"; mkdir -p "$OUT/roslog" "$OUT/roshome"
export ROS_LOG_DIR="$OUT/roslog" ROS_HOME="$OUT/roshome"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1

setsid ./tools/scripts/run_sim.sh stvl nogui > "$OUT/sim.log" 2>&1 &
SIM=$!
echo "[probe] 仿真 pid=$SIM，输出 $OUT"

# 等 global_costmap 与 plan_mask 都出现
for i in $(seq 1 90); do
  if ros2 topic list 2>/dev/null | grep -qx "/plan_mask" &&
     ros2 topic list 2>/dev/null | grep -qx "/global_costmap/costmap"; then
    echo "[probe] 话题就绪（等待 $((i*2))s）"; break
  fi
  sleep 2
done
sleep 20   # 让 STVL 跑一会儿、costmap 发布几帧

python3 - "$OUT" <<'PY'
import sys, collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

out = sys.argv[1]
rclpy.init()
n = Node('cm_probe')
res = {}

def make_cb(name, key):
    def cb(msg):
        c = collections.Counter(int(v) & 0xFF for v in msg.data)
        res[key] = (msg.info.width, msg.info.height, msg.info.resolution,
                    dict(sorted(c.items())))
        print(f'[{name}] {msg.info.width}x{msg.info.height} @{msg.info.resolution:.3f} '
              f'origin=({msg.info.origin.position.x:.2f},{msg.info.origin.position.y:.2f})')
        print(f'   取值分布(前 12 种): {dict(list(sorted(c.items()))[:12])}')
        print(f'   关键计数: 0={c[0]} 1..99={sum(v for k,v in c.items() if 1<=k<=99)} '
              f'100={c[100]} 253={c[253]} 254={c[254]} 255={c[255]}')
    return cb

qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                 reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', make_cb('costmap', 'cm'), qos)
n.create_subscription(OccupancyGrid, '/plan_mask', make_cb('plan_mask', 'mask'), qos)
for _ in range(60):
    rclpy.spin_once(n, timeout_sec=0.5)
print(f'[probe] 收到 costmap={("cm" in res)} plan_mask={("mask" in res)}')
n.destroy_node(); rclpy.shutdown()
PY

kill -INT -$SIM 2>/dev/null; sleep 6; kill -9 -$SIM 2>/dev/null
echo "[probe] 完成：$OUT"
