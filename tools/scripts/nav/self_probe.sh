#!/usr/bin/env bash
# 无人值守自测：起仿真 → 等导航激活 → 发固定目标 → 采集指标 → 退出
#
# 目的：不再让用户手工跑，也不需要"猜配置"。一次跑完直接产出可判读的指标：
#   · /tdt-local cmd 的 s= 序列（有没有沿路径推进）
#   · 指令抖动（相邻指令的欧氏差）
#   · 底盘实测速度（均值/峰值）
#   · T-DT 恢复 FSM 是否进入 ESCAPE
#   · 规划统计（点数/简化/snap 失败）
#
# 用法：tools/scripts/nav/self_probe.sh [目标x] [目标y] [驾驶秒数(仿真时间)]

# ★ 2026-11-13：**自定位仓库根**（原来写死 /home/weicheng/...，换个目录就废）。
#   从脚本所在目录逐级上溯，找到 .git 即为仓库根。
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
while [ ! -d "$REPO/.git" ] && [ "$REPO" != "/" ]; do REPO="$(dirname "$REPO")"; done

set +u   # ROS 的 setup.bash 引用未定义变量，不能用 set -u
ROOT="$REPO"; cd "$ROOT" || exit 1

GX="${1:-4.0}"; GY="${2:--2.0}"; BUDGET="${3:-20}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RCUTILS_LOGGING_BUFFERED_STREAM=1
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
set -u

OUT="$ROOT/.tmp_diag/selfprobe_$(date +%H%M%S)"
mkdir -p "$OUT/roslog" "$OUT/roshome"
# ★ 本机 ~/.ros 是只读的 ⇒ rclpy/ROS 节点默认写 $HOME/.ros/log 会直接抛
#   RuntimeError: Failed opening file ... Read-only file system
#   （实测：ab_goal.py 在 rclpy.init() 就崩）。必须指到可写目录。
export ROS_LOG_DIR="$OUT/roslog"
export ROS_HOME="$OUT/roshome"
echo "[probe] 输出目录: $OUT"
echo "[probe] 目标 ($GX, $GY)，预算 ${BUDGET}s 仿真时间"

# 1) 起仿真（无界面，RTF 更好）
setsid ./tools/scripts/run_sim.sh stvl nogui > "$OUT/sim.log" 2>&1 &
SIM_PID=$!
echo "[probe] 仿真启动 pid=$SIM_PID"

# 2) 等导航就绪（控制器激活 + navigate_to_pose action 可用），最多 260 s
ready=0
for i in $(seq 1 130); do
  if grep -q "controller_server\]: Activating" "$OUT/sim.log" 2>/dev/null &&
     ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; then
    ready=1; echo "[probe] 导航就绪（等待 $((i*2))s）"; break
  fi
  sleep 2
done
if [ "$ready" != "1" ]; then
  echo "[probe] ✗ 等待超时，导航未就绪；保留日志 $OUT" >&2
  kill -INT -$SIM_PID 2>/dev/null; sleep 5; kill -9 -$SIM_PID 2>/dev/null
  exit 1
fi

# 3) 发目标并测量
python3 tools/scripts/nav/ab_goal.py --gx "$GX" --gy "$GY" --budget "$BUDGET" \
  --out "$OUT/goal.json" 2>&1 | tail -20 | tee "$OUT/goal.txt"

# 4) 收尾：停机器人 → 关仿真
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
sleep 3
kill -INT -$SIM_PID 2>/dev/null
sleep 8
kill -9 -$SIM_PID 2>/dev/null
pkill -9 -f gzserver 2>/dev/null

# 5) 汇总（从 run_sim 的 launch.log 取，不是 sim.log：后者只有 run_sim 自己几行）
RUN=$(ls -1dt "$ROOT"/.tmp_diag/run/*/ 2>/dev/null | head -1)
echo "[probe] 运行日志: $RUN"
L="$RUN/launch.log"
if [ -f "$L" ]; then
  echo "[probe] ---- s= 序列（沿路径里程，应单调增） ----"
  grep -oE "s=[0-9.]+" "$L" | tail -40 | tr '\n' ' '; echo
  echo "[probe] ---- 指令条数 / 底盘速度 ----"
  echo "  cmd 条数(限频后): $(grep -c '\[tdt-local\] cmd:' "$L")"
  grep -oE "底盘实测 [0-9.]+ m/s" "$L" | awk '{s+=$2;n++; if($2>m)m=$2}
      END{if(n)printf "  底盘实测 平均 %.2f 峰值 %.2f (n=%d)\n",s/n,m,n; else print "  无底盘数据"}'
  echo "  ESCAPE 次数: $(grep -c '\[tdt\]\[恢复\] ESCAPE 中' "$L")"
  echo "  snap 失败:   $(grep -c 'snap=0' "$L")"
  echo "  kastar 失败: $(grep -c 'kastar \*\*失败' "$L")"
  grep -oE "平均 [0-9.]+ m/s" "$L" | awk '{s+=$2;n++} END{if(n)printf "  K-star 平均 %.2f m/s (n=%d)\n",s/n,n}'
fi
echo "[probe] 完成：$OUT"
