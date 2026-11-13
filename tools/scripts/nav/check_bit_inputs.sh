#!/usr/bin/env bash
# check_bit_inputs.sh —— 跑 `MODE=bit` 时，一条命令确认"数据齐不齐"。
#
# 为什么要有它：BIT 的 minco_planner 会在进程内自建 ROGMap，它要两路数据
#   （世界系点云 + 里程计），**任何一路断掉都表现为"规划不动"**，
#   而从 launch.log 里看不出是"没数据"还是"参数错"。
#
# 用法（仿真跑起来、定位到 NORMAL 之后执行）：
#     ./tools/scripts/nav/check_bit_inputs.sh
#     ./tools/scripts/nav/check_bit_inputs.sh 8        # 每路测 8 秒
#
# 判据（脚本末尾会汇总）：
#   /localization/odometry          必须 >0 Hz
#   /localization/lio_world_cloud   必须 >0 Hz     ← ROGMap 的点云输入
#   /aft_mapped_to_init             必须 >0 Hz     ← BIT 控制器订阅的里程计（我们中继过去的）
#   /cmd_vel_mpc                    车动时 >0 Hz   ← BIT 控制器输出
#   /plan                           有规划时 >0    ← nav2 给规划器的结果
set -u

DUR="${1:-5}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"   # nav → scripts → tools → 仓库根
cd "$REPO"
set +u   # ROS 的 setup.bash 会引用未定义变量，`set -u` 下会直接退出
source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
source install/setup.bash >/dev/null 2>&1 || true
[ -f .tmp_cache/zlio_ws/install/setup.bash ] && source .tmp_cache/zlio_ws/install/setup.bash >/dev/null 2>&1 || true
set -u

TOPICS=(
  "/localization/odometry|里程计（ROGMap + 控制器都要）"
  "/localization/lio_world_cloud|世界系点云（ROGMap 输入）★"
  "/aft_mapped_to_init|BIT 控制器订阅的里程计（中继产物）"
  "/cmd_vel_mpc|BIT 控制器输出（车动时才有）"
  "/plan|nav2 规划结果"
)

printf "  %-34s %-12s %s\n" 话题 速率 说明
printf "  %s\n" "----------------------------------------------------------------------"
fail=0
for entry in "${TOPICS[@]}"; do
  topic="${entry%%|*}"
  desc="${entry##*|}"
  out=$(timeout $((DUR + 4)) ros2 topic hz "$topic" 2>&1 | grep -oE "average rate: [0-9.]+" | tail -1 | grep -oE "[0-9.]+")
  if [ -n "${out:-}" ] && awk "BEGIN{exit !($out > 0)}" 2>/dev/null; then
    printf "  %-34s %-12s %s\n" "$topic" "${out} Hz" "$desc"
  else
    printf "  %-34s %-12s %s\n" "$topic" "—" "$desc  ✗ 没数据"
    case "$topic" in
      /localization/odometry|/localization/lio_world_cloud) fail=1 ;;
    esac
  fi
done
printf "  %s\n" "----------------------------------------------------------------------"
if [ "$fail" = "0" ]; then
  echo "  ✓ 两路关键数据都在（点云 + 里程计）⇒ BIT 的数据前提满足"
else
  echo "  ✗ 关键数据缺失（上面带 ✗ 的）——先解决它，再看规划"
  echo "    提示：world_cloud 只在定位 NORMAL 后才发（配置 publish_untrusted=false）"
fi
exit 0
