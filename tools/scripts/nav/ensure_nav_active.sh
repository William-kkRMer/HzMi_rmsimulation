#!/usr/bin/env bash
# 检查 nav2 各 lifecycle 节点是否 active；未激活的补一次 configure/activate。
#
# 背景（2026-11-11 实测）：启动瞬间 CPU 被拉满时，
#   lifecycle_manager 调 /smoother_server/change_state 会超时
#   （`failed to send response ... (timeout)`），导致 bringup 中断，
#   后面的 planner_server 压根没被 configure ⇒
#   L0/L1 全程无数据、RViz 里点目标毫无反应。
#   这不是配置错误，是启动竞态 ⇒ 用本脚本兜底重试。
#
# 用法（导航起来后，任一间终端）：
#   ./tools/scripts/nav/ensure_nav_active.sh

set -u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

# 依赖顺序：后面的节点 require 前面的
NODES=(
  map_server
  controller_server
  smoother_server
  planner_server
  behavior_server
  bt_navigator
  waypoint_follower
  velocity_smoother
)

state_of() {
  timeout 5 ros2 lifecycle get "/$1" 2>/dev/null | awk '{print $1}'
}

fixed=0
for n in "${NODES[@]}"; do
  st="$(state_of "$n")"
  printf '%-22s %s' "$n" "${st:-<不存在>}"
  case "$st" in
    active)
      echo "   ok" ;;
    unconfigured)
      echo "   → configure"
      timeout 20 ros2 lifecycle set "/$n" configure >/dev/null 2>&1
      timeout 20 ros2 lifecycle set "/$n" activate  >/dev/null 2>&1
      echo "       现在: $(state_of "$n")"; fixed=$((fixed+1)) ;;
    inactive)
      echo "   → activate"
      timeout 20 ros2 lifecycle set "/$n" activate >/dev/null 2>&1
      echo "       现在: $(state_of "$n")"; fixed=$((fixed+1)) ;;
    *)
      echo "   ?? 需要人工看" ;;
  esac
done

echo
if [ "$fixed" -eq 0 ]; then
  echo "全部已 active。若 RViz 仍无轨迹，问题在话题/QoS，不是生命周期。"
else
  echo "补齐了 $fixed 个节点。现在去 RViz 点目标应该就有轨迹了。"
fi
