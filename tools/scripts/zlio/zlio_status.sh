#!/usr/bin/env bash
# 一次性体检：网络/话题/地图是否有内容 —— 全部带超时，绝不阻塞
set +u
source /opt/ros/humble/setup.bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
source install/setup.bash 2>/dev/null
source .tmp_cache/zlio_ws/install/setup.bash 2>/dev/null
: "${ROS_DOMAIN_ID:=42}"
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID RMW_IMPLEMENTATION

ok()   { printf '  \033[32m✅ %s\033[0m\n' "$1"; }
bad()  { printf '  \033[31m❌ %s\033[0m\n' "$1"; }
warn() { printf '  \033[33m⚠️  %s\033[0m\n' "$1"; }

echo "=== 环境 ==="
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"

echo "=== 节点 ==="
for n in /backend_node /zlio; do
  if ros2 node list 2>/dev/null | grep -qx "$n"; then ok "节点 $n"; else bad "节点 $n 不存在"; fi
done

echo "=== 话题 ==="
for t in /livox/lidar /livox/imu /frontend/odometry /backend_node/runtime_map; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then ok "话题 $t"; else bad "话题 $t 不存在"; fi
done

echo "=== 频率（各 5 s）==="
hz() { local t=$1; local v; v=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | awk '/average rate/{print $3; exit}'); \
       if [ -n "$v" ]; then ok "$t = $v Hz"; else warn "$t 5 秒内无数据"; fi; }
hz /livox/lidar
hz /frontend/odometry
hz /backend_node/runtime_map

echo "=== 地图点数 ==="
W=$(timeout 6 ros2 topic echo /backend_node/runtime_map --field width --once 2>/dev/null | head -1)
if [ -n "$W" ]; then ok "runtime_map width = $W"; else bad "runtime_map 无消息（地图还是空的）"; fi

echo "=== 结论 ==="
if [ -n "$W" ]; then
  ok "地图有内容 ⇒ 可以保存：tools/scripts/zlio/zlio_save.sh"
else
  bad "地图为空 ⇒ 先开车跑 1~2 分钟再看；若一直为空，看后端日志的 invalid/covariance 计数"
fi
