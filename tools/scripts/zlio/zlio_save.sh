#!/usr/bin/env bash
# 调 save_map 并打印结果 + 目录内容
set +u
source /opt/ros/humble/setup.bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
source install/setup.bash 2>/dev/null
source .tmp_cache/zlio_ws/install/setup.bash 2>/dev/null
: "${ROS_DOMAIN_ID:=42}"
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID RMW_IMPLEMENTATION
echo "=== 调用 save_map ==="
timeout 25 ros2 service call /backend_node/save_map std_srvs/srv/Trigger "{}" 2>&1 | tail -3
echo "=== 目录 ==="
ls -la .tmp_cache/prior/runtime_map 2>&1 | head -10
[ -f .tmp_cache/prior/runtime_map/keyframes.txt ] && echo "关键帧数: $(wc -l < .tmp_cache/prior/runtime_map/keyframes.txt)"
