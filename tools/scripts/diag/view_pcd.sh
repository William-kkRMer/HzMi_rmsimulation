#!/usr/bin/env bash
# =============================================================================
# view_pcd.sh —— 一条命令把任意 PCD 丢进 RViz 看（学习 T-DT 那套：PCD 当资产 + RViz 看）
#
# 原理：`pcl_ros/pcd_to_pointcloud` 读 PCD → 以 PointCloud2 周期发布到 /cloud_pcd（pcl_ros 的固定话题名）。
#       RViz 里 Fixed Frame 设成 frame_id（默认 map），PointCloud2 选 /cloud_pcd。
#
# 用法：
#   tools/scripts/diag/view_pcd.sh export/lio_sam_pcd/CornerMap.pcd
#   tools/scripts/diag/view_pcd.sh a.pcd b.pcd          # 多个文件 ⇒ 用 voxel 合并后再看
#   FRAME=odom tools/scripts/diag/view_pcd.sh x.pcd     # 换坐标系
#   RV=0 tools/scripts/diag/view_pcd.sh x.pcd           # 只发话题，自己开 RViz
#
# 其它更合适的工具（本机都装了）：
#   · CloudCompare   —— 离线看质量最强（能算 Cloud-to-Cloud 距离、切片、量厚度）
#   · pcl_viewer     —— 最快看一眼（pcl_viewer x.pcd）
#   · pcd_preview.py —— 直接出 PNG（无显示器/写文档用）
# =============================================================================
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.." || exit 1
source install/setup.bash 2>/dev/null || true

FRAME=${FRAME:-map}
RATE=${RATE:-1.0}
RV=${RV:-1}
[ "$#" -ge 1 ] || { echo "用法: $0 <a.pcd> [b.pcd ...]" >&2; exit 2; }

PCD=$1
if [ "$#" -gt 1 ]; then
  PCD=.tmp_cache/view_pcd_merged.pcd
  mkdir -p .tmp_cache
  echo "多个输入 ⇒ 先体素合并到 $PCD"
  python3 tools/scripts/diag/pcd_merge.py "$@" --voxel 0.03 --out "$PCD" || exit 1
fi
[ -f "$PCD" ] || { echo "✗ 没有文件: $PCD" >&2; exit 2; }

echo "发布 $PCD → /cloud_pcd（frame=$FRAME, 周期 ${RATE}s）"
ros2 run pcl_ros pcd_to_pointcloud --ros-args \
    -p file_name:="$PWD/$PCD" -p tf_frame:="$FRAME" -p publishing_period_ms:=$(python3 -c "print(int(float('$RATE')*1000))") &
PUB=$!
sleep 2
if [ "$RV" = "1" ]; then
  RVIZ_CFG=$PWD/tools/scripts/diag/pcd_view.rviz
  echo "开 RViz（Fixed Frame=$FRAME）；关掉本终端即结束"
  ros2 run rviz2 rviz2 -d "$RVIZ_CFG" -f "$FRAME" >/dev/null 2>&1 || \
  ros2 run rviz2 rviz2 -d "$RVIZ_CFG" >/dev/null 2>&1
fi
wait $PUB 2>/dev/null
