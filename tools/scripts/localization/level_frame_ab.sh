#!/usr/bin/env bash
# =============================================================================
# level_frame_ab.sh —— `gravity:=tf`（有 odom_level）vs `gravity:=lio`（无中间帧）A/B
#
# 背景：odom_level 只解决两件事 —— ① 把倾斜的 odom 摆平；② 让 z 原点落在地面（-0.138）。
#   `gravity:=lio` 让 LIO 自己把 odom 摆平，等价功能、少一个中间帧（z 语义单一）。
#   代价：局部代价图 / p2l / SC / GICP 的帧名都要跟着变（launch 已用 `_LevelFrame()` 统一）。
#
# 这个脚本只做**一件事**：把两种档各跑一遍，采集可比的数字，输出到 CSV。
#
# 用法：
#   tools/scripts/localization/level_frame_ab.sh            # 两档各 120 s
#   GRAV_B_SECONDS=60 tools/scripts/localization/level_frame_ab.sh
#
# 判据（看 CSV）：
#   · odom_jitter_m ：`/odom` 相对 `/odom_ground_truth` 的**位移增量误差** p95（越小越好）
#   · scan_hz       ：`/scan` 频率（p2l 有没有掉帧；< 6 Hz 说明 TF 跟不上）
#   · level_frame   ：SC/GICP 回读到的生效帧（应分别为 odom_level / odom）
# =============================================================================
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.." || exit 1
OUT=${OUT:-.tmp_cache/level_frame_ab}
SECONDS_PER_RUN=${GRAV_B_SECONDS:-120}
mkdir -p "$OUT"
export ROS_LOG_DIR=${ROS_LOG_DIR:-$PWD/$OUT/logs}
export ROS_HOME=${ROS_HOME:-$PWD/$OUT/home}
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
# ★ 2026-11-03：**必须 source**（否则 `Package 'rm_nav_bringup' not found` —— 第一次跑就这么挂了）
if [ -f install/setup.bash ]; then
  # shellcheck disable=SC1091
  # ⚠️ colcon 的 setup.bash 在 `set -u` 下会报 `COLCON_TRACE: 未绑定的变量`（实测）⇒ 临时关 -u
  set +u
  source install/setup.bash
  set -u
else
  echo "✗ 没有 install/setup.bash ⇒ 先 colcon build" >&2; exit 1
fi

run_one() {
  local grav="$1" tag="grav_$1" dom=$((90 + RANDOM % 20))
  echo "=== [$tag] gravity:=$grav ${SECONDS_PER_RUN}s（ROS_DOMAIN_ID=$dom）==="
  # ★ 2026-11-03 修：原来的 `( export …\n source …\n ros2 launch … & )` 写法里
  #   `&` 落在最后一条命令上，subshell 内 `source` 失败时 launch 根本没起、日志也不生成。
  #   现在用"环境变量前缀 + 直接后台"的写法，并且**先确认 /clock 有数据**再计时。
  ( ROS_DOMAIN_ID=$dom ros2 launch rm_nav_bringup bringup_sim.launch.py \
      world:=RMUC2027 map_name:=RMUC2027_v2 mode:=nav \
      lio:=small_point_lio gravity:="$grav" localization:=beluga \
      nav:=mppi planner:=smac2d ground:=patchwork \
      local_obstacle:=stvl global_obstacle:=stvl robot:=chassis \
      spin_speed:=0.0 gui:=False nav_rviz:=False \
      map_yaml:="$PWD/src/bringup/rm_nav_bringup/map/RMUC2027_v2.yaml" \
      > "$OUT/$tag.launch.log" 2>&1 ) &
  local LP=$!
  echo "$LP" > "$OUT/$tag.pid"
  # 等栈起来（最多 90 s）：/clock 有数据才算
  local waited=0
  while [ "$waited" -lt 90 ]; do
    if ROS_DOMAIN_ID=$dom timeout 4 ros2 topic hz /clock > /dev/null 2>&1; then break; fi
    sleep 3; waited=$((waited+3))
  done
  echo "  栈就绪等待 ${waited}s"
  sleep 25
  ROS_DOMAIN_ID=$dom timeout $((SECONDS_PER_RUN-30)) python3 tools/scripts/diag/odom_error.py \
      > "$OUT/$tag.odom.log" 2>&1 || true
  ROS_DOMAIN_ID=$dom timeout 8 ros2 topic hz /scan > "$OUT/$tag.scan_hz.log" 2>&1 || true
  ROS_DOMAIN_ID=$dom timeout 8 ros2 param get /local_costmap/local_costmap global_frame \
      > "$OUT/$tag.local_frame.log" 2>&1 || true
  ROS_DOMAIN_ID=$dom timeout 8 ros2 param get /scan_context_relocalizer level_frame \
      > "$OUT/$tag.sc_level.log" 2>&1 || true
  kill "$LP" 2>/dev/null; wait "$LP" 2>/dev/null
  sleep 3
}

run_one tf
run_one lio

{
  echo "tag,odom_err_p95_m,odom_max_jump_m,scan_hz,local_frame,sc_level_frame"
  for tag in grav_tf grav_lio; do
    e=$(grep -oiE "p95[^0-9]*([0-9.]+)" "$OUT/$tag.odom.log" 2>/dev/null | tail -1 | grep -oE "[0-9.]+" || echo "")
    j=$(grep -oiE "max[^0-9]*([0-9.]+)" "$OUT/$tag.odom.log" 2>/dev/null | tail -1 | grep -oE "[0-9.]+" || echo "")
    s=$(grep -oE "average rate: [0-9.]+" "$OUT/$tag.scan_hz.log" 2>/dev/null | tail -1 | grep -oE "[0-9.]+" || echo "")
    lf=$(sed -n 's/.*value: *//p' "$OUT/$tag.local_frame.log" 2>/dev/null | head -1)
    sc=$(sed -n 's/.*value: *//p' "$OUT/$tag.sc_level.log" 2>/dev/null | head -1)
    echo "$tag,$e,$j,$s,$lf,$sc"
  done
} > "$OUT/summary.csv"
echo; echo "== 汇总 =="; column -s, -t < "$OUT/summary.csv" 2>/dev/null || cat "$OUT/summary.csv"
echo "原始文件在 $OUT/"
