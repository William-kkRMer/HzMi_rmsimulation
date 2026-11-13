#!/usr/bin/env bash
# _batch_k7.sh —— 量"传感器离地高度"（= linefit `sensor_height` 的物理真值）
# 只做一件事：跑一次静止栈 + dump（带 use_sim_time 的 TF 回读），供离线核对。

# ★ 2026-11-13：**自定位仓库根**（原来写死 /home/weicheng/...，换个目录就废）。
#   从脚本所在目录逐级上溯，找到 .git 即为仓库根。
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
while [ ! -d "$REPO/.git" ] && [ "$REPO" != "/" ]; do REPO="$(dirname "$REPO")"; done

set +u
cd "$REPO"
tools/scripts/tiltmount/run_costmap_input_dump.sh k7_geo --settle 30 --duration 8 --frames 3 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False \
    2>&1 | tail -8
echo "=== 批次 k7 完成"
