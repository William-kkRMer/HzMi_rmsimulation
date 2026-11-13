#!/usr/bin/env bash
# =============================================================================
# pack_2d.sh —— 把**已经验证过的 2D 方案**打成一个可搬到别的电脑的包
#
# 三种模式（都已在本仓验证过）：
#   mapping  = slam_toolbox 在线建 2D 图（不加载磁盘图、不起重定位）
#   nav      = 加载磁盘 2D 图 + **AMCL 重定位**（默认 beluga_amcl，LOCALIZATION=amcl 切 nav2_amcl）
#   slam_nav = 边建边导航（在线 SLAM 直接喂 costmap，不加载磁盘图）
#
# 用法：
#   tools/scripts/deploy/pack_2d.sh              # → dist/rm_2d_<日期>.tar.gz
#   tools/scripts/deploy/pack_2d.sh --no-sim     # 不带仿真资产（上实车/纯算法机）
#   tools/scripts/deploy/pack_2d.sh --with-pcd   # 额外带 PCD（gicp/sc 用，默认不带）
# =============================================================================
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.." || exit 1

OUT_ROOT=${OUT_ROOT:-dist}
WITH_SIM=1
WITH_PCD=0
for a in "$@"; do
  case "$a" in
    --no-sim) WITH_SIM=0 ;;
    --with-pcd) WITH_PCD=1 ;;
    *) echo "未知参数: $a" >&2; exit 2 ;;
  esac
done

STAMP=$(date +%Y%m%d_%H%M)
NAME=rm_2d_$STAMP
DST=$OUT_ROOT/$NAME
rm -rf "$DST"; mkdir -p "$DST"

echo "== 1/5 复制源码（排除 bag / PCD / attic / 大 posegraph）"
mkdir -p "$DST/src"
tar -C . -cf - \
    --exclude='src/bringup/rm_nav_bringup/bag' \
    --exclude='src/bringup/rm_nav_bringup/PCD' \
    --exclude='src/bringup/rm_nav_bringup/map/attic' \
    --exclude='src/bringup/rm_nav_bringup/map/RMUC2027_v2.data' \
    --exclude='src/bringup/rm_nav_bringup/map/RMUC2027_v2.posegraph' \
    --exclude='src/bringup/rm_nav_bringup/map/RMUL*' \
    --exclude='src/bringup/rm_nav_bringup/map/RMUC.*' \
    --exclude='__pycache__' \
    --exclude='.git' \
    src | tar -C "$DST" -xf -
if [ "$WITH_SIM" = 0 ]; then
  echo "   --no-sim：删除 rm_simulation"
  rm -rf "$DST/src/simulation/rm_simulation"
fi

echo "== 2/5 放地图（已验证的 2D 图）"
for f in RMUC2027_v2.pgm RMUC2027_v2.yaml RMUC2027_v2.meta.yaml; do
  cp -a "src/bringup/rm_nav_bringup/map/$f" "$DST/src/bringup/rm_nav_bringup/map/" 2>/dev/null || true
done
if [ "$WITH_PCD" = 1 ]; then
  echo "   --with-pcd：带 PCD（不含 attic）"
  mkdir -p "$DST/src/bringup/rm_nav_bringup/PCD"
  cp -a src/bringup/rm_nav_bringup/PCD/*.pcd src/bringup/rm_nav_bringup/PCD/*.yaml \
        "$DST/src/bringup/rm_nav_bringup/PCD/" 2>/dev/null || true
fi

echo "== 3/5 工具与文档"
tar -C . -cf - --exclude='__pycache__' tools docs | tar -C "$DST" -xf -

echo "== 4/5 生成 build/run 脚本与 README"
cat > "$DST/build.sh" <<'EOS'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
sudo apt install -y \
  ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-beluga-amcl \
  ros-humble-nav2-amcl ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-tf2-ros ros-humble-imu-complementary-filter \
  libeigen3-dev libpcl-dev libceres-dev 2>/dev/null || true
[ -d src ] || { echo "✗ 在包根目录执行（应有 src/）"; exit 1; }
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "✓ 编译完成：source install/setup.bash 后再跑 run_*.sh"
EOS

for m in mapping nav slam_nav; do
  cat > "$DST/run_$m.sh" <<EOS
#!/usr/bin/env bash
# 模式：$m
set -euo pipefail
cd "\$(dirname "\$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \\
  sim_backend:=\${SIM_BACKEND:-classic} \\
  world:=\${WORLD:-RMUC2027} map_name:=\${MAP_NAME:-RMUC2027_v2} \\
  mode:=$m \\
  lio:=small_point_lio gravity:=tf mapper:=slam_toolbox \\
  localization:=\${LOCALIZATION:-beluga} \\
  nav:=mppi planner:=smac2d ground:=patchwork \\
  local_obstacle:=stvl global_obstacle:=stvl \\
  robot:=chassis spin_speed:=0.0 \\
  map_yaml:="\$PWD/src/bringup/rm_nav_bringup/map/RMUC2027_v2.yaml" \\
  gui:=True nav_rviz:=True
EOS
done
chmod +x "$DST"/*.sh

cat > "$DST/README.md" <<'EOM'
# 2D 方案部署包（mapping / nav / slam_nav + AMCL 重定位）

## 这是什么
把**已验证的 2D 链路**（slam_toolbox 建图 → 2D 栅格图 → AMCL 重定位 → nav2 规划控制）
打成一个可搬迁的工作区。**3D（LIO-SAM/GICP/SC）不在这个包里**。

## 重定位用的是哪个 AMCL
- **默认 `beluga_amcl`**（AMCL 的现代实现：同资产、同契约、同 lifecycle 形态）；
- 切回经典 `nav2_amcl`：`LOCALIZATION=amcl ./run_nav.sh`。

## 依赖
ROS 2 Humble + nav2 + slam_toolbox + `ros-humble-beluga-amcl` + pcl_ros（`build.sh` 里有 apt 列表）。

## 三条命令
```bash
./build.sh
./run_mapping.sh     # 建图：slam_toolbox 在线建 2D 栅格
./run_nav.sh         # 导航：加载磁盘 2D 图 + AMCL 重定位
./run_slam_nav.sh    # 边建边导航：在线 SLAM 直接喂 costmap
```

## 常用覆盖
```bash
SIM_BACKEND=classic|gz         # 默认 classic（2D 就是在这上面验证的）
WORLD=RMUC2027 MAP_NAME=RMUC2027_v2
LOCALIZATION=beluga|amcl       # 只在 nav 模式生效
```

## nav 模式必做：给 AMCL 初值
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 5.9, y: 2.72, z: 0}, orientation: {w: 1.0}}}}'
```

## 验证点（30 秒）
```bash
ros2 topic hz /scan                 # ~10 Hz
ros2 topic hz /map                  # mapping/slam_nav 在长；nav 是静态图
ros2 topic echo /amcl_pose --once   # nav 模式应有位姿
ros2 run tf2_ros tf2_echo map base_link
```

## 排错
| 现象 | 原因 |
|---|---|
| `map→odom` 不出现 | nav 模式没给 `/initialpose`，或 `map_yaml` 路径不对 |
| `/scan` 没数据 | `lio`/点云链没起（包内默认 `lio:=small_point_lio`） |
| 编译缺包 | `rosdep install --from-paths src -y --ignore-src` |
EOM

echo "== 5/5 清单与打包"
{
  echo "package : $NAME"
  echo "created : $(date -Iseconds)"
  echo "commit  : $(git rev-parse --short HEAD 2>/dev/null || echo '-')"
  echo "with_sim: $WITH_SIM   with_pcd: $WITH_PCD"
  echo "files   : $(find "$DST" -type f | wc -l)"
  echo "size    : $(du -sh "$DST" | cut -f1)"
  echo
  echo "--- 顶层 ---"
  ls -1 "$DST"
} > "$DST/MANIFEST.txt"
tar -C "$OUT_ROOT" -czf "$OUT_ROOT/$NAME.tar.gz" "$NAME"
echo; cat "$DST/MANIFEST.txt"; echo
echo "✓ 已生成：$OUT_ROOT/$NAME.tar.gz（$(du -h "$OUT_ROOT/$NAME.tar.gz" | cut -f1)）"
