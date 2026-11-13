#!/usr/bin/env bash
# =============================================================================
# preflight.sh —— `sim_backend:=gz` 的**上线前体检**（只读，不改任何东西）
#
# 为什么需要：gz 档的失败模式大多是"环境缺件"，而报错分散在 launch / 构建 / 渲染三处。
# 这个脚本把它们收成一张 PASS/FAIL 表，并给出每一行的修复命令。
#
# 用法：  tools/scripts/gzsim/preflight.sh
# 退出码：0 = 全 PASS（可以起 gz 档）；1 = 有 FAIL（按提示补）
# =============================================================================
set -uo pipefail
fails=0

ok()   { printf '  \033[32mPASS\033[0m  %s\n' "$*"; }
bad()  { printf '  \033[31mFAIL\033[0m  %s\n' "$*"; fails=$((fails+1)); }
warn() { printf '  \033[33mWARN\033[0m  %s\n' "$*"; }

echo "== 1. Gazebo Fortress =="
if ver=$(ign gazebo --versions 2>/dev/null | head -1) && [ -n "$ver" ]; then
  ok "ign gazebo = $ver"
else
  bad "找不到 ign gazebo（需要 ignition-gazebo6）"
fi

echo "== 2. Fortress 开发头（构建 rmagine 插件要用） =="
if [ -d /usr/include/ignition/gazebo6 ]; then ok "/usr/include/ignition/gazebo6"; else bad "缺 ignition-gazebo6 开发头"; fi

echo "== 3. ROS↔GZ 桥 =="
if ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then ok "ros_gz_bridge $(ros2 pkg prefix ros_gz_bridge)"; else bad "缺 ros-gz-bridge（apt: ros-humble-ros-gz-bridge）"; fi

echo "== 4. 第三方 MID-360 gz 后端（libmid360_system.so） =="
prefix="${GZSIM_PREFIX:-$HOME/gzsim_ws/install}"
if [ -f "$prefix/lib/libmid360_system.so" ]; then
  ok "$prefix/lib/libmid360_system.so"
else
  bad "没找到 $prefix/lib/libmid360_system.so"
  echo "        修：tools/scripts/gzsim/fetch_and_patch_mid360.sh（需 github 可达）"
  echo "        或：export GZSIM_PREFIX=/已有构建前缀"
fi

echo "== 5. Embree ≥ 4（rmagine 依赖；apt 只有 3.12） =="
emb=$(ls -d /usr/include/embree4 2>/dev/null || true)
if [ -n "$emb" ]; then ok "系统 Embree4: $emb"; else
  warn "系统无 Embree4 ⇒ rmagine 会在 configure 时用 FetchContent 联网拉取（需要出网）"
fi

echo "== 6. 渲染能力（gz 的 Sensors 系统需要**真的** GL 上下文） =="
# ⚠️ 只看 DISPLAY 会给假 PASS：本机实测 DISPLAY=:1 存在，但 Xvfb 没有 GLX，
#    gz 一开 gpu_lidar 就 `X Error ... Major opcode 152 (GLX)`。
#    所以这里**真跑一个 5 步的最小世界**（带 gpu_lidar），看 stderr 有没有 GLX 报错。
if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
  ok "DISPLAY=${DISPLAY:-} WAYLAND=${WAYLAND_DISPLAY:-}（存在）"
else
  bad "没有 DISPLAY/WAYLAND ⇒ gpu_lidar 一定起不来"
fi
probe_dir=$(mktemp -d)
cat > "$probe_dir/glprobe.sdf" <<'SDF'
<?xml version="1.0"?>
<sdf version="1.9"><world name="glprobe">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size></physics>
  <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
  <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <model name="rig"><static>true</static><pose>0 0 1 0 0 0</pose><link name="l">
    <sensor name="lidar" type="gpu_lidar"><topic>probe</topic><update_rate>10</update_rate>
      <lidar><scan><horizontal><samples>16</samples><min_angle>-3.14</min_angle><max_angle>3.14</max_angle></horizontal>
      <vertical><samples>2</samples><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></vertical></scan>
      <range><min>0.2</min><max>10</max></range></lidar>
    </sensor></link></model>
</world></sdf>
SDF
if command -v ign >/dev/null 2>&1; then
  _part="gzpre_$$"
  probe_out=$(GZ_PARTITION="$_part" IGN_PARTITION="$_part" timeout 20 \
      ign gazebo -s -r --iterations 5 "$probe_dir/glprobe.sdf" 2>&1 | tr -d '\0')
  if echo "$probe_out" | grep -qiE "GLX|X Error|Ogre.*(fail|error)|Unable to create.*render"; then
    bad "gz 传感器渲染失败（真跑最小世界）：$(echo "$probe_out" | grep -iE 'GLX|X Error|Ogre' | head -1)"
    echo "        说明：DISPLAY 存在 ≠ 有 GLX。修：用带真实 GPU/桌面的会话；"
    echo "              或装 mesa 的 llvmpipe+EGL（Fortress 6.18 不支持 --headless-rendering）"
  else
    ok "gz 传感器渲染探针通过（$([ -z "$probe_out" ] || echo "无 GL 报错")）"
  fi
fi
rm -rf "$probe_dir"

echo "== 7. 本仓 gz 侧资产（不依赖上面任何一项，纯静态） =="
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
for f in src/simulation/rm_simulation/rm_livox_gz_sim/worlds/rmuc2027_gz.sdf \
         src/simulation/rm_simulation/rm_livox_gz_sim/worlds/rmuc2026_gz.sdf \
         src/simulation/rm_simulation/rm_livox_gz_sim/scripts/make_robot_gz_sdf.py \
         src/simulation/rm_simulation/rm_livox_gz_sim/scripts/make_world_gz_sdf.py \
         src/simulation/rm_simulation/rm_gz_planar_move/src/planar_move_system.cpp; do
  [ -f "$repo/$f" ] && ok "$f" || bad "缺 $f"
done
if ( cd "$repo" && colcon list 2>/dev/null | grep -q '^rm_gz_planar_move' ); then ok "包 rm_gz_planar_move 在索引里"; else warn "colcon 索引里没有 rm_gz_planar_move（source install/setup.bash 后再试）"; fi

echo
if [ "$fails" -eq 0 ]; then
  printf '\033[32m结论：全 PASS —— 可以跑 sim_backend:=gz\033[0m\n'; exit 0
else
  printf '\033[31m结论：%d 项 FAIL —— 先按上面的"修"处理\033[0m\n' "$fails"; exit 1
fi
