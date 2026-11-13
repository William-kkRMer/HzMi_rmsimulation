#!/usr/bin/env bash
# 构建离线 MPC 原型：不需要 sudo（OSQP 通过 apt-get download + dpkg-deb -x 解到 .deps/）
set -euo pipefail
cd "$(dirname "$0")"
DEPS=$PWD/.deps
if [ ! -f "$DEPS/opt/ros/humble/lib/libosqp.so" ]; then
  echo "== 取 OSQP（不需要 sudo）=="
  mkdir -p "$DEPS/deb"
  ( cd "$DEPS/deb" && apt-get download ros-humble-osqp-vendor >/dev/null 2>&1 )
  deb=$(ls "$DEPS"/deb/*.deb | head -1)
  dpkg-deb -x "$deb" "$DEPS/root"
  ln -sfn "$DEPS/root/opt/ros/humble" "$DEPS/opt_ros" 2>/dev/null || true
  mkdir -p "$DEPS/opt/ros"; rm -rf "$DEPS/opt/ros/humble"; cp -a "$DEPS/root/opt/ros/humble" "$DEPS/opt/ros/humble"
fi
QP=$DEPS/opt/ros/humble
g++ -O2 -std=c++17 mpc.cpp -I"$QP/include" -I/usr/include/eigen3 \
    -L"$QP/lib" -losqp -Wl,-rpath,"$QP/lib" -o mpc_proto
echo "✓ 构建完成：./mpc_proto {straight|narrow|turn|tight}   （MPC_DISTURB=1 加扰动）"
