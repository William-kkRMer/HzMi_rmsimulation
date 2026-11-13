#!/usr/bin/env bash
# 构建集成测试（C++20：tdt 源码用了 std::span / <ranges>）
set -euo pipefail
cd "$(dirname "$0")"
DEPS=$PWD/../../../tools/mpc_proto/.deps
QP=$DEPS/opt/ros/humble
[ -f "$QP/lib/libosqp.so" ] || { echo "先跑 tools/mpc_proto/build.sh 取 OSQP"; exit 1; }
g++ -O2 -std=c++20 -pthread -include iostream -include algorithm -include cmath \
  offline_integration.cpp ../src/mpc_core/mpc_core.cpp \
  ../src/tdt/yastar.cpp ../src/tdt/kinodynamicAstar.cpp ../src/tdt/sfcSquare.cpp \
  -I../src/mpc_core -I../src/tdt -I/usr/include/eigen3 \
  -I"$QP/include" -L"$QP/lib" -losqp -Wl,-rpath,"$QP/lib" -o offline_integration
echo "✓ ./offline_integration"
