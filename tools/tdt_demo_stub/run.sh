#!/usr/bin/env bash
# 跑 T-DT 原始 demo（main.cpp），不需要 OpenCV 开发包：
#   · 用 tools/tdt_demo_stub/opencv2 的极简桩（只做 imread(PGM)/空绘图）
#   · 用 tools/mpc_proto/.deps/tdt/install 的 OSQP（他们 setup.sh 配方）
# 用法：tools/tdt_demo_stub/run.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TDT="$ROOT/third_party/tdt-nav-kit"
T="$ROOT/tools/mpc_proto/.deps/tdt/install"
OUT="$ROOT/.tmp_cache/tdt_run"
mkdir -p "$OUT/work"
# 桩 imread 读 "<path>.pgm" ⇒ 先把 PNG 转一份
python3 - "$TDT/images/rmuc2025.png" <<'PY'
import sys
from PIL import Image
import numpy as np
src = sys.argv[1]
a = np.array(Image.open(src).convert('L'))
with open(src + '.pgm', 'wb') as f:
    f.write(b'P5\n%d %d\n255\n' % (a.shape[1], a.shape[0]))
    f.write(a.astype(np.uint8).tobytes())
PY
ln -sfn "$TDT/images" "$OUT/images"
INC="-I$ROOT/tools/tdt_demo_stub -I/usr/include/eigen3 -I$T/include -I$T/include/osqp"
cd "$TDT"
g++ -O2 -std=c++20 -pthread -include iostream -include algorithm -include cmath -include array \
  -DOPENCV_ALL_HPP $INC main.cpp src/MinimumSnapOsqp/minimumSnap.cpp src/MinimumSnapOsqp/sfcSquare.cpp -c
g++ -O2 -std=c++20 -pthread -include iostream -include algorithm -include cmath \
  $INC src/YAstar/yastar.cpp src/YAstar/kinodynamicAstar.cpp -c
g++ -O2 -std=c++20 -pthread *.o -L"$T/lib" -losqp -lOsqpEigen -Wl,-rpath,"$T/lib" -o "$OUT/tdt_demo"
rm -f *.o
cd "$OUT/work" && "$OUT/tdt_demo"
