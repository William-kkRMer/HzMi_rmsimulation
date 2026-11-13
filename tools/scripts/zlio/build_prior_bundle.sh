#!/usr/bin/env bash
# 从 PCD 生成 z-localization 的 prior bundle（关键帧 + Scan Context 描述子）
# 用法: tools/scripts/zlio/build_prior_bundle.sh <input.pcd> [out_dir] [sensor_z]
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
IN="${1:?input pcd}"
OUT="${2:-$ROOT/.tmp_cache/prior/bundle}"
SZ="${3:-0.35}"
BIN="$ROOT/.tmp_cache/zlio_ws/install/backend/lib/backend/build_pcd_map_bundle"
[ -x "$BIN" ] || { echo "缺少 build_pcd_map_bundle：先构建 .tmp_cache/zlio_ws"; exit 1; }
# sensor-z 必须与 backend.yaml 的 common.scan_context.lidar_height 一致（默认 0.35），否则拒绝加载
"$BIN" --input "$IN" --out-bundle "$OUT" \
  --x-min -20.0 --x-max 8.0 --y-min -9.0 --y-max 6.7 --sensor-z "$SZ" \
  --query-step 1.0 --bundle-step 1.0 --map-voxel 0.1 --scan-voxel 0.1 \
  --descriptor-type scan_context --occlusion true
echo "bundle: $OUT"
