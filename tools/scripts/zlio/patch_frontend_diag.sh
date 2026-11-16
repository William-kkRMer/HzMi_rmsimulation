#!/usr/bin/env bash
# 给 zlio 前端加一行"每帧诊断"日志（幂等）。
#
# 为什么需要这个脚本：
#   zlio 在 `.tmp_cache/zlio_ws`（**gitignore**），源码改动不会进 git；
#   这样别人/以后的自己重建工作区后，这一行不会丢。
#
# 加的是什么：
#   `[diag] frame: matches=… lidar_updated=… cloud=… cov_trace=…`
#   全部是**上游已经算好**的量（matches/lidar_updated 本来就在 FrontendFrame 消息里），
#   只是前端原来一行都不打印 ⇒ 排查"跟丢/这帧没用雷达"时看不到。
#
# 用法： tools/scripts/zlio/patch_frontend_diag.sh [工作区路径]
set -euo pipefail
WS="${1:-$HOME/HzMi_rmsimulation/.tmp_cache/zlio_ws}"
F="$WS/src/frontend/apps/node.cpp"
[ -f "$F" ] || { echo "  找不到 $F"; exit 1; }

python3 - "$F" <<'PY'
import sys
p=sys.argv[1]
t=open(p).read()
if '[diag] frame:' in t:
    print('  已打过补丁（含 [diag] frame）'); raise SystemExit(0)
lines=t.split('\n')
try:
    i=[k for k,l in enumerate(lines) if l.strip()=='for (const auto& result : results) {'][0]
except IndexError:
    print('  ✗ 没找到结果循环（上游代码结构变了？）'); raise SystemExit(1)
block='''      // ★★★ 2026-11-17：把上游已经算好、只是不打印的量打出来（1 Hz）。
      //   matches / lidar_updated 本来就在 FrontendFrame 消息里；cov_trace 是 15 维误差协方差的迹。
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "[diag] frame: matches=%zu lidar_updated=%d cloud=%zu cov_trace=%.4f",
          result.matches, static_cast<int>(result.lidar_updated), result.cloud.size(),
          result.estimate.covariance.trace());'''.split('\n')
lines[i+1:i+1]=block
open(p,'w').write('\n'.join(lines))
print('  ✓ 已插入 [diag] frame 日志')
PY
echo "  重新编译： cd $WS && colcon build --packages-select frontend --cmake-args -DCMAKE_BUILD_TYPE=Release"
