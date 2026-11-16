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
block='''      // ★★★ 2026-11-17：前端诊断三件套（全部用上游已经算好的量，不新增计算）
      //   ① 1 Hz 日志  ② CSV（$ROS_LOG_DIR/zlio_frames.csv）  ③ 协方差迹超阈值 WARN
      {
        const double cov_trace = result.estimate.covariance.trace();
        const double t_sec = static_cast<double>(result.estimate.state.time) * 1e-9;
        const auto& pos = result.estimate.state.pose.position;
        static std::ofstream csv;
        static bool csv_ready = false;
        static double cov_warn = -1.0;
        if (cov_warn < 0.0) {
          const char* w = std::getenv("ZLIO_COV_WARN");
          cov_warn = (w && *w) ? std::atof(w) : 0.5;
        }
        if (!csv_ready) {
          csv_ready = true;
          const char* env = std::getenv("ZLIO_DIAG_CSV");
          std::string path;
          if (env && *env) {
            path = env;
          } else {
            const char* ld = std::getenv("ROS_LOG_DIR");
            path = std::string(ld && *ld ? ld : "/tmp") + "/zlio_frames.csv";
          }
          csv.open(path, std::ios::app);
          if (csv) {
            if (csv.tellp() == 0) csv << "t_ros,matches,lidar_updated,cloud,cov_trace,x,y,z\n";
            RCLCPP_INFO(get_logger(), "[diag] CSV → %s", path.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "[diag] CSV 打不开：%s（只打日志）", path.c_str());
          }
        }
        if (csv) {
          csv << t_sec << ',' << result.matches << ',' << static_cast<int>(result.lidar_updated)
              << ',' << result.cloud.size() << ',' << cov_trace << ',' << pos.x() << ','
              << pos.y() << ',' << pos.z() << '\n';
          csv.flush();
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[diag] frame: matches=%zu lidar_updated=%d cloud=%zu cov_trace=%.4f",
            result.matches, static_cast<int>(result.lidar_updated), result.cloud.size(), cov_trace);
        if (cov_trace > cov_warn) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "[diag] 协方差迹 %.3f > %.2f（matches=%zu lidar_updated=%d）⇒ 定位不确定性偏大",
              cov_trace, cov_warn, result.matches, static_cast<int>(result.lidar_updated));
        }
      }'''.split('\n')
lines[i+1:i+1]=block
open(p,'w').write('\n'.join(lines))
print('  ✓ 已插入 [diag] frame 日志')
PY
echo "  重新编译： cd $WS && colcon build --packages-select frontend --cmake-args -DCMAKE_BUILD_TYPE=Release"
