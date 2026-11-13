#!/usr/bin/env bash
# 一键启动本栈仿真（环境 + 自检 + launch）
#
# 用法：
#   ./tools/scripts/run_sim.sh                 # 默认：patchwork 地面分割 + STVL 局部体素层
#   ./tools/scripts/run_sim.sh stvl            # 同上（显式写出，便于 A/B 记录）
#   ./tools/scripts/run_sim.sh scan            # 只吃 /scan 的旧档（对照用）
#   ./tools/scripts/run_sim.sh stvl_both       # STVL + scan + cloud 三源（CPU 最贵）
#   ./tools/scripts/run_sim.sh stvl nogui      # 无 gzclient
#
# ⚠️ `rogmap` 档已**废弃**（2026-11-11 决定）：
#    进程内 ROGMap 的语义层在空场地上会给出大量 ambiguous→OCCUPIED，
#    且 free_count 恒为 0、unknown 占近半，调参成本远高于收益。
#    ⇒ 地图输入改回 patchwork + STVL。代码仍留在 src/vendor/rog_map，仅不再启动。
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOCAL_OBSTACLE="${1:-stvl}"
GUI="${2:-gui}"

# 兼容旧写法：第一个参数若是 mask_impl 语义（terrain/mask/rogmap），
# 则按下面的映射转成局部障碍层档位，避免旧命令静默跑成别的配置。
case "$LOCAL_OBSTACLE" in
  rogmap)
    echo "✗ rogmap 档已废弃（2026-11-11）：地图输入改回 patchwork + STVL。" >&2
    echo "  用: ./tools/scripts/run_sim.sh stvl" >&2
    exit 1 ;;
  terrain)
    # ★ 2026-11-13：**terrain 档已弃用**（用户决定）。terrain_layer 从未实测通过，
    #   节点已从 bringup_sim.launch.py 移除 ⇒ 传了只会静默跑成别的配置，这里直接拒。
    echo "✗ terrain 档已弃用（2026-11-13）：障碍层统一用 costmap（STVL→costmap→plan_mask）。" >&2
    echo "  用: ./tools/scripts/run_sim.sh stvl" >&2
    exit 1 ;;
  mask)
    # 老命令的 mask_impl 语义 ⇒ 障碍层统一用 stvl
    LOCAL_OBSTACLE="stvl" ;;
esac

# ★★★ 2026-11-12：默认改成 **costmap** —— 即"STVL 的 2D 结果二值化后喂给 T-DT"。
#   原因（已核对代码）：
#     T-DT 规划器/控制器只吃 /map + /plan_mask（planner.cpp:122-126、
#     controller.cpp:81-88），**不读 costmap 代价**；
#     而 STVL 是 costmap 插件，成果只写进 Costmap2D
#     ⇒ 不桥接的话 STVL 对 T-DT 完全不生效（只影响 RViz）。
#   旧档保留可切：mask_impl:=terrain（3D 体素+坡/可钻语义，未实测）| mask（z 带一刀切）
MASK="costmap"

cd "$REPO"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH 2>/dev/null || true

source /opt/ros/humble/setup.bash
source install/setup.bash
source .tmp_cache/zlio_ws/install/setup.bash

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RCUTILS_LOGGING_BUFFERED_STREAM=1

for p in frontend rm_tdt_planner rm_nav_bringup; do
  if ! ros2 pkg prefix "$p" >/dev/null 2>&1; then
    echo "✗ 找不到包 $p —— 检查 source 顺序" >&2
    exit 1
  fi
done

echo "✓ 环境就绪: mask_impl=$MASK local_obstacle=$LOCAL_OBSTACLE gui=$GUI"
echo "  frontend        = $(ros2 pkg prefix frontend)"
echo "  rm_tdt_planner  = $(ros2 pkg prefix rm_tdt_planner)"
echo "  障碍链路        = patchwork(/segmentation/obstacle) → cloud_crop"
echo "                    → STVL(3D体素) → /global_costmap/costmap"
echo "                    → plan_mask_from_costmap(二值化, 只认 253/254)"
echo "                    → /plan_mask → T-DT 规划器+控制器（两者共用同一张 2D 图）"
# ★★★ 2026-11-12：**原来这行是错的**。
#   它写的是 pointcloud_to_laserscan 的 min/max_height（那只是 /scan 的切片），
#   而 T-DT 根本不读 /scan。真正决定"什么高度算障碍"的是 STVL 每个源的
#   `min_obstacle_height / max_obstacle_height`（见下方两行，直接读参数文件）。
_band() { python3 - "$1" <<'EOF'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))['global_costmap']['global_costmap']['ros__parameters']['stvl_layer']
v = d.get('livox_mark', {})
print(f"{v.get('min_obstacle_height')} ~ {v.get('max_obstacle_height')}")
EOF
}
# ★★★ 2026-11-13：**可切换地图**（launch 默认 = map/RMUC2027_cad.yaml，即 P 过的 CAD 图）。
#   用法： MAP_YAML=/abs/path/xxx.yaml ./tools/scripts/run_sim.sh
#   launch 里 `map_yaml` 是 LaunchConfiguration ⇒ 传进去即生效。
MAP_YAML="${MAP_YAML:-}"
# ★★★ 2026-11-13：**仿真 UWB 开关**。UWB=1 时：
#   · 后台起 tools/scripts/nav/sim_uwb_from_gt.py（把 /odom_ground_truth 转成 /uwb_pose）
#   · backend_config 换成 tools/configs/zlio_backend_sim_uwb.yaml（uwb.enabled=true）
#   目的：用"绝对位置观测"压住纯激光的定位跳变。
UWB="${UWB:-0}"
# ★★★ 2026-11-13：**三套一键模式**（走 bringup_mode.launch.py）：
#   MODE=tdt  T-DT 主线（planner=tdt + nav=rm_tdt_local）—— 默认就是这个组合
#   MODE=bit  BIT 的 MINCO 插件对（实验；自动起两条 topic 中继）
#   MODE=std  原版 nav2（navfn + dwb），作对照基准
MODE="${MODE:-}"
BACKEND_CONFIG="${BACKEND_CONFIG:-}"
if [ "$UWB" = "1" ]; then
  BACKEND_CONFIG="${BACKEND_CONFIG:-$REPO/tools/configs/zlio_backend_sim_uwb.yaml}"
  echo "  UWB             = 开（$BACKEND_CONFIG）"
elif [ -n "$BACKEND_CONFIG" ]; then
  echo "  后端配置        = $BACKEND_CONFIG"
fi
if [ -n "$MAP_YAML" ]; then
  if [ ! -f "$MAP_YAML" ]; then
    echo "✗ MAP_YAML 指向的文件不存在: $MAP_YAML" >&2; exit 1
  fi
  echo "  地图            = $MAP_YAML （MAP_YAML 覆盖）"
else
  echo "  地图            = launch 默认（map/RMUC2027_cad.yaml = P 过的 CAD 图）"
fi

echo "  障碍高度切片    = STVL global_costmap: $(_band "$REPO/src/nav/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml") m (相对 base_link)"
echo "                    ⚠ 这是一条**固定切片**：完全高于它的悬空障碍看不见 ⇒ 无法判限高"

if [ "$GUI" = "nogui" ]; then GUI_ARG="false"; else GUI_ARG="true"; fi

# ══════════════════════════════════════════════════════════════════════════
# ★★★ 2026-11-12：**给 gzserver 独占核**。
#
#   为什么：这台机器 28 核、load 只有 0.5，但 RTF 依旧很低。
#   原因不是"资源不够"，而是 **Gazebo classic 的物理 + 传感器生成是单线程**：
#   多核帮不上忙，而 28 个 ROS 进程会不停把这个线程挤下 CPU
#   （上下文切换 + cache 抖动）⇒ 单位真实时间推进的仿真步数变少 ⇒ RTF 掉。
#   把 gzserver 钉在固定的核上，能减少迁移与抢占；gzclient（若开）单独钉另一组。
#
#   核分配（可用 GZ_CORES / GZCLIENT_CORES 覆盖）：
#     gzserver   : 0-3   （物理+传感器，单线程为主，给 4 个核足够）
#     gzclient   : 4-7   （渲染，nogui 时不存在）
#     其余 ROS   : 不绑定，由内核调度到 8-27
#   注：renice 降 nice 需要 CAP_SYS_NICE，普通用户会失败 ⇒ 失败就跳过，不报错。
# ══════════════════════════════════════════════════════════════════════════
GZ_CORES="${GZ_CORES:-0-3}"
GZCLIENT_CORES="${GZCLIENT_CORES:-4-7}"

pin_proc() {   # $1=可执行文件名(精确)  $2=核集合  $3=说明
  # ⚠ 必须用 `pgrep -x`（**精确进程名**）而不是 `pgrep -f`（命令行匹配）：
  #   实测 `pgrep -f gzserver` 会匹配到**调用脚本自身/其它含该字符串的进程**，
  #   曾把 **PID 1** 绑到 0-3（已恢复）。这里再做二次校验：/proc/<pid>/exe 的
  #   文件名必须与期望一致，否则拒绝绑定。
  local pat="$1" cores="$2" label="$3" pid=""
  for _ in $(seq 1 120); do
    pid=$(pgrep -x "$pat" | head -1)
    [ -n "$pid" ] && break
    sleep 0.5
  done
  if [ -z "$pid" ]; then
    echo "  [pin] 未找到 $label，跳过" >&2
    return 0
  fi
  # 二次校验：真正的可执行文件必须就是它，防止误绑别的进程
  local real
  real=$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)
  case "$real" in
    */"$pat") : ;;
    *) echo "  [pin] pid=$pid 的可执行文件是 '$real'，不是 $pat ⇒ 拒绝绑定" >&2
       return 0 ;;
  esac
  if taskset -pc "$cores" "$pid" >/dev/null 2>&1; then
    echo "  [pin] $label(pid=$pid) → 核 $cores"
  else
    echo "  [pin] $label(pid=$pid) 绑定失败（权限？），继续" >&2
  fi
  renice -n -5 -p "$pid" >/dev/null 2>&1 || true   # 无权限时静默跳过
}

# 后台守护：等 gzserver/gzclient 起来后立即绑核；不阻塞 launch
( pin_proc "gzserver" "$GZ_CORES" "gzserver"
  [ "$GUI_ARG" = "true" ] && pin_proc "gzclient" "$GZCLIENT_CORES" "gzclient"
) &
PIN_WATCHER=$!
# ★ 2026-11-13：原来这里只 kill pin watcher ⇒ **Ctrl-C 或关终端时 post_run 不执行**，
#   于是 by_layer/ 和 failures.log 不会生成（实测：02:09 那次缺这两样）。
#   现在 EXIT/INT/TERM/HUP 都调 post_run（内部幂等，只会真正跑一次）。
trap 'kill $PIN_WATCHER 2>/dev/null; post_run' EXIT INT TERM HUP
echo "  CPU 绑定        = gzserver[$GZ_CORES] gzclient[$GZCLIENT_CORES]（可用 GZ_CORES/GZCLIENT_CORES 覆盖）"

# ══════════════════════════════════════════════════════════════════════════
# ★★★ 2026-11-12：**日志落盘**。原来只打到终端 ⇒ 关掉终端/滚屏就全丢，
#   而且 ROS 默认把 per-node 日志写到 `$HOME/.ros/log`（本机 ~/.ros 只读，
#   写不进去）⇒ 事后完全无法复盘。
#
#   现在每次运行一个独立目录：
#     .tmp_diag/run/<时间戳>/launch.log   ← 所有节点的**合并**输出（终端同款）
#     .tmp_diag/run/<时间戳>/roslog/      ← ROS **每节点独立**的 .log（自动分文件）
#     .tmp_diag/run/<时间戳>/roshome/     ← 可写的 ROS_HOME（绕开只读的 ~/.ros）
#
#   合并日志与独立日志**同时**存在：
#     · 想 grep 整条链（按 tag 混排的时间线）→ launch.log
#     · 想看单个节点、排除其它节点刷屏 → roslog/<node>_<pid>_<stamp>.log
#   `tee` 保证终端照常实时显示，Ctrl-C 同时结束 launch 与 tee。
# ══════════════════════════════════════════════════════════════════════════
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$REPO/.tmp_diag/run/$RUN_STAMP"
mkdir -p "$RUN_DIR/roslog" "$RUN_DIR/roshome"
export ROS_LOG_DIR="$RUN_DIR/roslog"
export ROS_HOME="$RUN_DIR/roshome"
LOG_FILE="$RUN_DIR/launch.log"
# 分层汇总 CSV（nav_stage_monitor 每 2 s 追加一行；用环境变量注入，不用改 launch）
export NAV_LAYER_CSV="$RUN_DIR/layers.csv"
echo "  本次日志目录    = $RUN_DIR"
echo "                    合并: launch.log   分层: by_layer/   每节点: roslog/   CSV: layers.csv"
echo "                    事后看: tail -f \"$LOG_FILE\""

UWB_PID=""
post_run() {
  # ★ 2026-11-13：**幂等**——EXIT/INT/TERM/HUP 都会调它，只能真正跑一次。
  [ "${POST_RAN:-0}" = "1" ] && return 0
  POST_RAN=1
  if [ -n "$UWB_PID" ]; then kill "$UWB_PID" 2>/dev/null || true; fi
  kill $READY_WATCHER 2>/dev/null
  # ★ 按"层"切分合并日志（一层可能跨多个节点，与 roslog/ 的按节点是两回事）
  "$REPO/tools/scripts/nav/split_run_log.py" "$RUN_DIR" 2>/dev/null || true
  # ★ 只保留最近 30 次运行（按目录名=时间戳排序）
  local keep=30
  local n
  n=$(ls -1d "$REPO"/.tmp_diag/run/*/ 2>/dev/null | wc -l)
  if [ "$n" -gt "$keep" ]; then
    ls -1dt "$REPO"/.tmp_diag/run/*/ 2>/dev/null | tail -n +$((keep + 1)) |
      while read -r d; do rm -rf "$d"; done
    echo "  [日志] 已清理，保留最近 $keep 次"
  fi
  echo "  [日志] 本次: $RUN_DIR"
}

# ★★★ 2026-11-12（实测）：**就绪横幅**。
#   启动要 ~178 s（8 个 lifecycle 节点逐个激活，每个约 5 s），而速度链的最后一环
#   `velocity_smoother` 是**最后**激活的。在它之前点目标 ⇒ 控制器照常算指令，
#   但 `/cmd_vel` 没有数据 ⇒ `车身命令 0.00`、车不动（19:43 那次就是这样被我误判成"配置问题"）。
#   这里在后台轮询，全部 active 时打一条绿底横幅；与 launch 共用同一个终端/日志。
( ./tools/scripts/nav/wait_ready.sh ) &
READY_WATCHER=$!

EXTRA_ARGS=()
[ -n "$MAP_YAML" ] && EXTRA_ARGS+=("map_yaml:=$MAP_YAML")
[ -n "$BACKEND_CONFIG" ] && EXTRA_ARGS+=("backend_config:=$BACKEND_CONFIG")

LAUNCH_FILE="bringup_zlio.launch.py"
if [ -n "$MODE" ]; then
  # 模式档：mask_impl / local_obstacle 由 bringup_mode 决定，这里不再传（避免互相打架）
  LAUNCH_FILE="bringup_mode.launch.py"
  EXTRA_ARGS+=("mode:=$MODE")
  EXTRA_ARGS+=("gui:=$GUI_ARG")
  EXTRA_ARGS+=("stage_test:=${STAGE_TEST:-all}")
  echo "  模式            = $MODE（$LAUNCH_FILE）"
else
  EXTRA_ARGS+=("mask_impl:=$MASK")
  EXTRA_ARGS+=("local_obstacle:=$LOCAL_OBSTACLE")
  EXTRA_ARGS+=("gui:=$GUI_ARG")
fi

if [ "$UWB" = "1" ]; then
  ( ./tools/scripts/nav/sim_uwb_from_gt.py --rate 20 --noise 0.02 ) &
  UWB_PID=$!
fi

  # ★★★ 2026-11-13（**用户反馈"起不来又没法 Ctrl-C"**）：整棵进程树放进**独立进程组**。
  #   之前直接 `ros2 launch ... | tee`：
  #     · launch 一报错就整体退出，gzserver/rviz2 有时变孤儿进程留在后台；
  #     · Ctrl-C 只打到前台管道，残留还在，终端看着像"卡住"。
  #   现在：setsid 独立进程组 ⇒ 退出/Ctrl-C 按**进程组**收（kill -- -PGID），
  #   收尾再兜底 TERM→KILL，保证 gzserver/rviz2/nav2 一个不剩。
  set +e
  ARGSTR=""
  for a in "${EXTRA_ARGS[@]}"; do ARGSTR="$ARGSTR $(printf '%q' "$a")"; done
  setsid bash -c "ros2 launch rm_nav_bringup '$LAUNCH_FILE' gravity:=base $ARGSTR 2>&1 | tee -a '$LOG_FILE'" &
  LAUNCH_PID=$!
  trap 'echo; echo "  [中断] 正在结束整个进程组…"; kill -INT -$LAUNCH_PID 2>/dev/null' INT TERM

  wait $LAUNCH_PID
  LAUNCH_RC=$?

  if kill -0 -$LAUNCH_PID 2>/dev/null; then
    echo "  [收尾] 进程组仍有残留 → TERM"
    kill -TERM -$LAUNCH_PID 2>/dev/null
    for _ in 1 2 3 4 5; do sleep 1; kill -0 -$LAUNCH_PID 2>/dev/null || break; done
    kill -KILL -$LAUNCH_PID 2>/dev/null
  fi
  set -e

  if [ "$LAUNCH_RC" -ne 0 ]; then
    echo
    echo "  ✗ launch 异常退出（rc=$LAUNCH_RC）——错误原文："
    grep -E "\[ERROR\]|Traceback|Caught exception" "$LOG_FILE" 2>/dev/null | tail -6 | sed 's/^/    /' || true
    echo "    完整日志: $LOG_FILE"
  fi

  post_run
