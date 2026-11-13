#!/usr/bin/env bash
# 等"全部导航节点 active"，然后打一条显著的 READY 横幅。
#
# 为什么需要（2026-11-12 实测）：
#   nav2 的 lifecycle_manager 在本机**逐个**激活 8 个节点，每个约 5 s
#   （RTF≈0.2 时更慢）⇒ 整个启动约 **178 s**。而速度链是：
#       controller_server → cmd_vel_nav → velocity_smoother → /cmd_vel → 底盘
#   `velocity_smoother` 是**最后一个**被激活的节点。
#   如果在这之前点目标：
#       · controller_server 照常算出指令（日志里有 `cmd: vx=-1.78 vy=-2.42`）
#       · 但 smoother 还没 active ⇒ 它的输出发布器未激活 ⇒ `/cmd_vel` 没有数据
#       · 于是 `[VEL] 车身命令 0.00`、`L2局部:FAILED(ctrl_v=0)`、车不动
#   实测三组：
#       185818: 目标 +174s / smoother +178s → 先丢 4 s 指令，之后正常
#       194309: 目标 +168s / smoother +178s → 只丢了 10 s，而这轮总共才跑 12 s ⇒ "完全不动"
#       193538: 目标 +110s / smoother +101s → 正常
#
# 用法：由 run_sim.sh 在后台启动（输出与 launch 同一终端）；也可手动跑。
set +u
NODES=(map_server controller_server smoother_server planner_server \
       behavior_server bt_navigator waypoint_follower velocity_smoother)

t0=$(date +%s)
last_beat=0
while true; do
  ok=0; missing=()
  for n in "${NODES[@]}"; do
    st=$(timeout 6 ros2 lifecycle get "/$n" 2>/dev/null | head -1)
    case "$st" in
      active*) ok=$((ok+1)) ;;
      *) missing+=("$n") ;;
    esac
  done
  now=$(date +%s); el=$((now - t0))
  if [ "$ok" -eq "${#NODES[@]}" ]; then
    printf '\n\033[1;42;30m                                                            \033[0m\n'
    printf '\033[1;42;30m   全部 %d 个导航节点已 active —— 现在可以点目标了           \033[0m\n' "${#NODES[@]}"
    printf '\033[1;42;30m                                                            \033[0m\n\n'
    exit 0
  fi
  if [ $((now - last_beat)) -ge 20 ]; then
    last_beat=$now
    printf '[ready] %3ds  已激活 %d/%d  等待: %s\n' \
      "$el" "$ok" "${#NODES[@]}" "$(IFS=,; echo "${missing[*]}")"
  fi
  if [ "$el" -gt 420 ]; then
    printf '[ready] ✗ 420s 仍未全部激活（缺 %s）—— 别再等了，看日志\n' "$(IFS=,; echo "${missing[*]}")" >&2
    exit 1
  fi
  sleep 5
done
