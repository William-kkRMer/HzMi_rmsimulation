#!/usr/bin/env bash
# 分层单点测试：一次只验一层。
#   ./stage_test.sh l0      # 感知：局部图有没有内容、占用率是否合理
#   ./stage_test.sh l1      # 全局：能否规划、路径多长
#   ./stage_test.sh l2      # 局部：Kastar/轨迹样本的峰值速度
#   ./stage_test.sh l3      # 控制：指令速度与来源
#   ./stage_test.sh all     # 四层状态一起看（5 秒）
set -u
STAGE="${1:-all}"
case "$STAGE" in
  l0) T=/nav/l0/status ;;
  l1) T=/nav/l1/status ;;
  l2) T=/nav/l2/status ;;
  l3) T=/nav/l3/status ;;
  all) T="/nav/l0/status /nav/l1/status /nav/l2/status /nav/l3/status" ;;
  *) echo "用法: $0 [l0|l1|l2|l3|all]"; exit 2 ;;
esac

echo "=== 话题健康状况 ==="
for t in $T; do
  n=$(timeout 2 ros2 topic info "$t" 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')
  printf "  %-20s publisher=%s\n" "$t" "${n:-0}"
done

echo
echo "=== 最近一次状态（5 s 超时）==="
for t in $T; do
  echo "--- $t"
  timeout 5 ros2 topic echo --once "$t" 2>/dev/null \
    | grep -E "stage|state|detail|keys|values|latency" -A0 | head -12
done

cat <<'TIP'

读法：
  0 感知 / 1 全局 / 2 局部 / 3 控制
  state: 1=OK  2=DEGRADED(有输出但可疑)  3=FAILED(该层自己判废)
  values 与 keys 一一对应（occupied/ratio/points/v ...）

示例定位：
  l0 state=3 ratio≈1.0   ⇒ 局部图全障碍（z 过滤带/点云问题），与控制无关
  l1 state=3             ⇒ 规划失败（A* 无解/被拒绝），看 /nav/l1/dbg/*
  l3 state=3 detail=fallback ⇒ 控制器没拿到轨迹，看 l2
TIP
