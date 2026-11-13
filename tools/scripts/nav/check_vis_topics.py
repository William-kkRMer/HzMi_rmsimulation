#!/usr/bin/env python3
"""RViz 显示自检：对 bit.rviz 里每个显示项的话题，报出
发布者数 / 订阅者数 / 3 秒内是否真的收到数据 / 收到多少帧。

用法（**导航跑着的时候，另开一个终端**）：
    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 tools/scripts/nav/check_vis_topics.py

判读：
  发布者 0            → 上游根本没发（不是 RViz 的问题）
  发布者 >0 但 0 帧    → QoS 不兼容 / 只在特定条件下发
  有帧但 RViz 不显示   → 显示项配置（Fixed Frame、Class、Enabled）
"""

import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# (标签, 话题, 期望 QoS) —— 与 bit.rviz 中的显示项一一对应
TOPICS = [
    ("地图 /map", "/map", "transient_local"),
    ("L1 MINCO 轨迹 /opt_path_vis", "/opt_path_vis", "volatile"),
    ("L1 SMAC 路径 /astar_path_vis", "/astar_path_vis", "transient_local"),
    ("L1 候选(被拒) /minco_candidate_path_vis", "/minco_candidate_path_vis", "transient_local"),
    ("L2 MPC 预测 /mpc_predict_path", "/mpc_predict_path", "volatile"),
    ("L2 MPC 实际 /mpc_real_path", "/mpc_real_path", "volatile"),
    ("nav2 /plan", "/plan", "volatile"),
    ("L0 语义层 /rog_map/layer_value", "/rog_map/layer_value", "best_effort"),
]


def main():
    rclpy.init()
    node = rclpy.create_node("vis_topic_check")

    # 用 rosidl_runtime_py 的通用订阅不方便，这里直接用 count 判断
    from rosidl_runtime_py.utilities import get_message

    counts = {t: 0 for _, t, _ in TOPICS}

    def make_cb(topic):
        def cb(_msg):
            counts[topic] += 1
        return cb

    for _, topic, mode in TOPICS:
        qos = QoSProfile(depth=1)
        if mode == "transient_local":
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        elif mode == "best_effort":
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            qos.durability = DurabilityPolicy.VOLATILE
        else:
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.VOLATILE

        try:
            msg_type = get_message("nav_msgs/msg/OccupancyGrid" if "layer_value" in topic or topic == "/map"
                                   else "nav_msgs/msg/Path")
            node.create_subscription(msg_type, topic, make_cb(topic), qos)
        except Exception as exc:  # noqa: BLE001
            print(f"  !! 无法订阅 {topic}: {exc}")

    # ★ 前置检查：导航没起来时，所有 "上游没发" 都是假象（进程不在了）
    import subprocess
    dead = []
    for n in ("planner_server", "controller_server", "bt_navigator"):
        try:
            out = subprocess.run(["ros2", "lifecycle", "get", "/" + n],
                                 capture_output=True, text=True, timeout=5).stdout.strip()
        except Exception:  # noqa: BLE001
            out = ""
        if "active" not in out:
            dead.append(f"{n}: {out or '不存在'}")
    if dead:
        print("！！ 导航未就绪，以下结果不可用于判断话题链路 ！！")
        for d in dead:
            print("   ", d)
        print("    先跑: ./tools/scripts/nav/ensure_nav_active.sh")
        print()

    print("采样中（3 秒）……")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    info = node.get_topic_names_and_types()
    pubs, subs = {}, {}
    for name, _types in info:
        pubs[name] = node.count_publishers(name)
        subs[name] = node.count_subscribers(name)

    print()
    print(f"{'显示项':44s} {'发布者':>5s} {'订阅者':>6s} {'3秒帧数':>8s}  判定")
    print("-" * 92)
    problems = []
    for label, topic, _mode in TOPICS:
        p = pubs.get(topic, 0)
        s = subs.get(topic, 0)
        n = counts[topic]
        if p == 0:
            verdict = "❌ 上游没发"
            problems.append((label, "上游没发这个话题"))
        elif n == 0:
            verdict = "❌ 有发布但收不到（QoS/条件）"
            problems.append((label, "有发布者但本节点收不到"))
        else:
            verdict = "✅ 有数据"
        print(f"{label:44s} {p:5d} {s:6d} {n:8d}  {verdict}")

    print()
    if problems:
        print("== 需要处理 ==")
        for label, why in problems:
            print(f"  - {label}: {why}")
    else:
        print("所有话题都有数据 ⇒ 问题在 RViz 显示配置（Fixed Frame / Class / Enabled）")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
