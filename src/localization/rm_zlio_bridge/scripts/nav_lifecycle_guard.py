#!/usr/bin/env python3
"""启动兜底：等定位就绪后，把 nav2 里没起来的 lifecycle 节点补成 active。

为什么需要它（2026-11-11 实测）：
    仿真启动瞬间 CPU 被 gzserver + zlio + rog_map + RViz 抢满，
    lifecycle_manager 调 `/smoother_server/change_state` 会超时
    （`failed to send response ... (timeout)`），bringup 就此中断，
    后面的 planner_server 压根没被 configure ⇒
        · /opt_path 一直没有 ⇒ L1 全局 FAILED
        · /rog_map/layer_value 一直没有 ⇒ L0 感知无数据
        · RViz 里点目标毫无反应（bt_navigator 拿不到 planning action）
    这不是配置错误，是启动竞态。本节点只做「事后补齐」，
    不改任何 nav2 参数，也绝不主动去停别人的节点。

策略：
    1. 等 /localization/odometry 连续有数据 2 s（定位活着）
    2. 每 3 s 检查一遍节点状态，未 active 的发 configure → activate
    3. 全部 active 或超过 90 s 后退出（不留后台负担）
"""

import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from rclpy.node import Node

# 依赖顺序：后面的节点要求前面的已激活
NODES = [
    "map_server",
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]

GIVE_UP_AFTER_S = 90.0


class NavLifecycleGuard(Node):
    def __init__(self):
        super().__init__("nav_lifecycle_guard")
        self.declare_parameter("odom_topic", "/localization/odometry")
        self.declare_parameter("settle_s", 2.0)
        self.declare_parameter("first_delay_s", 8.0)
        self.declare_parameter("repair_interval_s", 3.0)
        self.declare_parameter("give_up_after_s", GIVE_UP_AFTER_S)

        odom_topic = self.get_parameter("odom_topic").value
        self.settle_s = float(self.get_parameter("settle_s").value)
        self.first_delay_s = float(self.get_parameter("first_delay_s").value)
        self.repair_interval_s = float(self.get_parameter("repair_interval_s").value)
        self.give_up_after_s = float(self.get_parameter("give_up_after_s").value)

        # ⚠️ 不能叫 _clients：rclpy 的 Node 内部有同名 list，覆盖它会让 create_client 崩
        self._state_clients: dict[str, dict[str, object]] = {}
        for name in NODES:
            states = []
            for state in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                cli = self.create_client(ChangeState, f"/{name}/change_state")
                states.append(cli)
            self._state_clients[name] = {"clis": states, "last_try": 0.0, "done": False}

        self._odom_ok_since = None
        self._last_odom = None
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self._t0 = self.get_clock().now()
        self._reported = False
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"[nav_guard] 已启动：等 {odom_topic} 稳定 {self.settle_s:.0f}s 后，"
            f"补齐未激活的 nav2 节点（{self.first_delay_s:.0f}s 后才开始，"
            f"最多等 {self.give_up_after_s:.0f}s）")

    # ---- 定位就绪判定 ----
    def _on_odom(self, _msg):
        now = self.get_clock().now()
        if self._last_odom is None or (now - self._last_odom).nanoseconds > 1_000_000_000:
            self._odom_ok_since = now
        self._last_odom = now

    def _odom_ready(self):
        if self._odom_ok_since is None:
            return False
        age = (self.get_clock().now() - self._odom_ok_since).nanoseconds / 1e9
        return age >= self.settle_s

    def _elapsed(self):
        return (self.get_clock().now() - self._t0).nanoseconds / 1e9

    # ---- 主循环 ----
    def _tick(self):
        elapsed = self._elapsed()
        if not self._odom_ready() or elapsed < self.first_delay_s:
            if elapsed > self.give_up_after_s:
                self.get_logger().warn(
                    "[nav_guard] 定位一直没就绪，放弃补齐（请查 zlio 那一段）")
                raise SystemExit(0)
            return

        pending = []
        for name, info in self._state_clients.items():
            if info["done"]:
                continue
            pending.append((name, info))

        if not pending:
            if not self._reported:
                self.get_logger().info("[nav_guard] 全部节点已 active ✅")
                self._reported = True
            raise SystemExit(0)

        for name, info in pending:
            if elapsed - info["last_try"] < self.repair_interval_s:
                continue
            info["last_try"] = elapsed
            if self._try_activate(name, info):
                info["done"] = True
                self.get_logger().info(f"[nav_guard] {name} → active ✅")

        if elapsed > self.give_up_after_s:
            left = [n for n, i in self._state_clients.items() if not i["done"]]
            self.get_logger().error(f"[nav_guard] 放弃，仍未激活: {left}")
            raise SystemExit(0)

    def _try_activate(self, name, info):
        clis = info["clis"]
        # 先 configure（已 configured 的会失败，忽略），再 activate
        for idx, cli in enumerate(clis):
            if not cli.wait_for_service(timeout_sec=1.0):
                return False
            tr = Transition()
            tr.id = Transition.TRANSITION_CONFIGURE if idx == 0 else Transition.TRANSITION_ACTIVATE
            tr.label = "configure" if idx == 0 else "activate"
            req = ChangeState.Request()
            req.transition = tr
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.result() is None:
                # configure 在已激活的节点上必然失败，不是问题
                if idx == 1:
                    return False
        return True


def main():
    rclpy.init()
    node = NavLifecycleGuard()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
