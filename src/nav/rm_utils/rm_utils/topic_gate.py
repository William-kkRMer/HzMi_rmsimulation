#!/usr/bin/env python3
"""topic_gate.py —— 启动前置门：确认"上游真的在发"再算启动成功。

为什么需要：`ros2 launch lio_sam sim.launch.py` **单独跑**（没有 bringup / 没有仿真）时，
`use_sim_time` 拿不到 `/clock`、`/livox/imu` 也没有数据 ⇒ LIO-SAM 的 GTSAM 会抛
`IndeterminantLinearSystemException`（变量 id 是一串乱码 = 时间戳退化），
看起来像"崩溃"，实际是"上游没起"。这个门把这句话直接打进日志。

参数：
  topics    : 逗号分隔，`名称:类型` 或只写名称（默认按 sensor_msgs/msg/Imu 之外的通用订阅）
  timeout_s : 等多久（double）
  min_msgs  : 每个话题至少收到几条（默认 1）
  strict    : true ⇒ 有话题没数据就 ERROR 并 exit 1
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class TopicGate(Node):
    def __init__(self) -> None:
        super().__init__('topic_gate')
        self.declare_parameter('topics', '/clock,/livox/imu')
        self.declare_parameter('timeout_s', 20.0)
        self.declare_parameter('min_msgs', 1)
        self.declare_parameter('strict', False)
        self.counts: dict[str, int] = {}
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST)

    def _cb(self, name: str, _msg) -> None:
        self.counts[name] = self.counts.get(name, 0) + 1

    def run(self) -> int:
        raw = str(self.get_parameter('topics').value or '')
        names = [t.strip() for t in raw.split(',') if t.strip()]
        timeout_s = float(self.get_parameter('timeout_s').value or 20.0)
        min_msgs = int(self.get_parameter('min_msgs').value or 1)
        strict = bool(self.get_parameter('strict').value)

        # 类型未知时用 Generic 订阅成本高；这里对常见话题用具体类型，其余用 "任意"
        import importlib
        subs = []
        for spec in names:
            name, _, typ = spec.partition(':')
            typ = typ.strip() or 'sensor_msgs/msg/Imu'
            try:
                mod_name, _, cls_name = typ.replace('/', '.').rpartition('.')
                mod = importlib.import_module(mod_name)
                cls = getattr(mod, cls_name)
            except Exception:                                   # noqa: BLE001
                self.get_logger().error('topic_gate: 无法解析类型 %s（%s）' % (typ, name))
                continue
            self.counts[name] = 0
            subs.append(self.create_subscription(
                cls, name, lambda m, n=name: self._cb(n, m), self.qos))

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.counts.get(n, 0) >= min_msgs for n in self.counts):
                break

        missing = [n for n in self.counts if self.counts[n] < min_msgs]
        if not missing:
            self.get_logger().info(
                'topic_gate: OK — 上游都在发：%s' %
                ', '.join('%s×%d' % (n, self.counts[n]) for n in self.counts))
            return 0
        msg = ('topic_gate: **缺上游数据** %s ⇒ 本 launch 的前提不成立。'
               '常见原因：没有 bringup（`/clock` 不存在 ⇒ use_sim_time 时间戳退化 ⇒ '
               'LIO-SAM 的 GTSAM 抛 IndeterminantLinearSystemException）、'
               '或仿真/驱动没起。请用 bringup_sim.launch.py 启动整栈。' % ', '.join(missing))
        if strict:
            self.get_logger().error(msg)
            return 1
        self.get_logger().warn(msg)
        return 0


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = TopicGate()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                                       # noqa: BLE001
            pass
    return code


if __name__ == '__main__':
    sys.exit(main())
