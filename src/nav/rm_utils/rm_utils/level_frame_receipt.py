#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""level_frame_receipt —— **一次性**断言「下游节点真正生效的水平帧 == launch 派生值」。

为什么需要它（2026-11-03）：
  `odom_level` 只在 `gravity:=tf` 时存在（`gravity:=lio` 时 LIO 自己把 odom 摆平，没有这个帧）。
  但历史上有配置**写死** `odom_level`：
    · `scan_context_relocalizer` 的 `level_frame`
    · `gicp_registration` 的 `ground_filter_frame`（icp / gicp / small_gicp 三个槽位共用）
  写死之后的失败形态是**静默**的：节点照常启动，只是拿一个不存在的帧去摆平/过滤点云，
  甚至直接不干活 ⇒ 表现为"定位偶尔不对/SC 不触发"，很难从日志里看出来。

  现在 launch 用 `_LevelFrame()`（唯一真源 `_level_frame(context)`）**覆盖注入**这两个键。
  但"注入了"不等于"节点读到了" —— 这个节点把**生效值**读回来，与 expected 比对。

做什么（只读：不发话题、不发 TF、不改参数）：
  1. 等目标节点的 `get_parameters` 服务就绪；
  2. 读回 `param`（默认 `level_frame`）；
  3. 与 `expected` 比较；
  4. 打一行 receipt（OK/ERROR），不一致 ⇒ **ERROR + exit 1**（`strict:=True` 时）。

参数：
  · `target_node`  ：节点名（如 `/scan_context_relocalizer`、`/gicp_registration`）
  · `param`        ：键名（`level_frame` / `ground_filter_frame`）
  · `expected`     ：期望值（launch 的 `_LevelFrame()` 求值结果）
  · `timeout_s`    ：等服务/等节点的上限（默认 60 s；节点可能晚起）
  · `strict`       ：True = 不一致就 exit 1
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node


class LevelFrameReceipt(Node):
    def __init__(self):
        super().__init__('level_frame_receipt')
        self.declare_parameter('target_node', '')
        self.declare_parameter('param', 'level_frame')
        self.declare_parameter('expected', '')
        self.declare_parameter('timeout_s', 60.0)
        self.declare_parameter('strict', True)
        self.target = str(self.get_parameter('target_node').value or '')
        self.key = str(self.get_parameter('param').value or 'level_frame')
        self.expected = str(self.get_parameter('expected').value or '')
        self.timeout = float(self.get_parameter('timeout_s').value or 60.0)
        self.strict = bool(self.get_parameter('strict').value)

    def wait_and_check(self) -> int:
        if not self.target:
            self.get_logger().error('level_frame_receipt: target_node 为空 ⇒ 跳过（这是配置错误）')
            return 1 if self.strict else 0
        node_name = self.target if self.target.startswith('/') else '/' + self.target
        client = self.create_client(
            __import__('rcl_interfaces.srv', fromlist=['GetParameters']).GetParameters,
            node_name + '/get_parameters')
        t0 = time.monotonic()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() - t0 > self.timeout:
                self.get_logger().error(
                    'level_frame_receipt: %s/get_parameters %.0f s 内没就绪 '
                    '⇒ 无法断言（节点没起？名字写错？）' % (node_name, self.timeout))
                return 1 if self.strict else 0
        req = __import__('rcl_interfaces.srv', fromlist=['GetParameters']).GetParameters.Request()
        req.names = [self.key]
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.timeout)
        res = fut.result()
        if res is None or not res.values:
            self.get_logger().error(
                'level_frame_receipt: %s 的 %s 读不回来 ⇒ 无法断言' % (node_name, self.key))
            return 1 if self.strict else 0
        val = res.values[0]
        if val.type == 0:   # PARAMETER_NOT_SET
            self.get_logger().error(
                'level_frame_receipt: %s 的 %s **未设置**（not set）⇒ 下游会用默认值，'
                '这不是我们注入的那一个' % (node_name, self.key))
            return 1 if self.strict else 0
        actual = str(val.string_value)
        ok = (actual == self.expected)
        tail = ('%s ⇒ %s\n  node=%s key=%s actual=%r expected=%r'
                % ('OK' if ok else 'ERROR', '一致' if ok else '不一致',
                   node_name, self.key, actual, self.expected))
        if ok:
            self.get_logger().info('level_frame_receipt: ' + tail)
            return 0
        self.get_logger().error(
            'level_frame_receipt: ' + tail +
            '\n  为什么这是 ERROR：水平帧写错是**静默失败**（节点照跑、只是摆平/过滤用错帧）。'
            '\n  该值应来自 launch 的 `_LevelFrame()`（gravity:=tf ⇒ odom_level；gravity:=lio ⇒ odom）。'
            '\n  查：ros2 param get %s %s' % (node_name, self.key))
        return 1 if self.strict else 0


def main(argv=None):
    rclpy.init(args=argv)
    n = LevelFrameReceipt()
    try:
        rc = n.wait_and_check()
    finally:
        try:
            n.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
