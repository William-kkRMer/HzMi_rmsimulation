#!/usr/bin/env python3
"""livox_to_lio_sam_node —— 把 Livox `CustomMsg` 转成 **LIO-SAM 期望的 Velodyne 格式 PointCloud2**。

为什么需要：LIO-SAM（`src/localization/rm_localization/lio_sam`，官方 ros2 分支）的 `imageProjection` 把点云
反序列化成 `VelodynePointXYZIRT`（PCD 字段固定为 x,y,z,intensity,ring,time），且
`sensor:=livox` 走的也是这条路径（`imageProjection.cpp:235`）。我们的雷达只发
`livox_ros_driver2/CustomMsg`（x,y,z,reflectivity,offset_time,tag,line）⇒ 必须转一次。

字段映射：
  x/y/z        ← CustomPoint.x/y/z（float32）
  intensity    ← CustomPoint.reflectivity（uint8 → float32）
  ring         ← CustomPoint.line（uint8 → uint16；LIO-SAM 只用它做 N_SCAN 分环）
  time         ← CustomPoint.offset_time * 1e-9（**秒**，相对帧首；LIO-SAM 用它做去畸变）

⚠️ `offset_time` 只有在仿真插件打开 **`LIVOX_INTRA_FRAME_TIME=1`** 时才非零；否则全 0
   ⇒ LIO-SAM 认为"帧内无运动"（不去畸变），与改造前等价。

用法：
  ros2 run cloud_accumulator livox_to_lio_sam \
      --ros-args -p in_topic:=/livox/lidar -p out_topic:=/lio_sam/points
"""
from __future__ import annotations

import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField

try:
    from livox_ros_driver2.msg import CustomMsg
except Exception:  # 没装 livox 消息包时也能 import（只是不能用）
    CustomMsg = None

# 与 LIO-SAM 的 VelodynePointXYZIRT 逐字段对应（顺序即内存布局）
_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
    PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1),
]
# struct '<ffffHf' = 4+4+4+4+2+4 = **22 字节**（'<' = 标准尺寸、不做对齐填充）
_POINT_STEP = 22


class LivoxToLioSam(Node):
    def __init__(self):
        super().__init__('livox_to_lio_sam')
        self.declare_parameter('in_topic', '/livox/lidar')
        self.declare_parameter('out_topic', '/lio_sam/points')
        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, out_topic, qos)
        if CustomMsg is None:
            self.get_logger().error('没有 livox_ros_driver2 消息包 ⇒ 无法转换')
            return
        # ★ 2026-11-04：**订阅必须 BEST_EFFORT**。mid360 保真档的插件按 `qos=best_effort`
        #   发 `/livox/lidar`，而这里原来用 RELIABLE 订阅 ⇒ QoS 不兼容 ⇒
        #   `New publisher ... offering incompatible QoS. No messages will be received`
        #   ⇒ /lio_sam/points 一帧都没有 ⇒ LIO-SAM 没关键帧 ⇒ Ctrl-C 存图报
        #   `pcl::IOException: Input point cloud has no data!`。
        #   BEST_EFFORT 订阅对 best_effort 和 reliable 的发布者都兼容。
        sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CustomMsg, in_topic, self.on_cloud, sub_qos)
        self.get_logger().info('livox_to_lio_sam: %s (CustomMsg) → %s (PointCloud2 xyz/intensity/ring/time)'
                               % (in_topic, out_topic))
        self._warned_zero = False

    def on_cloud(self, msg):
        n = len(msg.points)
        if n == 0:
            return
        offs = np.zeros(n, dtype=np.float32)
        nonmono = 0
        prev = -1.0
        buf = bytearray(n * _POINT_STEP)
        for i, p in enumerate(msg.points):
            t = p.offset_time * 1e-9
            if t < prev:
                nonmono += 1
            prev = t
            offs[i] = t
            struct.pack_into('<ffffHf', buf, i * _POINT_STEP,
                             p.x, p.y, p.z, float(p.reflectivity),
                             int(p.line) & 0xFFFF, float(t))
        if nonmono == 0 and offs[-1] == 0.0 and not self._warned_zero:
            self._warned_zero = True
            self.get_logger().warn('offset_time 全为 0 ⇒ LIO-SAM 不会去畸变。'
                                   '想启用请 export LIVOX_INTRA_FRAME_TIME=1 后重启仿真')

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n
        out.fields = _FIELDS
        out.is_bigendian = False
        out.point_step = _POINT_STEP
        out.row_step = _POINT_STEP * n
        out.is_dense = True
        out.data = bytes(buf)
        self.pub.publish(out)


def main():
    rclpy.init()
    n = LivoxToLioSam()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
