#!/usr/bin/env python3
"""pc2_to_custom_node —— `sensor_msgs/PointCloud2` → **Livox `CustomMsg`**（反向适配）。

为什么需要（两条独立的用途）：
  1. **gz 后端的保底路**：Fortress 原生 `gpu_lidar` 只发 `PointCloudPacked`／`PointCloud2`，
     而本仓链路（`small_point_lio` 的 AVIA 分支、`/livox/lidar` 的所有消费者）吃的是
     `livox_ros_driver2/CustomMsg` ⇒ 用本节点把 gz 的原生雷达接回既有链路
     （不用第三方 `libmid360_system.so` 也能跑通整栈；保真度见
      docs/fortress_sim_backend.md 5.6：丢非重复扫描图案与逐点时间）。
  2. **实车/其它雷达接入**：任何发 PointCloud2 的驱动都能这样并进 `/livox/lidar`。

字段映射（反方向，与 `livox_to_lio_sam_node.py` 对齐）：
  CustomPoint.x/y/z     ← x/y/z
  CustomPoint.reflectivity ← intensity（若有点段则取；否则 0）
  CustomPoint.line      ← ring（若有；否则 0）
  CustomPoint.offset_time ← **time 字段 × 1e9**（秒→纳秒）；无 time 字段则 0
                            （0 = LIO 认为"帧内无运动"，与 Classic 插件默认一致）
  CustomPoint.tag       ← 参数 `tag`（默认 0；`small_point_lio` 要求 (tag & 0x3f)==0）

用法：
  ros2 run cloud_accumulator pc2_to_custom --ros-args \
      -p in_topic:=/gz/lidar/points -p out_topic:=/livox/lidar
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
except Exception:  # pragma: no cover
    CustomMsg = None
    CustomPoint = None

# ⚠️ 键必须是 **PointField 的整型常量**（datatype 是 int，不是字符串）——
#    第一版按 'FLOAT32' 做键，实测 KeyError: 7（= PointField.FLOAT32=7）。
_STRUCT = {
    1: ('i1', 1), 2: ('u1', 1), 3: ('i2', 2), 4: ('u2', 2),
    5: ('i4', 4), 6: ('u4', 4), 7: ('f4', 4), 8: ('f8', 8),
}


class Pc2ToCustom(Node):
    def __init__(self):
        super().__init__('pc2_to_custom')
        self.declare_parameter('in_topic', '/gz/lidar/points')
        self.declare_parameter('out_topic', '/livox/lidar')
        self.declare_parameter('tag', 0)
        self.declare_parameter('line_from_ring', True)
        # ★ 2026-11-03（goal 轮 6）：gz 桥出来的 PointCloud2 的 header.frame_id 是 **gz scoped name**
        #   （形如 `<robot>/livox_frame/lidar`），不是 `livox_frame` ⇒ 下游 TF 查不到。
        #   传非空就覆盖成这个帧名（本仓约定 = livox_frame）。
        self.declare_parameter('frame_id', '')
        # ★ 2026-11-03（goal 轮 6）：非空时把**同一帧**再以 PointCloud2 转发到该话题。
        #   为什么要它：本仓感知链（patchwork/linefit）吃的是 `/livox/lidar/pointcloud`，
        #   而 gz 原生雷达只给 `/gz/lidar` ⇒ native 模式下用它补齐这个输入，
        #   并把 frame_id 覆盖成 livox_frame（gz 桥出来的是 scoped name）。
        self.declare_parameter('out_pc2_topic', '')
        # ★ 2026-11-03：**合成逐点时间**。gz 的 gpu_lidar 不带 time 字段，
        #   而 LIO（point-lio/LIO-SAM）靠 offset_time 去畸变。这里按"一帧 = 一次扫掠"的
        #   均匀角速度假设，给第 i 个点填 t = i/N / synthetic_time_hz 秒（0 = 关闭）。
        #   近似但有物理依据（水平角速度恒定）；扫描顺序=行优先（每环内按方位角递增）。
        self.declare_parameter('synthetic_time_hz', 0.0)
        # 每帧的**水平采样数**（gz gpu_lidar 的 PointCloudPacked 是 **ring-major**：
        #   i = ring*h_samples + azimuth_idx ⇒ 时间应按 **方位角** 走，而不是按整体点序！
        #   第一版按 i/N 铺开 ⇒ 每圈只占 1/N_SCAN 的时长 ⇒ 去畸变完全错 ⇒ /odom 出 NaN
        #   ⇒ lio_tf_adapter 发 NaN TF ⇒ LIO-SAM 的 GTSAM 直接 IndeterminantLinearSystem 崩。
        self.declare_parameter('synthetic_h_samples', 640)
        # 合成时间的**算法**：azimuth（推荐）= 按点的方位角 atan2(y,x) 归一化到 [0,2π)；
        #   index = 按存储顺序（ring-major 假设）。azimuth 与存储顺序无关，且物理上就是
        #   "旋转雷达扫到这个角度的时间"，比 index 可靠得多。
        self.declare_parameter('synthetic_time_mode', 'azimuth')
        in_t = self.get_parameter('in_topic').value
        out_t = self.get_parameter('out_topic').value
        self.tag = int(self.get_parameter('tag').value) & 0xFF
        self.use_ring = bool(self.get_parameter('line_from_ring').value)
        self.frame_id = str(self.get_parameter('frame_id').value or '')
        self.pc2_topic = str(self.get_parameter('out_pc2_topic').value or '')
        self.synth_hz = float(self.get_parameter('synthetic_time_hz').value or 0.0)
        self.synth_h = int(self.get_parameter('synthetic_h_samples').value or 0)
        self.synth_mode = str(self.get_parameter('synthetic_time_mode').value or 'azimuth')

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(CustomMsg, out_t, qos) if CustomMsg else None
        # ★ 顺序坑：必须**在 qos 定义之后**创建（第一版提前了，实测 UnboundLocalError: qos）
        self.pc2_pub = (self.create_publisher(PointCloud2, self.pc2_topic, qos)
                        if self.pc2_topic else None)
        if CustomMsg is None:
            self.get_logger().error('没有 livox_ros_driver2 消息包 ⇒ 无法转换')
            return
        # 订阅端放宽：gz 桥的 QoS 常是 BEST_EFFORT
        self.create_subscription(PointCloud2, in_t, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info('pc2_to_custom: %s (PointCloud2) → %s (CustomMsg, tag=%d)'
                               % (in_t, out_t, self.tag))
        self._warned_no_time = False
        self._warned_synth = False

    def _field(self, msg, name):
        for f in msg.fields:
            if f.name == name:
                return f
        return None

    def on_cloud(self, msg: PointCloud2):
        n = msg.width * msg.height
        if n == 0:
            return
        fx, fy, fz = self._field(msg, 'x'), self._field(msg, 'y'), self._field(msg, 'z')
        if fx is None or fy is None or fz is None:
            self.get_logger().warn('点云缺 x/y/z 字段，跳过', throttle_duration_sec=5.0)
            return
        fi = self._field(msg, 'intensity')
        fr = self._field(msg, 'ring') if self.use_ring else None
        ft = self._field(msg, 'time')

        def col(f):
            if f is None:
                return None
            dt, sz = _STRUCT[f.datatype]
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
            return raw[:, f.offset:f.offset + sz].copy().view(dt).reshape(n)

        x, y, z = col(fx).astype(np.float32), col(fy).astype(np.float32), col(fz).astype(np.float32)
        inten = col(fi) if fi is not None else np.zeros(n, np.float32)
        inten = np.clip(inten.astype(np.float32), 0, 255).astype(np.uint8)
        if fr is not None:
            line = np.clip(col(fr), 0, 255).astype(np.uint8)
        else:
            line = np.zeros(n, np.uint8)
        if ft is not None:
            # 秒 → 纳秒：**用 rint 而不是截断**（0.010*1e9=9999999.999… ⇒ 截断会丢 1 ns）
            t = np.rint(col(ft).astype(np.float64) * 1e9)
            t = np.clip(t, 0, 2 ** 32 - 1).astype(np.uint32)
        else:
            if self.synth_hz > 0.0:
                # 合成：一帧扫掠时间 T = 1/hz。
                period_ns = 1e9 / self.synth_hz
                if self.synth_mode == 'azimuth':
                    # 按**方位角**：t = ((atan2(y,x) 归一到 [0,2π)) / 2π) * T
                    # 与存储顺序无关 ⇒ 不依赖 ring-major/azimuth-major 的猜测。
                    az = np.arctan2(y.astype(np.float64), x.astype(np.float64))
                    az = np.mod(az, 2.0 * np.pi)
                    t = np.rint(az / (2.0 * np.pi) * period_ns)
                else:
                    # 按存储顺序（ring-major）：azimuth_idx = i % h_samples
                    h = self.synth_h if self.synth_h > 1 else n
                    idx = np.arange(n, dtype=np.float64) % float(h)
                    t = np.rint(idx / float(h) * period_ns)
                t = np.clip(t, 0, 2 ** 32 - 1).astype(np.uint32)
                if not self._warned_synth:
                    self._warned_synth = True
                    self.get_logger().info(
                        '点云无 time 字段 ⇒ **合成** offset_time（%.1f Hz 扫掠，%d 点/帧，末点 %d ns）'
                        % (self.synth_hz, n, int(t[-1])))
            else:
                if not self._warned_no_time:
                    self._warned_no_time = True
                    self.get_logger().warn(
                        '点云无 time 字段 ⇒ offset_time 全 0（LIO 不去畸变）。'
                        '想让 LIO 去畸变：-p synthetic_time_hz:=10')
                t = np.zeros(n, np.uint32)

        if self.pc2_pub is not None:
            relay = PointCloud2()
            relay.header = msg.header
            if self.frame_id:
                relay.header.frame_id = self.frame_id
            relay.height, relay.width = msg.height, msg.width
            relay.fields, relay.is_bigendian = msg.fields, msg.is_bigendian
            relay.point_step, relay.row_step = msg.point_step, msg.row_step
            relay.is_dense, relay.data = msg.is_dense, msg.data
            self.pc2_pub.publish(relay)

        out = CustomMsg()
        out.header = msg.header
        if self.frame_id:
            out.header.frame_id = self.frame_id
        out.timebase = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        out.point_num = int(n)
        out.lidar_id = 0
        pts = []
        for i in range(n):
            p = CustomPoint()
            p.offset_time = int(t[i])
            p.x, p.y, p.z = float(x[i]), float(y[i]), float(z[i])
            p.reflectivity = int(inten[i])
            p.tag = self.tag
            p.line = int(line[i])
            pts.append(p)
        out.points = pts
        self.pub.publish(out)


def main():
    rclpy.init()
    n = Pc2ToCustom()
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
