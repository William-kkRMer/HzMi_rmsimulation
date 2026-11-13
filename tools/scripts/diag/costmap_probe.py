#!/usr/bin/env python3
"""costmap_probe.py —— 把 **局部/全局两张代价图** 在同一块地方并排打出来（ASCII），
用来回答"为什么全局过得去、局部过不去"到底是"有障碍(253)"还是"只是膨胀(1~252)"。

用法（仿真跑起来、车开到卡住的位置时）：
  python3 tools/scripts/diag/costmap_probe.py            # 默认窗口 ±1.5 m，1 Hz
  python3 tools/scripts/diag/costmap_probe.py 2.5        # 窗口 ±2.5 m

判读：
  '#' = 致命(≥253，真障碍/内切)   'X' = 高代价(≥128)   数字 = 低代价(膨胀)    '.' = 自由   ' ' = 未知
  两张图都以**车为中心**、按各自帧打印 ⇒ 同一行同一列就是同一块地方：
    · 若局部 '#/X' 而全局 '.'/'数字' ⇒ 局部确实多了障碍（那就是"高度/感知"问题，配合 voxel_z_stats 看 z）；
    · 若两张图都只是数字、但局部数字更密/更高 ⇒ 纯膨胀问题（宽度），不是高度。
"""
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener


def ascii_window(grid, cx, cy, half):
    res = grid.info.resolution
    n = int(round(2 * half / res))
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    w, h = grid.info.width, grid.info.height
    data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
    lines = []
    lethal = high = 0
    for j in range(n, -1, -1):
        y = cy - half + j * res
        row = ''
        for i in range(n + 1):
            x = cx - half + i * res
            col = int((x - ox) / res)
            rr = int((y - oy) / res)
            if not (0 <= col < w and 0 <= rr < h):
                row += '?'
                continue
            c = data[rr, col]
            if c < 0:
                row += ' '
            elif c >= 253:
                row += '#'; lethal += 1
            elif c >= 128:
                row += 'X'; high += 1
            elif c > 0:
                row += str(min(9, max(1, c // 26)))
            else:
                row += '.'
        lines.append('%6.2f %s' % (y, row))
    return lines, lethal, high


class Probe(Node):
    def __init__(self, half, at=None):
        super().__init__('costmap_probe')
        self.half = half
        self.at = at
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        # ★ 2026-11-02：nav2 的 costmap_raw 是 **volatile** 发布 ⇒ 订阅端必须 requested=VOLATILE，
        #   否则 TRANSIENT_LOCAL 订阅与 VOLATILE 发布**QoS 不兼容**（一个字节都收不到）。
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.local = None
        self.glob = None
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap_raw',
                                 lambda m: setattr(self, 'local', m), qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap_raw',
                                 lambda m: setattr(self, 'glob', m), qos)
        # 兜底：raw 话题若因 QoS/未开而不来，退到普通 costmap
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: setattr(self, 'local', m), qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'glob', m), qos)
        self.create_timer(1.0, self.tick)

    def robot_in(self, frame):
        for src in ('base_link_fake', 'base_link'):
            try:
                t = self.tf.lookup_transform(frame, src, rclpy.time.Time())
                return (t.transform.translation.x, t.transform.translation.y)
            except Exception:
                continue
        return None

    def tick(self):
        if self.at is not None:
            for name, grid in (('LOCAL', self.local), ('GLOBAL', self.glob)):
                if grid is None:
                    self.get_logger().info('%s: 还没收到 costmap' % name); continue
                cx, cy = self.at
                res = grid.info.resolution
                col = int((cx - grid.info.origin.position.x) / res)
                row = int((cy - grid.info.origin.position.y) / res)
                if not (0 <= col < grid.info.width and 0 <= row < grid.info.height):
                    self.get_logger().info('%s(%s): 点(%.2f,%.2f) 不在本图范围'
                                           % (name, grid.header.frame_id, cx, cy)); continue
                c = int(np.asarray(grid.data, dtype=np.int16).reshape(
                    grid.info.height, grid.info.width)[row, col])
                lines, lethal, high = ascii_window(grid, cx, cy, 0.5)
                self.get_logger().info(
                    '\n===== %s(%s) 世界点 (%.2f, %.2f) 的代价 = %d %s | 周围0.5m lethal=%d high=%d =====\n%s'
                    % (name, grid.header.frame_id, cx, cy, c,
                       '(致命≥253)' if c >= 253 else ('(高≥128)' if c >= 128 else '(可通行)'),
                       lethal, high, '\n'.join(lines)))
            return
        for name, grid in (('LOCAL', self.local), ('GLOBAL', self.glob)):
            if grid is None:
                self.get_logger().info('%s: 还没收到 costmap_raw' % name)
                continue
            frame = grid.header.frame_id
            rp = self.robot_in(frame)
            if rp is None:
                self.get_logger().warn('%s: 取不到 %s 下的车体位姿' % (name, frame))
                continue
            lines, lethal, high = ascii_window(grid, rp[0], rp[1], self.half)
            self.get_logger().info(
                '\n===== %s costmap (%s) 车=(%.2f,%.2f) 窗口±%.1fm | lethal=%d high=%d =====\n%s'
                % (name, frame, rp[0], rp[1], self.half, lethal, high, '\n'.join(lines)))


def main():
    # 用法二：`costmap_probe.py --at X Y` —— 直接查**世界坐标**处的代价（门/缝隙定点诊断）
    at = None
    if len(sys.argv) >= 4 and sys.argv[1] == '--at':
        at = (float(sys.argv[2]), float(sys.argv[3]))
    half = float(sys.argv[1]) if (len(sys.argv) > 1 and at is None and not sys.argv[1].startswith('-')) else 1.5
    rclpy.init()
    n = Probe(half, at)
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
