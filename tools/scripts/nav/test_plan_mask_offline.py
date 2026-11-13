#!/usr/bin/env python3
"""离线验证 plan_mask_from_costmap：不需要 Gazebo / nav2。

构造一张与 /map 同几何的假 costmap，人为塞进
    100 致命 / 99 内切 / 50 膨胀软带 / 0 自由 / -1 未知
    （取值来自 nav2 的 cost_translation_table_：LETHAL→100、INSCRIBED→99）
    默认 **只把 100 变障碍**：99 是 robot_radius(0.22 m) 的内切膨胀，
    而 T-DT 自己有 dilate(0.10)+shrink(0.15)；叠加会把 0.7 m 通道判死（实测）。
然后检查输出的 /plan_mask 是否**只把 100 变成障碍**，其余一律 0。
"""
import sys
import threading

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

W, H, RES, OX, OY = 560, 313, 0.05, -19.90, -8.89

# 每格放一个"探针"，位置固定，便于断言
CASES = [
    ('lethal', 100, 10, 10),
    ('inscribed', 99, 11, 10),
    ('inflated_mid', 50, 12, 10),
    ('inflated_low', 50, 13, 10),
    ('free', 0, 14, 10),
    ('unknown', -1, 15, 10),
]


def make(info_frame='map'):
    m = OccupancyGrid()
    m.header.frame_id = info_frame
    m.info.resolution = RES
    m.info.width = W
    m.info.height = H
    m.info.origin.position.x = OX
    m.info.origin.position.y = OY
    m.data = [0] * (W * H)
    return m


def to_int8(cost):
    """OccupancyGrid.data 是 int8：254 需以 -2 发送，-1 原样。"""
    c = cost & 0xFF if cost >= 0 else cost
    return c - 256 if c > 127 else c


def main():
    rclpy.init()
    node = Node('costmap_faker')

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
    volatile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST)

    map_pub = node.create_publisher(OccupancyGrid, '/map', latched)
    cm_pub = node.create_publisher(OccupancyGrid, '/global_costmap/costmap', volatile)

    got = {}

    def on_mask(msg):
        got['msg'] = msg

    node.create_subscription(OccupancyGrid, '/plan_mask', on_mask, volatile)

    m = make()
    cm = make()
    for _, cost, x, y in CASES:
        cm.data[y * W + x] = to_int8(cost)

    # 先发 /map（基准几何），再发 costmap；重复几轮确保订阅端都收到
    for _ in range(40):
        map_pub.publish(m)
        cm_pub.publish(cm)
        rclpy.spin_once(node, timeout_sec=0.05)
        if 'msg' in got:
            break

    if 'msg' not in got:
        print('失败：没收到 /plan_mask，节点可能没起来')
        return 2

    out = got['msg']
    ok = True
    print(f'{"探针":<14}{"cost 输入":>10}{"mask 输出":>10}   判定')
    for name, cost, x, y in CASES:
        v = out.data[y * W + x]
        expect = 100 if cost == 100 else 0
        good = (v == expect)
        ok &= good
        print(f'{name:<14}{cost:>10}{v:>10}   {"✅" if good else "❌ 期望 " + str(expect)}')

    geo_ok = (out.info.width == W and out.info.height == H
              and abs(out.info.resolution - RES) < 1e-9
              and abs(out.info.origin.position.x - OX) < 1e-6)
    print(f'\n几何对齐: {"✅" if geo_ok else "❌"} '
          f'{out.info.width}x{out.info.height} @ {out.info.resolution}')
    print(f'输出数据长度: {len(out.data)}（期望 {W*H}）')

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0 if (ok and geo_ok) else 1


if __name__ == '__main__':
    sys.exit(main())
