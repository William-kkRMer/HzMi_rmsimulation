#!/usr/bin/env python3
"""把 nav2 costmap（= STVL 的 2D 投影结果）二值化，转成 T-DT 消费的 `/plan_mask`。

## 为什么需要它

T-DT 的规划器与控制器只吃两样东西（`planner.cpp:122-126`、`controller.cpp:81-88`）：

    "/map"        —— 先验静态图（transient_local）
    "/plan_mask"  —— 动态障碍（二值：>50 即障碍）

两者**不读 costmap 的代价**。而 STVL 是 costmap 插件，它的成果只写进
`Costmap2D` 的代价格 ⇒ 不经过本节点，STVL 对 T-DT 就**完全不生效**
（这正是切到 T-DT 之后 STVL "变摆设"的原因）。

## 关键：必须二值化，不能照抄代价

costmap 的 0–255 里：

    >= 253   INSCRIBED(253) / LETHAL(254) / 未知致命        ⇒ 障碍
     1..252  膨胀层软带（由 inflation_radius 按 exp 衰减生成）⇒ **不是障碍**
        0    自由
       -1    未知（本栈 `track_unknown_space: true`，可能大量出现）⇒ **不是障碍**

若把 1..252 也当障碍，T-DT 的 `dilate_cells` 会**再叠一次**余量，
窄道会被挤没 —— 与仓库里 `field.inflation_radius: 0.33` 那个坑同源。
安全余量统一交给 T-DT 自己的 `dilate_cells`（1 格 = 0.05 m）负责。

## 几何对齐

mask 的尺寸/原点/分辨率必须与 `/map` **完全一致**，否则 T-DT 直接丢弃
（`controller.cpp:534-537` 有严格校验）。因此本节点先订阅 `/map` 取基准几何，
只在与 costmap 完全吻合时才发 mask；不吻合就丢弃并限频告警。
"""
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


class CostmapToPlanMask(Node):
    def _dump_mask(self, m):
        """把这一帧 mask 存成 pgm（0=障碍 205=未知 254=可走），供离线叠加分析。"""
        import os
        import numpy as np
        from PIL import Image
        d = os.path.join(os.path.expanduser('~'), '.ros', 'rm_tdt_dump')   # 不再写死仓库路径
        os.makedirs(d, exist_ok=True)
        w, h = m.info.width, m.info.height
        a = np.array(m.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 254, np.uint8)
        img[a < 0] = 205
        img[a >= 50] = 0
        stamp = str(int(m.header.stamp.sec)) if m.header.stamp.sec else str(int(__import__('time').time()))
        p = f'{d}/mask_{stamp}.pgm'
        with open(p, 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (w, h))
            f.write(img[::-1].tobytes())        # 行 0 在上，与 pgm 约定一致
        with open(p.rsplit('.', 1)[0] + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(p)}\nmode: trinary\nresolution: {m.info.resolution}\n'
                    f'origin: [{m.info.origin.position.x}, {m.info.origin.position.y}, 0]\n'
                    f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        # 只留最近 3 张
        files = sorted(f for f in os.listdir(d) if f.startswith('mask_') and f.endswith('.pgm'))
        for old in files[:-3]:
            for ext in ('.pgm', '.yaml'):
                try:
                    os.remove(os.path.join(d, old.rsplit(".", 1)[0] + ext))
                except OSError:
                    pass
        self.get_logger().info(f'[plan_mask] 落盘 {p}（障碍 {int((img == 0).sum())} 格）')

    def __init__(self):
        super().__init__('plan_mask_from_costmap')

        self.declare_parameter('input_topic', '/global_costmap/costmap')
        self.declare_parameter('output_topic', '/plan_mask')
        self.declare_parameter('map_topic', '/map')
        # >= 该代价视为障碍。默认 253（内切/致命），**刻意排除 1..252 的软带**。
        self.declare_parameter('lethal_threshold', 99)
        # ★★★ 2026-11-12（晚，**实测确认**）：是否把「内切膨胀」(值 99, 原始 cost 253) 也算障碍。
        #   默认 **False = 只认障碍本体(100)**。
        #   为什么：99 对应 `distance <= inscribed_radius`(= robot_radius 0.22 m)，
        #   那是给 **nav2 自己的控制器**留的余量；而 T-DT 不读 costmap，
        #   它自己有 `dilate_cells`(0.10) + `corridor_shrink`(0.15) = 0.25 m。
        #   两套叠加 ⇒ 每侧 0.22+0.10+0.15 = **0.47 m**，而 0.7 m 通道每侧只有 0.35 m
        #   ⇒ **窄通道被直接判死**（用户实测：改 99/100 后通道确实被堵死）。
        #   打开（true）只在"想更保守/调试为什么进不去"时用，会重新堵窄道。
        self.declare_parameter('include_inscribed', False)
        self.declare_parameter('obstacle_value', 100)
        self.declare_parameter('publish_on_unchanged', False)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        map_topic = self.get_parameter('map_topic').value
        self.lethal = int(self.get_parameter('lethal_threshold').value)
        self.include_inscribed = bool(self.get_parameter('include_inscribed').value)
        self.obstacle_value = int(self.get_parameter('obstacle_value').value)
        self.publish_unchanged = bool(self.get_parameter('publish_on_unchanged').value)
        # mask 落盘周期（秒）；0 = 关闭。用于确认"实时层把哪些地方当成了障碍"。
        self.declare_parameter('dump_every_s', 30.0)

        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        volatile = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST)

        self.map_info = None
        self.pub = self.create_publisher(OccupancyGrid, out_topic, volatile)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, latched)
        self.create_subscription(OccupancyGrid, in_topic, self.on_costmap, volatile)

        self.mismatch_count = 0
        self.published = 0
        self.last_key = None
        self.obstacle_cells = 0
        self.soft_only_cells = 0
        self.unknown_cells = 0
        self.get_logger().info(
            f'[plan_mask] {in_topic} → {out_topic}'
            f'（>= {self.lethal} 记为障碍；1..{self.lethal - 1} 的膨胀软带被丢弃）')

    # ---- 基准几何：只认 /map ----
    def on_map(self, msg: OccupancyGrid):
        self.map_info = (msg.info.resolution,
                         msg.info.origin.position.x,
                         msg.info.origin.position.y,
                         msg.info.width,
                         msg.info.height)
        self.get_logger().info(
            f'[plan_mask] 基准几何: {msg.info.width}x{msg.info.height} @ '
            f'{msg.info.resolution:.3f} origin=({msg.info.origin.position.x:.2f},'
            f'{msg.info.origin.position.y:.2f})')

    def _geometry_ok(self, msg: OccupancyGrid) -> bool:
        if self.map_info is None:
            return False
        res, ox, oy, w, h = self.map_info
        i = msg.info
        return (abs(i.resolution - res) < 1e-9
                and abs(i.origin.position.x - ox) < 1e-6
                and abs(i.origin.position.y - oy) < 1e-6
                and i.width == w and i.height == h)

    def on_costmap(self, msg: OccupancyGrid):
        if not self._geometry_ok(msg):
            self.mismatch_count += 1
            # 限频：每 200 次告警一次，避免淹没日志
            if self.mismatch_count == 1 or self.mismatch_count % 200 == 0:
                self.get_logger().warn(
                    f'[plan_mask] 几何与 /map 不一致，丢弃（第 {self.mismatch_count} 次）: '
                    f'{msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} '
                    f'origin=({msg.info.origin.position.x:.2f},{msg.info.origin.position.y:.2f}) '
                    f'期望 {self.map_info}')
            return

        data = msg.data
        out = bytearray(len(data))
        hard = 0
        soft = 0
        unknown = 0
        inscribed = 0
        for i, c in enumerate(data):
            # ★★★ 2026-11-12（**源码级修正**）：nav2 发布的 costmap **不是代价原值**，
            #   而是经过 `cost_translation_table_` 映射后的**占用值 0..100**。
            #   证据：third_party/nav2/nav2_costmap_2d/src/costmap_2d_publisher.cpp:89-100
            #       [0]   = 0    无
            #       [253] = 99   INSCRIBED（内切）
            #       [254] = 100  LETHAL（致命）
            #       [255] = -1   UNKNOWN
            #       1..252 → 1..98（膨胀软带）
            #   ⇒ 旧代码判 `253 <= u <= 254` **永远不成立** ⇒ mask 一直是空的！
            #   实测后果：planner `命中 0 格` 15/15、`L0感知(mask_hit=0)` 243 次、
            #   桥的"已发 40 帧"日志一次都没出现（内容不变 ⇒ 只发过 1 帧）。
            #   ⇒ **T-DT 的动态避障此前完全失效，只靠静态图**。
            #   现在按 99/100 判；255(未知) 仍必须排除。
            u = c & 0xFF
            # 只认障碍本体(100)；内切(99) 默认**丢弃**（见 include_inscribed 的说明）
            if u == 100 or (self.include_inscribed and u == 99):
                out[i] = self.obstacle_value
                hard += 1
            elif u == 255:
                unknown += 1       # 未知（-1 / 255）：**不是障碍**
            elif u == 99:
                inscribed += 1     # 内切(原 253)：默认不计入，见 include_inscribed
            elif u >= 1:
                soft += 1          # 膨胀软带 1..252：**刻意不计入障碍**
        self.obstacle_cells = hard
        self.inscribed_cells = inscribed
        self.soft_only_cells = soft
        self.unknown_cells = unknown

        key = bytes(out)
        if not self.publish_unchanged and key == self.last_key:
            return
        self.last_key = key

        m = OccupancyGrid()
        m.header = msg.header
        m.header.frame_id = msg.header.frame_id or 'map'
        m.info = msg.info
        m.data = list(out)
        # ★★★ 2026-11-13（**确认"实时把坡当障碍"用**）：把 mask 落盘，每 30 s 一张，留最近 3 张。
        #   为什么要：日志只能告诉我们"新增障碍 1.5 万格"，**看不出长在哪**。
        #   落盘后可以直接和静态图叠加，一眼看出是斜坡、地面还是别的。
        #   关掉：参数 dump_every_s:=0
        try:
            every = float(self.get_parameter('dump_every_s').value)
        except Exception:
            every = 30.0
        if every > 0.0:
            import time as _t
            now_s = _t.time()
            if now_s - getattr(self, '_last_dump', 0.0) > every:
                self._last_dump = now_s
                try:
                    self._dump_mask(m)
                except Exception as exc:      # 落盘失败不能影响主链路
                    self.get_logger().warn(f'[plan_mask] 落盘失败: {exc}')
        self.pub.publish(m)

        self.published += 1
        if self.published % 40 == 0:
            self.get_logger().info(
                f'[plan_mask] 已发 {self.published} 帧 | 障碍 {hard} 格 '
                f'| 内切(已忽略) {inscribed} 格 | 仅软带(已忽略) {soft} 格 '
                f'| 未知 {unknown} 格')


def main():
    rclpy.init()
    node = CostmapToPlanMask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 双重 shutdown 会抛 RCLError（本仓库其它脚本踩过），这里用 try 兜住
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
