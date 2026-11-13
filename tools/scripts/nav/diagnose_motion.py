#!/usr/bin/env python3
"""诊断「车走的路和规划不一致」到底是哪一环错了。

同时看四个量，全部换算到 **map 世界系** 比较方向：
  1. 期望方向   = 当前位置 → /plan 上"前方 1.5 m 处的点"
  2. 指令方向   = /cmd_vel（车体系）经当前 yaw 旋转到世界系
  3. 实际方向   = 位姿差分得到的世界系速度
  4. yaw 对比   = base_link_fake 与 base_link 的朝向差（fake_vel_transform 的偏置）

判读：
  · 指令方向 ≈ 实际方向，但都 ≠ 期望  ⇒ 规划/参考点问题
  · 指令 ≈ 期望，实际 ≠              ⇒ 底盘/fake_vel_transform/轮子方向
  · 指令 与 期望 差常数角度           ⇒ 坐标系旋转问题（双重旋转）
  · 指令 与 期望 差 180°              ⇒ 符号问题

用法：
  python3 tools/scripts/nav/diagnose_motion.py --seconds 30
  （启动后请在 RViz 里发一个目标）
"""
import argparse, math, csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import tf2_ros

def wrap(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def deg(a):
    return math.degrees(wrap(a))

class Diag(Node):
    def __init__(self, odom_topic, gt_topic, lookahead, seconds):
        super().__init__("diagnose_motion")
        self.plan = None
        self.odom = None
        self.cmd = None
        self.gt = None
        self.prev_gt = None
        self.gt_dir = None
        self.prev_xy = None
        self.actual_dir = None
        self.rows = []
        self.lookahead = lookahead
        self.seconds = seconds
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Path, "/plan", self.on_plan, qos)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 20)
        if gt_topic:
            self.create_subscription(Odometry, gt_topic, self.on_gt, 20)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.create_timer(0.2, self.tick)
        self.t_end = self.get_clock().now().nanoseconds * 1e-9 + seconds

    def on_plan(self, m): self.plan = m
    def on_odom(self, m): self.odom = m
    def on_cmd(self, m): self.cmd = m
    def on_gt(self, m): self.gt = m

    def yaw_of(self, parent, child):
        try:
            t = self.tfb.lookup_transform(parent, child, rclpy.time.Time())
            q = t.transform.rotation
            return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        except Exception:
            return None

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now > self.t_end:
            raise SystemExit(0)
        if self.odom is None or self.plan is None or not self.plan.poses:
            return
        p = self.odom.pose.pose.position
        yaw = self.yaw_of("map", "base_link")
        yaw_fake = self.yaw_of("map", "base_link_fake")

        # 实际方向（位姿差分，世界系）
        if self.prev_xy is not None:
            dx, dy = p.x - self.prev_xy[0], p.y - self.prev_xy[1]
            if math.hypot(dx, dy) > 0.01:
                self.actual_dir = math.atan2(dy, dx)
        self.prev_xy = (p.x, p.y)

        # Gazebo 真值方向（判断"仿真里到底往哪走了"）
        if self.gt is not None:
            g = self.gt.pose.pose.position
            if self.prev_gt is not None:
                gx, gy = g.x - self.prev_gt[0], g.y - self.prev_gt[1]
                if math.hypot(gx, gy) > 0.01:
                    self.gt_dir = math.atan2(gy, gx)
            self.prev_gt = (g.x, g.y)

        # 期望方向：plan 上离车 1.5 m 的点
        want = None
        for q in self.plan.poses:
            qp = q.pose.position
            if math.hypot(qp.x - p.x, qp.y - p.y) >= self.lookahead:
                want = qp; break
        if want is None:
            want = self.plan.poses[-1].pose.position
        want_dir = math.atan2(want.y - p.y, want.x - p.x)

        # 指令方向：/cmd_vel 是车体系 ⇒ 用 base_link 的 yaw 转到世界
        cmd_dir = None
        if self.cmd is not None and yaw is not None:
            vx, vy = self.cmd.linear.x, self.cmd.linear.y
            if math.hypot(vx, vy) > 0.02:
                cmd_dir = yaw + math.atan2(vy, vx)

        row = {
            "t": round(now, 2),
            "x": round(p.x, 3), "y": round(p.y, 3),
            "yaw": round(deg(yaw), 1) if yaw is not None else "",
            "yaw_fake": round(deg(yaw_fake), 1) if yaw_fake is not None else "",
            "fake_minus_real": round(deg(yaw_fake - yaw), 1) if (yaw is not None and yaw_fake is not None) else "",
            "want_deg": round(deg(want_dir), 1),
            "cmd_deg": round(deg(cmd_dir), 1) if cmd_dir is not None else "",
            "actual_deg": round(deg(self.actual_dir), 1) if self.actual_dir is not None else "",
            "cmd_minus_want": round(deg(cmd_dir - want_dir), 1) if cmd_dir is not None else "",
            "actual_minus_want": round(deg(self.actual_dir - want_dir), 1) if self.actual_dir is not None else "",
            "gt_deg": round(deg(self.gt_dir), 1) if self.gt_dir is not None else "",
            "gt_minus_act": round(deg(self.gt_dir - self.actual_dir), 1)
                            if (self.gt_dir is not None and self.actual_dir is not None) else "",
            "v": round(math.hypot(self.cmd.linear.x, self.cmd.linear.y), 2) if self.cmd else "",
        }
        self.rows.append(row)
        print(f"want={row['want_deg']:>6}° cmd={row['cmd_deg']:>6}° act={row['actual_deg']:>6}° "
              f"| cmd-want={row['cmd_minus_want']:>6}° act-want={row['actual_minus_want']:>6}° "
              f"| yaw={row['yaw']}° fake-yaw={row['fake_minus_real']}° "
              f"gt={row['gt_deg']}° gt-act={row['gt_minus_act']}° v={row['v']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--odom", default="/localization/odometry")
    ap.add_argument("--gt", default="/odom_ground_truth", help="仿真真值里程计，没有就传空")
    ap.add_argument("--lookahead", type=float, default=1.5)
    ap.add_argument("--csv", default="/tmp/motion_diag.csv")
    a = ap.parse_args()
    rclpy.init()
    n = Diag(a.odom, a.gt, a.lookahead, a.seconds)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    if n.rows:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(n.rows[0].keys()))
            w.writeheader(); w.writerows(n.rows)
        def avg(k):
            v = [r[k] for r in n.rows if r[k] != ""]
            return sum(v)/len(v) if v else float("nan")
        print("\n== 汇总 ==")
        print(f"样本 {len(n.rows)} 行, CSV: {a.csv}")
        print(f"cmd - want    平均 {avg('cmd_minus_want'):+.1f}°")
        print(f"actual - want 平均 {avg('actual_minus_want'):+.1f}°")
        print(f"gt     - act  平均 {avg('gt_minus_act'):+.1f}°  ← ≈0 说明定位与仿真一致")
        fmr = [r['fake_minus_real'] for r in n.rows if r['fake_minus_real'] != ""]
        if fmr:
            print(f"fake_yaw - yaw 平均 {sum(fmr)/len(fmr):+.1f}°  (范围 {min(fmr):+.1f} ~ {max(fmr):+.1f})")
        print("\n判读：")
        print(" · cmd-want 接近常数且 |值| 大  ⇒ 坐标系/旋转问题")
        print(" · cmd-want≈0 但 actual-want 大 ⇒ 底盘或 fake_vel_transform")
        print(" · 两者都大 ⇒ 规划参考点本身就不在车上")
        print(" · gt-act 大   ⇒ Gazebo 真值与定位不一致（底盘/建模/命令链路）")
        print(" · gt-act≈0 且 act≈want ⇒ 实际是对的；Gazebo 里看着怪 = 万向底盘横移/模型朝向")
    else:
        print("没有采到数据（/plan 或 /odom 为空？先发目标再采）")


if __name__ == "__main__":
    main()
