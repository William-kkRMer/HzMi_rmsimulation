#!/usr/bin/env python3
"""分层导航状态监视器。

订阅 /nav/l{0..3}/status（rm_nav_interfaces/StageStatus），
定期打印一行摘要，并在某层 FAILED 时立刻告警。

用法（一般由 launch 传入 stages 参数）：
    ros2 run rm_tdt_planner nav_stage_monitor --ros-args -p stages:=all -p period:=2.0
    ros2 run rm_tdt_planner nav_stage_monitor --ros-args -p stages:=l0
"""
import os
import sys
import rclpy
from rclpy.node import Node

try:
    from rm_nav_interfaces.msg import StageStatus
except ImportError:                      # 典型原因：shell 在包构建前 source 过
    sys.stderr.write(
        "\n[nav_stage_monitor] 找不到 rm_nav_interfaces。\n"
        "  原因：当前 shell 的环境是在该包构建之前 source 的。\n"
        "  修法：source install/setup.bash 后重新 launch（或开新终端）。\n"
        "  验证：python3 -c 'from rm_nav_interfaces.msg import StageStatus'\n\n")
    raise SystemExit(2)

NAMES = {0: "L0感知", 1: "L1全局", 2: "L2局部", 3: "L3控制"}
STATE = {0: "UNKNOWN", 1: "OK", 2: "DEGRADED", 3: "FAILED"}


class StageMonitor(Node):
    def __init__(self):
        super().__init__("nav_stage_monitor")
        self.declare_parameter("stages", "all")
        self.declare_parameter("period", 2.0)
        # ★★★ 2026-11-12：**每 2 s 的分层汇总同时写 CSV**（结构化、可画图、可两次运行对比）
        #   优先级：参数 csv_path > 环境变量 NAV_LAYER_CSV > 不写。
        #   run_sim.sh 会导出 NAV_LAYER_CSV=<run_dir>/layers.csv。
        self.declare_parameter("csv_path", "")
        sel = str(self.get_parameter("stages").value).strip().lower()
        period = float(self.get_parameter("period").value)

        csv_path = str(self.get_parameter("csv_path").value).strip()
        if not csv_path:
            csv_path = os.environ.get("NAV_LAYER_CSV", "").strip()
        self._csv = None
        if csv_path:
            try:
                need_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
                self._csv = open(csv_path, "a", buffering=1)
                if need_header:
                    self._csv.write("t,L0_state,L1_state,L2_state,L3_state,"
                                    "L0_hit,L1_pts,L1_len_m,L2_ctrl_v,L3_cmd_v,L3_actual_v\n")
                print(f"[nav_stage_monitor] CSV 记录: {csv_path}", flush=True)
            except OSError as e:
                print(f"[nav_stage_monitor] CSV 打不开({e})，仅打日志", flush=True)
                self._csv = None

        self.want = {0, 1, 2, 3} if sel == "all" else {
            int(c) for c in sel.replace("l", "").replace(",", "") if c.isdigit()
        }
        if not self.want:
            self.want = {0, 1, 2, 3}

        self.last = {}
        for s in sorted(self.want):
            self.create_subscription(
                StageStatus, f"/nav/l{s}/status",
                lambda m, _s=s: self._on(m, _s), 10)

        self.get_logger().info(
            "监视层: " + ", ".join(NAMES[s] for s in sorted(self.want))
            + f"（周期 {period:.1f}s）")
        self.create_timer(period, self._report)
        self._bad = set()

    def _on(self, msg: StageStatus, stage: int):
        kv = dict(zip(msg.keys, msg.values))
        self.last[stage] = (msg.state, msg.detail, kv)
        if msg.state == 3 and stage not in self._bad:
            self._bad.add(stage)
            detail = " ".join(f"{k}={v:.3g}" for k, v in kv.items())
            self.get_logger().error(
                f"[{NAMES[stage]}] FAILED: {msg.detail} | {detail}")
        elif msg.state != 3:
            self._bad.discard(stage)

    def _write_csv(self):
        """把当前四层状态写成一行（缺层留空，不用 0 冒充）。"""
        if not self._csv:
            return
        try:
            import time as _t
            ts = f"{_t.time():.3f}"
            st = [""] * 4
            for i, layer in enumerate(sorted(self.want)):
                if layer in self.last:
                    st[layer] = str(self.last[layer][0])
            def kv(layer, key):
                if layer in self.last:
                    v = self.last[layer][2].get(key)
                    return f"{v:.3g}" if isinstance(v, (int, float)) else ""
                return ""
            row = ",".join([ts] + st +
                           [kv(0, "mask_hit"), kv(1, "points"), kv(1, "len_m"),
                            kv(2, "ctrl_v"), kv(3, "cmd_v"), kv(3, "actual_v")])
            self._csv.write(row + "\n")
        except OSError:
            self._csv = None

    def _report(self):
        self._write_csv()
        parts = []
        for s in sorted(self.want):
            if s not in self.last:
                parts.append(f"{NAMES[s]}:无数据")
                continue
            state, detail, kv = self.last[s]
            brief = ",".join(f"{k}={v:.3g}" for k, v in list(kv.items())[:3])
            parts.append(f"{NAMES[s]}:{STATE.get(state, '?')}({brief})")
        self.get_logger().info(" | ".join(parts))


def main():
    rclpy.init()
    node = StageMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
