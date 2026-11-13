#!/usr/bin/env python3
"""把一次运行的 launch.log 按“层”切成独立文件。

为什么需要：`launch.log` 是**所有节点混排**的完整时间线，追因果最方便，
但排错时常常只想看一层（例如只看 K-star 为什么失败），混排噪声太大。
`roslog/` 里是 ROS **按节点**分的文件，和“按层”不是一回事
（一层可能由多个节点共同产生，一个节点也可能横跨多层）。

本脚本在 run_sim.sh 退出时自动执行，产出：

    <run_dir>/by_layer/
        00_感知.log        地面分割 / 裁剪 / mask 桥 / 点云
        10_全局.log        骨架、A*、MinSnap、走廊、恢复 FSM、goal checker
        20_局部.log        控制器跟踪、K-star、fallback
        30_执行.log        /cmd_vel 链路、底盘实测
        40_TF.log          zlio 适配器、静态 TF、fake link
        50_状态.log        四层状态汇总、lifecycle guard
        60_nav2.log        nav2 自己（controller_server / planner_server / bt_navigator …）
        70_定位.log        zlio 前端/后端、imu_prep
        99_其它.log        未归类的行

用法：
    split_run_log.py <run_dir>          # 读 <run_dir>/launch.log
"""
import os
import re
import sys

# (目录名, 匹配的正则) —— 顺序即优先级，先匹配先归类
GROUPS = [
    ("00_感知", [
        r"\[ground_segmentation\]", r"\[terrain\]", r"\[cloud_crop\]",
        r"\[plan_mask\]", r"\[livox", r"patchwork", r"PatchWork",
    ]),
    ("10_全局", [
        r"\[tdt\]", r"\[plan_goal_checker\]", r"\[tdt\]\[恢复\]",
    ]),
    ("20_局部", [
        r"\[tdt-local\]", r"kastar", r"Kastar",
    ]),
    ("30_执行", [
        r"\[VEL\]", r"\[fake_vel\]", r"\[world→body\]", r"cmd_vel_chassis",
    ]),
    ("40_TF", [
        r"\[zlio_tf\]", r"static_transform_publisher", r"fake_vel_transform",
        r"Timed out waiting for transform", r"Lookup would require extrapolation",
    ]),
    ("50_状态", [
        r"\[nav_stage_monitor\]", r"\[nav_guard\]", r"nav_lifecycle_guard",
    ]),
    ("60_nav2", [
        r"controller_server", r"planner_server", r"bt_navigator",
        r"behavior_server", r"smoother_server", r"waypoint_follower",
        r"velocity_smoother", r"lifecycle_manager", r"global_costmap",
        r"local_costmap", r"map_server",
    ]),
    ("70_定位", [
        r"\[backend\]", r"backend_node", r"\[zlio_node\]", r"\[imu_prep\]",
        r"tryRelocalization", r"global correction",
    ]),
]


def classify(line: str) -> str:
    for name, pats in GROUPS:
        for p in pats:
            if re.search(p, line):
                return name
    return "99_其它"


def main() -> int:
    if len(sys.argv) < 2:
        print("用法: split_run_log.py <run_dir>", file=sys.stderr)
        return 2
    run_dir = sys.argv[1]
    src = os.path.join(run_dir, "launch.log")
    if not os.path.exists(src):
        print(f"[split] 没有 {src}，跳过", file=sys.stderr)
        return 0

    out_dir = os.path.join(run_dir, "by_layer")
    os.makedirs(out_dir, exist_ok=True)
    handles = {}
    counts = {}
    try:
        for name, _ in GROUPS + [("99_其它", None)]:
            handles[name] = open(os.path.join(out_dir, f"{name}.log"), "w",
                                 encoding="utf-8", errors="replace")
            counts[name] = 0
        with open(src, encoding="utf-8", errors="replace") as f:
            for line in f:
                g = classify(line)
                handles[g].write(line)
                counts[g] += 1
    finally:
        for h in handles.values():
            h.close()

    # ★★★ 2026-11-12：**独立的问题清单**（failures.log）。
    #   为什么要单独一份：by_layer 适合"按层看"，但排查时最想先看的是
    #   "这次跑到底出了哪些问题、各多少次"。这里按固定模式抽出来，一次看清。
    FAIL_PATTERNS = [
        ("MinSnap 失败",        r"MinimumSnap 失败"),
        ("简化路径穿墙",         r"简化路径穿墙"),
        ("安全抽稀后仍穿墙",     r"安全抽稀后仍穿静态墙"),
        ("A* 无解",             r"A\* 没找到路径"),
        ("K-star 失败",          r"kastar \*\*失败"),
        ("局部目标裁剪",         r"局部目标裁剪"),
        ("恢复触发",            r"\[tdt\]\[恢复\] 触发"),
        ("恢复 FAILED",         r"\[tdt\]\[恢复\].*FAILED"),
        ("nav2 规划失败",        r"failed to generate a valid path"),
        ("目标中止",            r"Aborting handle|Goal was canceled"),
        ("TF 超时",             r"Timed out waiting for transform|extrapolation"),
        # ★ 2026-11-12：新增的诊断项也进清单（否则只在 launch.log 里，容易漏看）
        ("失败现场快照",         r"失败现场已存"),
        ("逃逸:自车格占用",      r"自车所在格被判占用"),
        ("逃逸:16方向全堵",      r"16 个方向都 <"),
        ("mask:自车周边豁免",    r"自车周围豁免 [1-9]"),
    ]
    summary = []
    fail_path = os.path.join(run_dir, "failures.log")
    try:
        with open(fail_path, "w", encoding="utf-8", errors="replace") as fo:
            for name, pat in FAIL_PATTERNS:
                rx = re.compile(pat)
                hits = []
                with open(src, encoding="utf-8", errors="replace") as f:
                    for line in f:
                        if rx.search(line):
                            hits.append(line.rstrip())
                summary.append((name, len(hits)))
                if hits:
                    fo.write(f"===== {name}（{len(hits)} 次）=====\n")
                    for h in hits:
                        fo.write(h + "\n")
                    fo.write("\n")
        printed = [(n, c) for n, c in summary]
        print("[split] 问题清单 → failures.log")
        for n, c in printed:
            if c:
                print(f"[split]   {n:<18} {c:>5} 次")
    except OSError as e:
        print(f"[split] failures.log 写不了: {e}", file=sys.stderr)

    total = sum(counts.values())
    kept = [(k, v) for k, v in counts.items() if v]
    kept.sort()
    print(f"[split] {total} 行 → {out_dir}")
    for k, v in kept:
        print(f"[split]   {k:<10} {v:>7} 行")
    return 0


if __name__ == "__main__":
    sys.exit(main())
