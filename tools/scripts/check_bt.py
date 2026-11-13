#!/usr/bin/env python3
"""行为树静态检查：把"跑起来才报错"的结构问题挡在启动之前。

为什么需要（2026-11-12 实测事故）：
    我们把行为树换成了带 `RecoveryNode` 的"纯重试"版本，但 nav2 的
    `RecoveryNode` **必须正好 2 个子节点**（动作 + 恢复动作）。只给 1 个时
    bt_navigator 抛异常：
        Behavior tree threw exception: Recovery Node 'ComputePathToPose'
        must only have 2 children.. Exiting with failure.
    ⇒ 整棵树加载失败 ⇒ **规划不跑、指令不下发**，而日志里只有这一行 ERROR，
      极容易被当成"控制器坏了"。

本脚本检查的规则（都是实测踩过的）：
    1. XML 可解析，根节点是 <root> 且带 main_tree_to_execute
    2. 主树 ID 与 main_tree_to_execute 一致
    3. `RecoveryNode` 必须正好 2 个子节点（nav2 硬约束）
    4. `RetryUntilSuccessful` 必须正好 1 个子节点（BT.CPP 装饰器约束）
    5. 不允许出现需要 costmap 的恢复动作（Spin/BackUp/Wait/ClearEntireCostmap）
       ——本项目已决定"恢复交给 T-DT 自己的 FSM"，这里只提示不算失败
    6. 叶子节点（动作）必须有非空属性集合（至少 goal/path/controller_id 之一）

用法：
    check_bt.py <file.xml> [--allow-recovery]
    check_bt.py            # 自动检查 nav2_params_sim_base.yaml 里引用的那棵树
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

# 需要 costmap 才能工作的恢复动作
COSTMAP_NODES = {"Spin", "BackUp", "Wait", "ClearEntireCostmap"}
# nav2 要求正好 2 个子节点的节点
EXACTLY_TWO = {"RecoveryNode"}
# 装饰器必须正好 1 个子节点
EXACTLY_ONE = {"RetryUntilSuccessful", "RateController", "Inverter",
               "ForceSuccess", "ForceFailure", "KeepRunningUntilFailure",
               "DistanceController", "SpeedController",
               "SingleTrigger", "Repeat", "Timeout"}


def referenced_tree() -> str:
    """从 nav2 参数里找当前生效的行为树文件。"""
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    yml = os.path.join(root, "src", "rm_navigation", "rm_navigation", "params",
                       "nav2_params_sim_base.yaml")
    if not os.path.exists(yml):
        return ""
    txt = open(yml, encoding="utf-8").read()
    m = re.search(r"default_nav_to_pose_bt_xml:\s*(\S+)", txt)
    return m.group(1) if m else ""


def check(path: str, allow_recovery: bool) -> int:
    errs, warns = [], []
    try:
        tree = ET.parse(path)
    except ET.ParseError as e:
        print(f"BAD  XML 解析失败: {e}")
        return 1
    root = tree.getroot()
    if root.tag != "root":
        errs.append(f"根节点应为 <root>，实际 <{root.tag}>")
    main = root.get("main_tree_to_execute")
    if not main:
        errs.append("<root> 缺少 main_tree_to_execute")

    trees = {t.get("ID"): t for t in root.findall("BehaviorTree")}
    if main and main not in trees:
        errs.append(f"main_tree_to_execute='{main}' 但树里没有同名 <BehaviorTree>")
    target = trees.get(main) or (next(iter(trees.values())) if trees else root)

    for e in target.iter():
        if e is target:
            continue
        n = len(list(e))
        if e.tag in EXACTLY_TWO and n != 2:
            errs.append(f"<{e.tag} name='{e.get('name', '')}'> 必须正好 2 个子节点，实际 {n}"
                        "（nav2 会在加载时抛异常 ⇒ 不规划、不下指令）")
        if e.tag in EXACTLY_ONE and n != 1:
            errs.append(f"<{e.tag}> 必须正好 1 个子节点，实际 {n}")
        if e.tag in COSTMAP_NODES:
            (warns if allow_recovery else errs).append(
                f"<{e.tag}> 需要 costmap（本项目已决定不用恢复动作）")
        if n == 0 and not list(e.attrib.keys()):
            warns.append(f"<{e.tag}> 是叶子节点但没有任何属性（可能漏了 goal/path 等端口）")

    for m in errs:
        print(f"BAD  {m}")
    for m in warns:
        print(f"WARN {m}")
    if errs:
        return 1
    print(f"OK   {os.path.basename(path)}（树 ID={main}，节点 {sum(1 for _ in target.iter()) - 1} 个）")
    return 0


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    allow = "--allow-recovery" in sys.argv
    path = args[0] if args else referenced_tree()
    if not path:
        print("没有参数，也没能从 nav2 参数里找到行为树路径")
        return 2
    if not os.path.exists(path):
        print(f"BAD  文件不存在: {path}")
        return 1
    return check(path, allow)


if __name__ == "__main__":
    sys.exit(main())
