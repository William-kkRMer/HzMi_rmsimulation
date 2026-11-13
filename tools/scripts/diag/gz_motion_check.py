#!/usr/bin/env python3
"""gz_motion_check.py —— 判"Gazebo 里的车到底动没动"，不看画面看数据。

用 `ign topic` 采 `/world/<w>/dynamic_pose/info`，**两次**比较 chassis 的世界位姿：
  · 第一次：静止
  · 然后自己发 1 秒 /cmd_vel_chassis（0.3 m/s 前进），再采一次
判读：
  dynamic_pose 里**找不到 chassis** ⇒ 模型是 `<static>true</static>`（永远不动，GUI 也不会动）
  找到了但两次位姿**不同**       ⇒ 正常（模型确实在动）
  找到了但**完全相同**           ⇒ 动态模型但插件没写位姿（看 /cmd_vel_chassis 有没有数据）

用法：
    python3 tools/scripts/diag/gz_motion_check.py                 # 自动找世界名与分区
    python3 tools/scripts/diag/gz_motion_check.py --model chassis
    python3 tools/scripts/diag/gz_motion_check.py --partition rm_gz_xxx --world rmuc2027_gz
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import subprocess
import sys
import time


def sh(cmd: list[str], env: dict | None = None, timeout: float = 12.0) -> str:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env)
        return (r.stdout or '') + (r.stderr or '')
    except subprocess.TimeoutExpired as e:
        return ((e.stdout or '') if isinstance(e.stdout, str) else '') + \
               ((e.stderr or '') if isinstance(e.stderr, str) else '')
    except FileNotFoundError:
        return ''


def detect_partition() -> str:
    """从最新的 launch 日志里抓 `gz transport partition = xxx`。

    同时看 `$ROS_LOG_DIR`（沙箱/自定义日志目录里不会有 ~/.ros/log）与 `~/.ros/log`。
    """
    pats = []
    d = os.environ.get('ROS_LOG_DIR', '')
    if d:
        pats.append(os.path.join(os.path.expanduser(d), '*', 'launch.log'))
        pats.append(os.path.join(os.path.expanduser(d), 'launch.log'))
    pats.append(os.path.expanduser('~/.ros/log/*/launch.log'))
    logs = []
    for pat in pats:
        logs.extend(glob.glob(pat))
    logs = sorted(logs, key=os.path.getmtime)
    for p in reversed(logs[-5:]):
        try:
            txt = open(p, encoding='utf-8', errors='ignore').read()
        except OSError:
            continue
        m = re.search(r'gz transport partition = (\S+)', txt)
        if m:
            return m.group(1)
    return ''


def partition_from_process() -> str:
    """从正在跑的 `ign gazebo` 进程环境里读 IGN_PARTITION —— 最可靠。

    launch 只把 partition 打在**自己的 stdout**（不写进 ~/.ros/log/<ts>/launch.log），
    所以按日志找经常找不到；`/proc/<pid>/environ` 一定带着 GZ_PARTITION/IGN_PARTITION。
    """
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            cmd = open(f'/proc/{pid}/cmdline', 'rb').read().replace(b'\0', b' ').decode('utf-8', 'ignore')
        except OSError:
            continue
        if 'gazebo' not in cmd or 'ign' not in cmd:
            continue
        try:
            envb = open(f'/proc/{pid}/environ', 'rb').read().split(b'\0')
        except OSError:
            continue
        for kv in envb:
            k, _, v = kv.decode('utf-8', 'ignore').partition('=')
            if k in ('IGN_PARTITION', 'GZ_PARTITION') and v:
                return v
    return ''


def detect_world(env: dict) -> str:
    out = sh(['ign', 'topic', '-l'], env=env, timeout=10)
    for line in out.splitlines():
        line = line.strip()
        if line.startswith('/world/') and line.endswith('/dynamic_pose/info'):
            return line.split('/')[2]
    return ''


def sample_pose(env: dict, world: str, model: str) -> dict | None:
    """返回 {'x','y','yaw'} 或 None（话题里没有这个 model ⇒ 它是 static）。"""
    out = sh(['ign', 'topic', '-e', '-n', '1', '-t',
              f'/world/{world}/dynamic_pose/info'], env=env, timeout=12)
    if not out.strip():
        return None
    # 按 entity 块切
    blocks = re.split(r'\n\s*name: "', out)
    for b in blocks[1:]:
        name = b.split('"', 1)[0]
        if name != model:
            continue
        pos = re.search(r'position\s*\{([^}]*)\}', b, re.S)
        ori = re.search(r'orientation\s*\{([^}]*)\}', b, re.S)

        def num(block: str, key: str) -> float:
            m = re.search(key + r'\s*:\s*(-?[\d.eE+]+)', block) if block else None
            return float(m.group(1)) if m else float('nan')

        p = pos.group(1) if pos else ''
        o = ori.group(1) if ori else ''
        return {'x': num(p, 'x'), 'y': num(p, 'y'), 'z': num(p, 'z'),
                'yaw': num(o, 'z')}
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=os.environ.get('GZ_MODEL', 'chassis'))
    ap.add_argument('--partition', default='')
    ap.add_argument('--world', default='')
    ap.add_argument('--speed', type=float, default=0.3, help='探测用的前进速度 m/s')
    ap.add_argument('--topic', default='/cmd_vel_chassis',
                    help='发速度的话题。默认直发底盘；用 /cmd_vel 可以连带验证 '
                         'fake_vel_transform + teleop 链路。')
    ap.add_argument('--seconds', type=float, default=1.0, help='发多久（默认 1 s）')
    args = ap.parse_args()

    env = dict(os.environ)
    part = args.partition or partition_from_process() or detect_partition()
    if part:
        env['IGN_PARTITION'] = part
        env['GZ_PARTITION'] = part
        print(f'partition        = {part}')
    else:
        print('partition        = (未找到，沿用当前环境)')

    world = args.world or detect_world(env)
    if not world:
        print('✗ 找不到世界名：gz 没在跑？（`ign topic -l | grep dynamic_pose` 应有 /world/<w>/dynamic_pose/info）')
        return 2
    print(f'world            = {world}')
    print(f'model            = {args.model}')

    a = sample_pose(env, world, args.model)
    if a is None:
        print('\n结论：**模型不在 dynamic_pose/info 里** ⇒ 它是 `<static>true</static>`。')
        print('      static 模型的位姿**永远不会变**（不是"看起来不动"，是绝对不动，GUI 也拿不到更新）。')
        print('      修法：`GZ_DRIVE_MOTION=visual`（默认值）重跑，并确认 launch 日志里')
        print('            `驱动模型 motion = visual`。')
        return 1

    print(f'静止位姿         = x={a["x"]:.3f} y={a["y"]:.3f} z={a["z"]:.3f}')

    # 自己发 1 秒速度，绕开 teleop / fake_vel_transform
    n = max(1, int(round(args.seconds * 10)))
    sh(['ros2', 'topic', 'pub', '-r', '10', '--times', str(n),
        args.topic, 'geometry_msgs/msg/Twist',
        '{linear: {x: %f}}' % args.speed], timeout=args.seconds + 6)
    time.sleep(0.4)
    b = sample_pose(env, world, args.model)
    if b is None:
        print('\n结论：第二次采样失败（话题没数据了？）')
        return 2
    print(f'运动后位姿       = x={b["x"]:.3f} y={b["y"]:.3f} z={b["z"]:.3f}')

    d = ((b['x'] - a['x']) ** 2 + (b['y'] - a['y']) ** 2) ** 0.5
    expect = args.speed * args.seconds
    print(f'位移             = {d:.3f} m（发了 {args.speed} m/s × {args.seconds} s，期望≈{expect:.2f} m；话题 {args.topic}）')
    if d > 0.02:
        print('\n结论：**正常** —— 模型确实在动（GUI 里看得到，位姿也真的更新了）。')
        return 0
    print('\n结论：**动态模型但位姿没变** ⇒ 插件没把位姿写进去。')
    print('      查：`ros2 topic hz /cmd_vel_chassis`（应有 ~10 Hz）、')
    print('          `ros2 param get /fake_vel_transform spin_speed`（应为 0.0）。')
    return 1


if __name__ == '__main__':
    sys.exit(main())
