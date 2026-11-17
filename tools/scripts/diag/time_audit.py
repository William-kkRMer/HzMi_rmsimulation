#!/usr/bin/env python3
"""time_audit.py —— 一次跑完/一个包，检查"时间戳是否自洽"。

回答三个问题：
  ① 点云 timebase/offset 与 IMU 时间是否同一时间轴（每帧窗口里 IMU 数应 ≈ 100Hz×帧长）
  ② TF 各条边的戳是否与传感器同一时间轴（有没有被加前瞻/滞后）
  ③ 包里有没有 /clock（决定 `ros2 bag play --clock` 能不能让 use_sim_time 节点工作）

用法：
  python3 tools/scripts/diag/time_audit.py .tmp_diag/run/<ts>/bag
"""
import sys, collections
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock

def resolve(path):
    """bag 目录 / .db3 都能用；录制中的包没有 metadata.yaml，直接读 db3。"""
    import os, glob
    if os.path.isfile(path):
        return path, ''
    meta = os.path.join(path, 'metadata.yaml')
    if os.path.isfile(meta):
        return path, ''
    for cand in (path, os.path.join(path, 'bag')):
        dbs = sorted(glob.glob(os.path.join(cand, '*.db3')), key=os.path.getmtime)
        if dbs:
            return dbs[-1], '（没有 metadata.yaml：包可能还在录，或没收尾干净 ⇒ 直接读 db3）'
    return path, '（目录里既没有 metadata.yaml 也没有 .db3）'


def main(path):
    path, note = resolve(path)
    if note:
        print(f'   {note}')
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id='sqlite3'), ConverterOptions('', ''))
    cloud, imu, odom = [], [], []
    tf = collections.defaultdict(list)
    has_clock = 0
    while r.has_next():
        t, d, rt = r.read_next()
        if t == '/livox/lidar':
            m = deserialize_message(d, CustomMsg)
            off = max([p.offset_time for p in m.points]) if m.points else 0
            pts = m.points
            near5 = near25 = 0
            for q in pts:
                rr = (q.x * q.x + q.y * q.y + q.z * q.z) ** 0.5
                if rr < 0.5:
                    near5 += 1
                if rr < 0.25:
                    near25 += 1
            cloud.append((m.timebase * 1e-9, (m.timebase + off) * 1e-9, len(pts),
                          near5 / max(1, len(pts)), near25 / max(1, len(pts))))
        elif t == '/livox/imu':
            m = deserialize_message(d, Imu)
            imu.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        elif t == '/localization/odometry':
            m = deserialize_message(d, Odometry)
            odom.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        elif t == '/tf':
            m = deserialize_message(d, TFMessage)
            for tr in m.transforms:
                tf[f'{tr.header.frame_id}->{tr.child_frame_id}'].append(
                    tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)
        elif t == '/clock':
            has_clock += 1

    print(f'== {path}')
    print(f'   /clock 消息: {has_clock} {"（✓ 可用 --clock 回放）" if has_clock else "（✗ 回放时 use_sim_time 节点会拿不到时间）"}')
    if cloud:
        c = np.array(cloud)
        print(f'   点云 {len(c)} 帧 | timebase {c[0,0]:.3f} → {c[-1,0]:.3f} s | 每帧跨 {1000*np.median(c[:,1]-c[:,0]):.1f} ms | 点数中位 {int(np.median(c[:,2]))}')
        print(f'   自击比例: <0.5 m {100*np.median(c[:,3]):.1f}%  <0.25 m {100*np.median(c[:,4]):.1f}%'
              f'  （抬高雷达前实测 34.7% / 31.7%；越低越好，zlio minimum_range=0.5 m 会丢弃 <0.5 m 的点）')
    if imu:
        a = np.array(imu); dt = np.diff(a)
        print(f'   IMU  {len(a)} 个 | {a[0]:.3f} → {a[-1]:.3f} s | 间隔中位 {1000*np.median(dt):.1f} ms | 单调 {bool(np.all(dt > 0))}')
    if cloud and imu:
        a = np.array(imu); c = np.array(cloud)
        cnt = np.array([int(((a >= tb) & (a <= te + 0.02)).sum()) for tb, te, *_ in c])
        print(f'   每帧窗口内 IMU 数: 中位 {int(np.median(cnt))} 最小 {cnt.min()} 最大 {cnt.max()}'
              f'  （100 Hz 时 100 ms 窗口应 ≈ 10~12）')
    if odom:
        o = np.array(odom)
        print(f'   odom {len(o)} 条 | {o[0]:.3f} → {o[-1]:.3f} s')
    for k, v in sorted(tf.items(), key=lambda kv: -len(kv[1])):
        s = np.array(v)
        print(f'   TF {k:34s} n={len(v):6d} | 戳 {s[0]:8.3f} → {s[-1]:8.3f} s')
    parents = collections.Counter(k.split('->')[1] for k in tf)
    bad = {f: n for f, n in parents.items() if n > 1}
    print(f'   多父 frame: {bad if bad else "无（✓ TF 树干净）"}')

if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '.')
