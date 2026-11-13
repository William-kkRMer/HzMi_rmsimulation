#!/usr/bin/env python3
# =============================================================================
# make_robot_gz_sdf.py —— 把本仓的 URDF（robot11 等）变成 Gazebo Fortress 能跑的 SDF 模型
# =============================================================================
#
# 输入：一份 URDF 文本（由 launch 用**和 Classic 完全相同的 xacro 参数**生成）。
# 输出：<out-dir>/<model-name>/model.sdf + model.config（可直接被
#       `model://<model-name>` 解析，launch 会把这个 out-dir 放到
#       GZ_SIM_RESOURCE_PATH 的最前面）。
#
# 为什么不是"另写一份 robot11 的 SDF"：robot11 有 113 个真碰撞体 + 12 个 mesh，
# 手抄一定会和 Classic 漂移。这里让它成为**同一份 URDF 的派生**。
#
# 做四件事（每一步都对应一个实测过的 Fortress 坑）：
#  1. 注入 <gazebo reference="<joint>"><preserveFixedJoint>true</preserveFixedJoint></gazebo>
#     —— sdformat 默认会把 fixed joint 折进父 link，`livox_frame` 会**消失**，
#        而我们的插件必须 `LinkByName("livox_frame")`。保留它，才有独立的传感器 link。
#  2. `ign sdf -p` 转 SDF（sdformat12）。它输出的 `<link><pose relative_to='<joint>'>`
#     再把 joint pose 折平进 link pose ⇒ `<link name="livox_frame"><pose>… -0.523599 0 0
#     </pose></link>`，也就是 B① 决策要的「30° 写在 SDF <link><pose> 里」。
#  3. 剥掉所有 Classic 插件/传感器：libros2_livox.so（Classic 的 ray 插件）、
#     libgazebo_ros_imu_sensor.so、libgazebo_ros_planar_move.so。Fortress 加载不了它们
#     （会报 "Failed to load system plugin" 并让模型插入失败）。
#  4. 加回我们的东西：两个 mid360 插件实例（CustomMsg + PointCloud2）、一个 gz `<sensor
#     type="imu">`（<topic> 显式给，插件订阅它转成 ROS 的 /livox/imu）、以及**底盘驱动**
#     `librm_gz_planar_move.so`（Classic 的 planar_move 在 Fortress 上的对应实现，
#     见 src/simulation/rm_simulation/rm_gz_planar_move/src/planar_move_system.cpp）。
#
# 用法：
#   make_robot_gz_sdf.py --urdf <in.urdf> --out-dir <dir> [--model-name robot11]
#                        [--csv <mid360.csv>] [--qos best_effort|reliable]
#                        [--tilt-rpy "-0.523598775598293 0 0"]
#        （--urdf - 表示从 stdin 读）
#   或： make_robot_gz_sdf.py --xacro <file.xacro> [xacro args as k:=v ...] --out-dir <dir>
#
# 打印：生成出来的 model.sdf 路径（最后一行），日志在 stderr。
# =============================================================================
import argparse
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

FIXED_JOINTS_TO_PRESERVE = ('body_to_livox', 'imu_joint')
CLASSIC_PLUGIN_PREFIXES = ('libros2_livox', 'libgazebo_ros_')
# 插件的固定部分（csv / qos 由参数填）
PLUGIN_TMPL = """
    <plugin filename="libmid360_system.so"
            name="mid360_simulation_plugin_ros2::Mid360System">
      <link_name>{link}</link_name>
      <ray_backend>embree</ray_backend>
      <map_key>default</map_key>
      <csv_file>{csv}</csv_file>
      <samples>30000</samples>
      <downsample>1</downsample>
      <update_rate>10</update_rate>
      <point_rate>300000</point_rate>
      <min_range>0.1</min_range>
      <max_range>40.0</max_range>
      <topic>{topic}</topic>
      <publish_pointcloud_type>{kind}</publish_pointcloud_type>
      <frame_id>{frame}</frame_id>
      <output_yaw>0.0</output_yaw>
      <node_name>{node}</node_name>
      <qos_reliability>{qos}</qos_reliability>
      <ignore_self>{ignore_self}</ignore_self>
      <custom_msg_tag>{custom_msg_tag}</custom_msg_tag>{imu}
    </plugin>"""


def log(msg):
    print('[make_robot_gz_sdf] ' + msg, file=sys.stderr)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def rpy_from_quat(q):
    x, y, z, w = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def fmt(vals):
    return ' '.join('%.12g' % v for v in vals)


def parse_pose(text):
    parts = [float(v) for v in (text or '0 0 0 0 0 0').split()]
    while len(parts) < 6:
        parts.append(0.0)
    return parts[:3], parts[3:6]


def rel_to_model_pose(link, parents):
    """把 <pose relative_to='joint'> 折成相对 model 的绝对 pose。

    parents[link] = (parent_link, joint_pose_xyz, joint_pose_rpy)；只有一层（robot11
    是 base_link -> livox_frame / imu_link）。用矩阵/四元数做，不假设 rpy 分量能相加。
    """
    xyz, rpy = parse_pose(link.findtext('pose'))
    q = quat_from_rpy(*rpy)
    p = list(xyz)
    cur = link.get('name')
    guard = 0
    while cur in parents and guard < 64:
        parent, jxyz, jrpy = parents[cur]
        jq = quat_from_rpy(*jrpy)
        # 绝对 = 关节位姿 ∘ 子 link 相对关节的位姿（只加 R_j*p 会漏掉关节平移）
        rp = quat_rotate(jq, p)
        p = [jxyz[i] + rp[i] for i in range(3)]
        q = quat_mul(jq, q)
        cur = parent
        guard += 1
    return p, quat_from_rpy(*rpy_from_quat(q))


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--urdf', help="URDF 文件；'-' = stdin")
    src.add_argument('--xacro', help='xacro 文件（本脚本自己跑 xacro）')
    ap.add_argument('--xacro-arg', action='append', default=[],
                    help='透传给 xacro 的 k:=v（可重复）')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--model-name', default='rm_robot11_gz')
    ap.add_argument('--csv', default='', help='mid360.csv 绝对路径')
    ap.add_argument('--qos', default='best_effort', choices=['best_effort', 'reliable'])
    ap.add_argument('--tilt-rpy', default='',
                    help='启用「强制把传感器 link 拧到该 rpy」；默认空 = 用 URDF 关节里已有的姿态')
    ap.add_argument('--imu-frame', default='imu_link')
    # ★ 2026-11-03（goal 轮 6）：传感器来源。
    #   mid360（默认）= 第三方 libmid360_system.so 双发布实例（本仓正式路径，保真）
    #   native        = gz 原生 <sensor type="gpu_lidar"> + <sensor type="imu">
    #                   （**无第三方依赖**，但规则网格、无逐点时间 ⇒ 只用于管线验证）
    ap.add_argument('--sensor', default='mid360', choices=['mid360', 'native'])
    ap.add_argument('--lidar-hsamples', default='640', help='native: 水平采样数（MID-360 FOV 360°）')
    ap.add_argument('--lidar-vsamples', default='32', help='native: 垂直采样数（-7°..+52°）')
    ap.add_argument('--lidar-rate', default='10', help='native: 雷达频率 Hz')
    # ★ 2026-11-03：`auto` = 有底盘驱动（--drive true）时**必须 dynamic**。
    #   为什么：gz 的 <static>true</static> 模型即使被插件 SetWorldPose 改了位姿，
    #   也不会进 `dynamic_pose/info` ⇒ **GUI 里车不动、RViz 里却在动**（实测：
    #   static 时 /odom_ground_truth 从 5.9→8.25 而世界里 chassis 位姿不变；
    #   改 dynamic 后世界位姿 7.98 ✓）。robot11 无底盘驱动 ⇒ 保持 static。
    ap.add_argument('--static', default='auto', choices=['auto', 'true', 'false'])
    # ★ 2026-11-04：**"GUI 里车会动"**的实验档。Fortress 6 (SDF 1.9) **不支持**
    #   `<kinematic>`（实测 `ign sdf -p` 会直接丢掉这个标签），所以"kinematic 模型"这条路走不通。
    #   退一步：**dynamic + 剥掉所有 <collision> + 每根 link 关重力** ——
    #   仍然进 dynamic_pose/info（GUI/位姿会跟随），但没有碰撞体 ⇒ 不会与场地 mesh 做接触求解
    #   ⇒ 不会再出现"求解发散 → NaN"那条链。
    # ★ 2026-11-04：**PolarBear(pb2025) 的做法**：sim 机器人是 SDF 里**真的轮子关节** +
    #   MecanumDrive/DiffDrive 插件，靠物理走 ⇒ GUI 天然跟着动。我们原来的做法是
    #   "URDF 的轮子是 fixed + 插件直接 teleport 位姿"，所以 GUI 不动、dynamic 还会发散。
    #   `--drive-mode wheels` = 把轮子关节改成 revolute + 挂 gz DiffDrive（opt-in，默认仍 planar）。
    ap.add_argument('--drive-mode', default='planar', choices=['planar', 'wheels'],
                    help="planar（默认）= 现在的运动学 teleport 插件；"
                         "wheels = **真轮子**（revolute 关节 + ignition-gazebo-diff-drive-system），"
                         "靠物理走，gz GUI 会正常跟随（PolarBear 的做法）")
    ap.add_argument('--wheel-separation', default='0.218', help='轮距 m')
    ap.add_argument('--wheel-radius', default='0.058', help='轮半径 m')
    ap.add_argument('--motion', default='static',
                    choices=['static', 'visual', 'physics', 'safe'],
                    help="static（最稳；gz GUI 里车不动）= <static>true</static>；"
                         "visual = dynamic + 去掉 collision + 关重力（GUI 可能不刷新）；"
                         "physics = **原样动态**（保留 collision、保留重力，最像 Classic），"
                         "前提是出生高度正确（否则会插进场地 → 接触力爆掉 → NaN）；"
                         "safe = dynamic + **保留 collision** + `collide_bitmask=2`（与场地不碰）"
                         "+ 关重力 —— 想同时要「GUI 正常渲染」与「不接触发散」时用这个")
    ap.add_argument('--ignore-self', default='true', choices=['true', 'false'],
                    help='true（默认）= 把自己所属 model 注册进 MapRegistry，'
                         'BVH 里剔除它 ⇒ 看不到自己的车体；false 只用于对照实验')
    ap.add_argument('--drive', default='true', choices=['true', 'false'],
                    help='true（默认）= 给模型加底盘驱动插件（运动学 planar_move：'
                         '/cmd_vel_chassis → 位姿 + /odom_ground_truth）；'
                         'false = 保持"纯静态底座"（A/B 对照用）')
    ap.add_argument('--drive-odom-rate', default='50',
                    help='真值里程计 /odom_ground_truth 的发布率（Hz，仿真时间）')
    ap.add_argument('--custom-msg-tag', default='0',
                    help='/livox/lidar（CustomMsg）每个点的 tag 字节。**默认 0 = 与 Classic 的 '
                         'libros2_livox.so 一致**；small_point_lio 的适配器要求 (tag & 0x3f)==0，'
                         '上游写死的 0x10 会让它把每一帧的每个点都丢掉（实测症状：LIO 永不初始化、'
                         '没有 odom→base_link 与 /cloud_registered）。传 16 可复现上游行为')
    args = ap.parse_args()

    if args.xacro:
        cmd = ['xacro', args.xacro] + list(args.xacro_arg)
        log('run: ' + ' '.join(cmd))
        urdf = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    elif args.urdf == '-':
        urdf = sys.stdin.read()
    else:
        urdf = open(args.urdf).read()

    # ---- 1. 注入 preserveFixedJoint（否则 livox_frame 会被折进 base_link 而消失）
    # ★ 2026-11-03（goal 轮 1）：不再**只**看写死的两个 joint 名 —— 从 URDF 里按 child link
    #   自动找（任一 fixed joint，其 child 是传感器 link）。这样 robot11 / chassis /
    #   以后的新车都自动覆盖；写死的两个名字保留在第一顺位，robot11 的生成物逐字节不变。
    _sensor_links = ('livox_frame', 'livox_link', 'lidar_link', 'imu_link')
    preserve = list(FIXED_JOINTS_TO_PRESERVE)
    try:
        _root = ET.fromstring(urdf)
        for _j in _root.findall('joint'):
            if (_j.get('type') or '') != 'fixed':
                continue
            # URDF 里是 <child link="livox_frame"/>（**属性**，不是文本）⇒ 必须读 attribute
            _ce = _j.find('child')
            _c = ((_ce.get('link') if _ce is not None else '') or '').strip()
            _n = _j.get('name') or ''
            if _c in _sensor_links and _n and _n not in preserve:
                preserve.append(_n)
    except ET.ParseError:
        pass
    log('preserveFixedJoint 注入：%s' % (', '.join(preserve) or '(空)'))
    inject = ''.join(
        '\n  <gazebo reference="%s"><preserveFixedJoint>true</preserveFixedJoint></gazebo>'
        % j for j in preserve)
    if '</robot>' not in urdf:
        sys.exit('URDF 里没有 </robot>，不像一份 URDF')
    urdf2 = urdf.replace('</robot>', inject + '\n</robot>')

    # ---- 1b. ★ 2026-11-03（goal 轮 1）：给**没有 inertial 的传感器 link 补一个极小 inertial。
    #   `ign sdf -p`（sdformat12）的 urdf 解析器会把"无 inertial 的 link"在 fixed-joint lumping
    #   时直接丢掉，连 <preserveFixedJoint> 都救不回来（实测 chassis：所有 link 塌成 base_link，
    #   报 "link[imu_link] has no <inertial> block defined ... parent joint[imu_joint] ignored"）。
    #   robot11 的传感器 link 自带 inertial 所以没暴露。补 1e-6 kg 不影响物理（fixed joint）。
    try:
        _r = ET.fromstring(urdf2)
        _added = []
        for _lk in _r.findall('link'):
            if (_lk.get('name') or '') not in _sensor_links:
                continue
            if _lk.find('inertial') is not None:
                continue
            _in = ET.SubElement(_lk, 'inertial')
            ET.SubElement(_in, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
            ET.SubElement(_in, 'mass', {'value': '1e-6'})
            ET.SubElement(_in, 'inertia', {'ixx': '1e-9', 'ixy': '0', 'ixz': '0',
                                           'iyy': '1e-9', 'iyz': '0', 'izz': '1e-9'})
            _added.append(_lk.get('name'))
        # ★ 2026-11-03（goal 轮 1b）：joint 与 link 同名 ⇒ `ign sdf -p` 会生成两个同名 <frame>
        #   （实测 chassis：link w1..w4 与 joint w1..w4 同名 ⇒ "frame with name[w1] already exists"，
        #   `ign sdf -k` 直接报错）。SDF 里 frame 名必须全局唯一 ⇒ 把撞名的 joint 改名为 <n>_joint。
        #   只改喂给 ign 的这份 URDF，xacro/Classic 一行不动。
        _link_names = {(_lk.get('name') or '') for _lk in _r.findall('link')}
        _renamed = []
        for _jj in _r.findall('joint'):
            _jn = _jj.get('name') or ''
            if _jn and _jn in _link_names:
                _new = _jn + '_joint'
                while _new in _link_names:
                    _new += '_'
                _jj.set('name', _new)
                _renamed.append('%s→%s' % (_jn, _new))
        if _renamed:
            log('joint 重命名（与 link 同名，SDF frame 会重复）：%s' % ', '.join(_renamed))
        if _added or _renamed:
            urdf2 = ET.tostring(_r, encoding='unicode')
        if _added:
            log('补 inertial（无惯性块的传感器 link）：%s' % ', '.join(_added))
    except ET.ParseError:
        pass

    # ---- 1c. drive-mode=wheels：**在 URDF 阶段**把轮子关节改成 revolute。
    #   必须在 `ign sdf -p` 之前改 —— fixed joint 会被 sdformat 折进父 link（joint 直接消失），
    #   之后再改就晚了（实测：SDF 里既没有 revolute 也没有 joint 可挂 DiffDrive）。
    _wheel_joint_names = []
    if args.drive_mode == 'wheels':
        try:
            _w = ET.fromstring(urdf2)
            for _j in _w.findall('joint'):
                _ch = _j.find('child')
                _cl = (_ch.get('link') if _ch is not None else '')
                if _cl not in ('w1', 'w2', 'w3', 'w4'):
                    continue
                _j.set('type', 'revolute')
                _ax = _j.find('axis')
                if _ax is None:
                    _ax = ET.SubElement(_j, 'axis')
                _x = _ax.find('xyz')
                if _x is None:
                    _x = ET.SubElement(_ax, 'xyz')
                _x.text = '0 1 0'
                # ⚠️ URDF 里 <limit> 是 **<joint> 的直接子元素**（不是 <axis> 的子元素），
                #   且属性写成 child elements 也会让 `ign sdf -p` 报
                #   "SDF parsing the xml failed"（两个坑都踩过，实测）。
                if _j.find('dynamics') is None:
                    _dyn = ET.SubElement(_j, 'dynamics')
                    _dyn.set('damping', '0.05')
                    _dyn.set('friction', '0.2')
                if _j.find('limit') is None:
                    _lim = ET.SubElement(_j, 'limit')
                    _lim.set('lower', '-1000000')
                    _lim.set('upper', '1000000')
                    _lim.set('effort', '200')
                    _lim.set('velocity', '40')
                _wheel_joint_names.append(_j.get('name'))
            if _wheel_joint_names:
                urdf2 = ET.tostring(_w, encoding='unicode')
                log('drive-mode=wheels：URDF 里轮子关节 → revolute：%s'
                    % ', '.join(_wheel_joint_names))
            else:
                log('⚠ drive-mode=wheels：URDF 里没找到 w1..w4 的 joint')
        except ET.ParseError:
            log('⚠ drive-mode=wheels：URDF 解析失败，改不了轮子关节')

    # ---- 2. ign sdf -p（用临时文件：`ign sdf -p -` 在 Fortress 上不接受 stdin）
    import tempfile
    tmpd = tempfile.mkdtemp(prefix='mkgzsdf_')
    tmp_urdf = os.path.join(tmpd, 'in.urdf')
    with open(tmp_urdf, 'w') as f:
        f.write(urdf2)
    log('run: ign sdf -p %s' % tmp_urdf)
    conv = subprocess.run(['ign', 'sdf', '-p', tmp_urdf],
                          capture_output=True, text=True)
    if conv.returncode != 0:
        log(conv.stdout[-2000:])
        sys.exit('ign sdf -p 失败：\n' + conv.stderr[-4000:])
    sdf_text = conv.stdout

    # ---- 3. 后处理
    root = ET.fromstring(sdf_text)
    model = root.find('model')
    if model is None:
        sys.exit('转换结果里没有 <model>')

    # 2a. 先收集 joint 的 pose（折平用），再把 fixed joint 全部删掉
    parents = {}
    links = {l.get('name'): l for l in model.findall('link')}
    for j in list(model.findall('joint')):
        jxyz, jrpy = parse_pose(j.findtext('pose'))
        child = (j.find('child') is not None and j.find('child').text or '').strip()
        parent = (j.find('parent') is not None and j.find('parent').text or '').strip()
        if child and parent:
            parents[child] = (parent, jxyz, jrpy)
    for j in list(model.findall('joint')):
        model.remove(j)

    # 2b. link pose 折成绝对（去掉 relative_to）
    for name, link in links.items():
        pose = link.find('pose')
        abs_p, abs_q = rel_to_model_pose(link, parents)
        rpy = rpy_from_quat(abs_q)
        if pose is None:
            pose = ET.SubElement(link, 'pose')
        if pose.get('relative_to') is not None:
            del pose.attrib['relative_to']
        pose.text = fmt(list(abs_p) + list(rpy))

    # 2c. 剥掉 Classic 插件块 + ray 传感器；IMU 传感器保留但换成 gz 的写法
    dropped = []
    for link in model.findall('link'):
        for sensor in list(link.findall('sensor')):
            stype = sensor.get('type')
            if stype == 'ray' or stype == 'gpu_ray':
                for pl in sensor.findall('plugin'):
                    dropped.append(pl.get('filename'))
                link.remove(sensor)
                continue
            # imu：清掉 Classic 插件，加 <topic>
            for pl in list(sensor.findall('plugin')):
                dropped.append(pl.get('filename'))
                sensor.remove(pl)
            if sensor.find('topic') is None:
                t = ET.SubElement(sensor, 'topic')
                t.text = '/mid360/imu_gz'
            if sensor.find('always_on') is None:
                ao = ET.SubElement(sensor, 'always_on')
                ao.text = 'true'
    for pl in list(model.findall('plugin')):
        fn = pl.get('filename') or ''
        if fn.startswith(CLASSIC_PLUGIN_PREFIXES):
            dropped.append(fn)
            model.remove(pl)
    if dropped:
        log('剥掉的 Classic 插件/传感器插件：' + ', '.join(dropped))

    # 2d. 找传感器 link（livox_frame / 其关节的 child）
    link_name = None
    for cand in ('livox_frame', 'livox_link', 'lidar_link'):
        if cand in links:
            link_name = cand
            break
    if link_name is None:
        sys.exit('找不到传感器 link（livox_frame / livox_link / lidar_link）：'
                 + ', '.join(links))
    log('sensor link = %s' % link_name)

    # 2e. --tilt-rpy：强制把该 link 的 roll 写成指定值（B① 的字面实现）
    if args.tilt_rpy:
        link = links[link_name]
        p, _ = parse_pose(link.findtext('pose'))
        tilt = [float(v) for v in args.tilt_rpy.split()]
        while len(tilt) < 3:
            tilt.append(0.0)
        link.find('pose').text = fmt(p + tilt[:3])
        log('强制 %s 的 rpy = %s' % (link_name, fmt(tilt[:3])))

    # 2e2. --sensor native：把 gz 原生传感器挂到 link 里（SDF 规定 <sensor> 必须是 link 的子元素）
    if args.sensor == 'native':
        def _add_sensor(link_el, xml_text):
            frag = ET.fromstring('<frag>%s</frag>' % xml_text)
            for ch in list(frag):
                link_el.append(ch)

        lidar_xml = '''
      <sensor name="lidar" type="gpu_lidar">
        <topic>/gz/lidar</topic>
        <update_rate>%s</update_rate>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <lidar>
          <scan>
            <horizontal><samples>%s</samples><min_angle>-3.141592</min_angle><max_angle>3.141592</max_angle></horizontal>
            <vertical><samples>%s</samples><min_angle>-0.122173</min_angle><max_angle>0.907571</max_angle></vertical>
          </scan>
          <range><min>0.2</min><max>40.0</max></range>
        </lidar>
      </sensor>''' % (args.lidar_rate, args.lidar_hsamples, args.lidar_vsamples)
        _add_sensor(links[link_name], lidar_xml)
        log('native 传感器：lidar 挂 %s（h=%s v=%s @%s Hz, 360°/-7°..+52°）'
            % (link_name, args.lidar_hsamples, args.lidar_vsamples, args.lidar_rate))

        if args.imu_frame in links:
            imu_xml = '''
      <sensor name="imu" type="imu">
        <topic>/livox/imu</topic>
        <update_rate>100</update_rate>
        <always_on>true</always_on>
      </sensor>'''
            _add_sensor(links[args.imu_frame], imu_xml)
            log('native 传感器：imu 挂 %s（100 Hz）' % args.imu_frame)
        else:
            log('⚠ native：找不到 imu link（%s）⇒ 没有 IMU 输出' % args.imu_frame)

    # 2f. 静态化（无轮子关节 ⇒ 不该让物理引擎去"掉落"它）
    _static = args.static
    if _static == 'auto':
        # ★ 2026-11-04 **回退**：drive 时改成 dynamic 会让底盘与场地 mesh 产生接触求解，
        #   实测本机出现 NaN（点云变 NaN ⇒ LIO 状态 NaN ⇒ /odom 空、lio_tf_adapter 发 NaN TF
        #   ⇒ RViz/Gazebo 都看不到车）。所以 auto 一律 static（= 改造前行为）：
        #   代价是 gz GUI 里那辆车不会跟着动（RViz 正常）。要 GUI 也动，需要另一套修法（见 docs）。
        _static = 'true'
        log('static=auto ⇒ true（drive=%s；dynamic 会与场地接触求解出 NaN，已回退）' % args.drive)
    if args.motion in ('visual', 'physics', 'safe'):
        _static = 'false'
        log('motion=visual ⇒ <static>false</static> + 剥掉 collision + 关重力'
            '（GUI 会跟随；无碰撞体 ⇒ 不触发接触求解）')
    if model.find('static') is None:
        st = ET.SubElement(model, 'static')
        st.text = _static
    else:
        model.find('static').text = _static

    # 2f-2a. motion=safe：保留 collision（GUI 能渲染成正常动态模型），但让它**不与场地碰撞**：
    #   · 每个 collision 的 <surface><contact><collide_bitmask> = 2（场地 category=1 ⇒ 位与为 0）
    #   · 每根 link 关重力（否则没有支撑会掉下去）
    #   这样既不会有接触求解（NaN 的来源），又保持"普通动态模型"的身份。
    # ★ 2026-11-06：轮子摩擦/接触参数。实测（gz, wheels, physics）：
    #   发 0.5 m/s 4 s，odom 走 2.09 m，但 dynamic_pose 只移动 0.67 m ⇒ **打滑 3 倍**。
    #   Fortress 默认 mu 偏低 + 无接触刚度，所以显式给一遍。
    # ★ 实验开关：轮子碰撞体换成**球**（GZ_WHEEL_SPHERE=1）。
    #   动机：cylinder-vs-trimesh 的接触在 ODE/Bullet 里出了名的差（点接触不稳 ⇒ 持续打滑），
    #   而球-三角网格的接触要稳得多。半径 = 轮半径，球心在轮心。
    # ★ 实验开关：去掉底盘（base_link）碰撞体（GZ_NOC_BASE=1），用来验证
    #   "底盘盒子在刮地、轮子没受力"这一假设。
    if os.environ.get('GZ_NOC_BASE', '0') == '1':
        _n = 0
        for _lnk in model.findall('link'):
            if _lnk.get('name') != 'base_link':
                continue
            for _c in _lnk.findall('collision'):
                _lnk.remove(_c)
                _n += 1
        if _n:
            log('实验：删除 base_link 的 %d 个 collision（验证是否底盘刮地）' % _n)

    if args.drive_mode == 'wheels' and os.environ.get('GZ_WHEEL_SPHERE', '0') == '1':
        _n = 0
        for _lnk in model.findall('link'):
            if _lnk.get('name') not in ('w1', 'w2', 'w3', 'w4'):
                continue
            for _c in _lnk.findall('collision'):
                _g = _c.find('geometry')
                if _g is None:
                    continue
                for _ch in list(_g):
                    _g.remove(_ch)
                ET.SubElement(_g, 'sphere').append(ET.Element('radius'))
                _g.find('sphere').find('radius').text = '0.058'
                _p = _c.find('pose')
                if _p is None:
                    _p = ET.SubElement(_c, 'pose')
                _p.text = '0 0 0 0 0 0'
                _n += 1
        if _n:
            log('physics：轮子碰撞体 → 球（r=0.058），共 %d 个（球-三角网格接触更稳）' % _n)

    if args.drive_mode == 'wheels' and args.motion in ('physics', 'safe'):
        _n_wheel = 0
        for _lnk in model.findall('link'):
            if _lnk.get('name') not in ('w1', 'w2', 'w3', 'w4'):
                continue
            for _c in _lnk.findall('collision'):
                _s = _c.find('surface')
                if _s is None:
                    _s = ET.SubElement(_c, 'surface')
                _f = _s.find('friction')
                if _f is None:
                    _f = ET.SubElement(_s, 'friction')
                _o = _f.find('ode')
                if _o is None:
                    _o = ET.SubElement(_f, 'ode')
                ET.SubElement(_o, 'mu').text = '1.2'
                ET.SubElement(_o, 'mu2').text = '1.2'
                _ct = _s.find('contact')
                if _ct is None:
                    _ct = ET.SubElement(_s, 'contact')
                _co = _ct.find('ode')
                if _co is None:
                    _co = ET.SubElement(_ct, 'ode')
                ET.SubElement(_co, 'kp').text = '1000000'
                ET.SubElement(_co, 'kd').text = '100'
                _n_wheel += 1
        if _n_wheel:
            log('physics：给 %d 个轮子碰撞体加摩擦 mu=1.2 / kp=1e6 kd=100' % _n_wheel)

    if args.motion == 'safe':
        _n_c = 0
        for _lnk in model.findall('link'):
            for _c in _lnk.findall('collision'):
                _surf = _c.find('surface')
                if _surf is None:
                    _surf = ET.SubElement(_c, 'surface')
                _ct = _surf.find('contact')
                if _ct is None:
                    _ct = ET.SubElement(_surf, 'contact')
                _bm = _ct.find('collide_bitmask')
                if _bm is None:
                    _bm = ET.SubElement(_ct, 'collide_bitmask')
                _bm.text = '2'
                _n_c += 1
            if _lnk.find('gravity') is None:
                _g = ET.SubElement(_lnk, 'gravity')
                _g.text = 'false'
        log('motion=safe：%d 个 collision 设 collide_bitmask=2（不与场地碰撞）+ 关重力'
            % _n_c)

    # 2f-2b. motion=physics：什么都不改（保留 collision / 重力），只是 <static>false</static>
    if args.motion == 'physics':
        log('motion=physics：保留 collision 与重力（最接近 Classic 的动态底盘）')

    # 2f-2. motion=visual：去掉所有 collision（不参与接触求解）+ 每根 link 关重力
    if args.motion == 'visual':
        _n_coll = 0
        for _lnk in model.findall('link'):
            for _c in _lnk.findall('collision'):
                _lnk.remove(_c)
                _n_coll += 1
            if _lnk.find('gravity') is None:
                _g = ET.SubElement(_lnk, 'gravity')
                _g.text = 'false'
        log('motion=visual：剥掉 %d 个 collision、%d 根 link 关重力'
            % (_n_coll, len(model.findall('link'))))

    # 2f-3. drive-mode=wheels（PolarBear 的做法：真轮子关节 + gz DiffDrive）
    #   ⚠️ 关键教训：`ign sdf -p` 会把 URDF 的 fixed joint **折平**（连 joint 元素都不留），
    #   所以在 URDF 阶段把 fixed 改成 revolute 也**没用**（实测 SDF 里 0 个 joint）。
    #   正解：**在 SDF 阶段自己建关节** —— 用轮子 link 当前的 <pose>（相对模型）作为关节位姿，
    #   再把该 link 的 pose 清零（SDF 里 link 的位置由 joint 决定）。
    # URDF 阶段那次改动**不足以**保留关节（sdformat 仍会折平 fixed→没有任何 joint），
    # 所以这里**无条件**在 SDF 里自建轮子关节。
    _wheel_joints = []
    if args.drive_mode == 'wheels':
        _base = None
        for _lnk in model.findall('link'):
            if _lnk.get('name') == 'base_link':
                _base = _lnk
                break
        _base_p = _base.findtext('pose') if _base is not None else None
        _bp = [float(v) for v in (_base_p or '0 0 0 0 0 0').split()]
        for _lnk in model.findall('link'):
            _nm = _lnk.get('name')
            if _nm not in ('w1', 'w2', 'w3', 'w4'):
                continue
            _lp = [float(v) for v in (_lnk.findtext('pose') or '0 0 0 0 0 0').split()]
            _rel = [_lp[i] - _bp[i] for i in range(3)] + [_lp[3 + i] - _bp[3 + i] for i in range(3)]
            # ★ 2026-11-06 实验开关：轮距对称化。
            #   URDF 里四轮是**平行四边形**（前轴 y 偏 +3.16 cm、后轴 -3.16 cm，整体歪 11.1°），
            #   用 DiffDrive 驱动会持续侧滑（实测 odom 1.57 m vs 实走 0.54 m）。
            #   GZ_WHEEL_SYMMETRIZE=1 时把每个轮子的 |y| 统一成均值（0.1405），只改 gz 侧关节位姿，
            #   **不动 URDF**（真车描述仍是原样）。
            # ★★ 2026-11-06 关键修复：**轮子 link 原点不在轮胎中心**。
            #   实测 STL 包围盒中心：w1/w2 的 mesh 中心 y = −0.0315，w3/w4 = +0.0315
            #   ⇒ 关节轴通过 link 原点时，轮胎是**偏心**的（绕轴画半径 3.15 cm 的圆）
            #   ⇒ 轮子转动时"边转边跳"，车体只走 1/3 距离（实测 odom 3.10 m vs 车体 0.96 m）。
            #   修法：把 link 原点平移到轮胎中心（关节 y 加上 mesh 中心偏移），
            #   同时把该 link 的碰撞体偏移清零（视觉 mesh 也自然落回中心）。
            # 说明（2026-11-06 晚）：**不要**把 link 原点挪到"轮胎中心"。
            #   STL 包围盒中心 y=±0.0315 只说明"轮毂在原点、轮胎沿轴外移"，
            #   绕 Y 轴旋转仍是绕轮胎自身轴，并不偏心（实测挪过去后打滑更差 4.6×）。

            # 实验开关：把轮子整体下移（验证"底盘盒子先着地、轮子没受力"这一假设）
            _drop = float(os.environ.get('GZ_WHEEL_DROP', '0'))
            if _drop:
                _rel[2] -= _drop
            if os.environ.get('GZ_WHEEL_SYMMETRIZE', '0') == '1' and abs(_rel[1]) > 1e-6:
                _rel[1] = math.copysign(0.1405, _rel[1])
            _j = ET.SubElement(model, 'joint')
            _j.set('name', _nm + '_joint')
            _j.set('type', 'revolute')
            ET.SubElement(_j, 'parent').text = 'base_link'
            ET.SubElement(_j, 'child').text = _nm
            ET.SubElement(_j, 'pose').text = ' '.join('%.9g' % v for v in _rel)
            _ax = ET.SubElement(_j, 'axis')
            # ★ 2026-11-06：轴取 **-y**。实测 `0 1 0` 时 odom 积分 +x 但车体实际走 **-x**
            #   （dynamic_pose: 5.911 → 5.392），即正转轮子把车推向后 ⇒ 轴符号反了。
            ET.SubElement(_ax, 'xyz').text = '0 -1 0'
            _lim = ET.SubElement(_ax, 'limit')
            ET.SubElement(_lim, 'lower').text = '-1e9'
            ET.SubElement(_lim, 'upper').text = '1e9'
            ET.SubElement(_lim, 'effort').text = '200'
            ET.SubElement(_lim, 'velocity').text = '40'
            # link 的 pose 归零（位置交给 joint）
            _pl = _lnk.find('pose')
            if _pl is None:
                _pl = ET.SubElement(_lnk, 'pose')
            _pl.text = '0 0 0 0 0 0'
            _wheel_joints.append(_nm + '_joint')
        if _wheel_joints:
            log('drive-mode=wheels：在 SDF 里新建轮子关节（revolute）：%s'
                % ', '.join(_wheel_joints))

    # 2f-3. drive-mode=wheels（PolarBear 的做法：真轮子关节 + gz DiffDrive）
    _wheel_joints = _wheel_joints or list(_wheel_joint_names)
    if args.drive_mode == 'wheels' and not _wheel_joints:
        for _j in model.findall('joint'):
            _child = _j.find('child')
            _cl = (_child.get('link') if _child is not None else '')
            if _cl not in ('w1', 'w2', 'w3', 'w4'):
                continue
            _j.set('type', 'revolute')
            _ax = _j.find('axis')
            if _ax is None:
                _ax = ET.SubElement(_j, 'axis')
            _xyz = _ax.find('xyz')
            if _xyz is None:
                _xyz = ET.SubElement(_ax, 'xyz')
            _xyz.text = '0 1 0'
            if _ax.find('limit') is None:
                _lim = ET.SubElement(_ax, 'limit')
                for _k, _v in (('lower', '-1e9'), ('upper', '1e9'),
                               ('effort', '200'), ('velocity', '40')):
                    ET.SubElement(_lim, _k).text = _v
            _wheel_joints.append(_j.get('name'))
        log('drive-mode=wheels：轮子关节 → revolute：%s' % ', '.join(_wheel_joints))

    # 2g/2h. 序列化后按字符串插入两个实例（ET 对注释/tail 的处理很容易出错，
    #        字符串插入更直白），顺便把版本钉到 1.9（sdformat12 的上限；
    #        1.10 会让整个世界加载失败，实测见 livox_sensor_prior_art.md §10.4.6）。
    root.set('version', '1.9')
    ET.indent(root, space='  ')
    xml = ET.tostring(root, encoding='utf-8').decode('utf-8')
    if args.drive_mode == 'wheels' and _wheel_joints:
        # 名字可能是 w1/w3，也可能被重命名成 w1_joint/w3_joint ⇒ 用**前缀**匹配
        _left = [j for j in _wheel_joints if j.startswith(('w1', 'w3'))]
        _right = [j for j in _wheel_joints if j.startswith(('w2', 'w4'))]
        _dd = ('\n      <plugin filename="ignition-gazebo-diff-drive-system"'
               ' name="ignition::gazebo::systems::DiffDrive">'
               + ''.join('\n        <left_joint>%s</left_joint>' % j for j in _left)
               + ''.join('\n        <right_joint>%s</right_joint>' % j for j in _right)
               + '\n        <wheel_separation>%s</wheel_separation>' % args.wheel_separation
               + '\n        <wheel_radius>%s</wheel_radius>' % args.wheel_radius
               + '\n        <topic>cmd_vel_chassis</topic>'
               + '\n        <odom_topic>/odom_ground_truth</odom_topic>'
               + '\n        <publish_odom>true</publish_odom>'
               + '\n        <publish_odom_tf>false</publish_odom_tf>'
               + '\n        <publish_wheel_tf>false</publish_wheel_tf>'
               + '\n        <odom_publish_frequency>50</odom_publish_frequency>'
               + '\n      </plugin>')
        xml = xml.replace('</model>', _dd + '\n    </model>', 1)
        log('drive-mode=wheels：DiffDrive left=%s right=%s sep=%s r=%s'
            % (','.join(_left), ','.join(_right), args.wheel_separation, args.wheel_radius))
    # 顶层是 <sdf><model>…</model></sdf>：插件必须插进 <model>，不是 <sdf>。
    imu_block = ('\n      <imu_topic>/livox/imu</imu_topic>'
                 '\n      <imu_frame_id>%s</imu_frame_id>'
                 '\n      <gz_imu_topic>/mid360/imu_gz</gz_imu_topic>'
                 '\n      <publish_imu>true</publish_imu>' % args.imu_frame)
    if args.sensor == 'native':
        plugins = ''
    else:
        plugins = (
        '\n    <!-- HzMi: 双发布实例 #1 —— 只发 CustomMsg（LIO 用 /livox/lidar） -->'
        + PLUGIN_TMPL.format(link=link_name, csv=args.csv, topic='/livox/lidar',
                             kind='3', frame=link_name, node='mid360_gz_custom',
                             qos=args.qos, ignore_self=args.ignore_self,
                             custom_msg_tag=args.custom_msg_tag, imu=imu_block)
        + '\n    <!-- HzMi: 双发布实例 #2 —— 只发 PointCloud2（linefit 用 /livox/lidar/pointcloud） -->'
        + PLUGIN_TMPL.format(link=link_name, csv=args.csv,
                             topic='/livox/lidar/pointcloud', kind='2',
                             frame=link_name, node='mid360_gz_pc2', qos=args.qos,
                             ignore_self=args.ignore_self,
                             custom_msg_tag=args.custom_msg_tag,
                             imu='\n      <publish_imu>false</publish_imu>')
    )
    # ★ 2026-11-04：**wheels 档不能再挂 planar 插件**。两者都驱动同一个模型
    #   （planar 每 tick 写 components::Pose、DiffDrive 走物理）⇒ 互相打架 ⇒ 求解发散 NaN。
    #   实测就是这么炸的：既插了 diff-drive，又留着 planar_move。
    # ★ 诊断开关（GZ_JOINT_STATES=1）：加 joint_state_publisher，用来读**真实关节速度**，
    #   判断打滑是"关节没转到指令值"还是"轮子转了但地面打滑"。
    if os.environ.get('GZ_JOINT_STATES', '0') == '1':
        plugins = plugins + (
            '\n    <plugin filename="ignition-gazebo-joint-state-publisher-system"'
            ' name="ignition::gazebo::systems::JointStatePublisher">'
            '\n      <topic>joint_states</topic>\n    </plugin>')
        log('诊断：加 joint_state_publisher（topic=joint_states）')

    if args.drive == 'true' and args.drive_mode == 'wheels':
        log('底盘驱动：drive-mode=wheels ⇒ **不挂** planar_move（位姿交给物理 + DiffDrive）')
    elif args.drive == 'true':
        drive_link = 'base_link' if 'base_link' in links else sorted(links)[0]
        plugins = plugins + (
            '\n    <!-- HzMi: 底盘驱动（Classic 的 libgazebo_ros_planar_move.so 的对应实现）\n'
            '         运动学、平面；/cmd_vel_chassis(gz Twist) → 模型位姿 + /odom_ground_truth。\n'
            '         为什么不是 DiffDrive/VelocityControl：见该插件源码头部与\n'
            '         docs/fortress_sim_backend.md「底盘运动」一节。 -->\n'
            '    <plugin filename="librm_gz_planar_move.so" name="rm_gz::PlanarMove">\n'
            '      <link_name>%s</link_name>\n'
            '      <topic>/cmd_vel_chassis</topic>\n'
            '      <odom_topic>/odom_ground_truth</odom_topic>\n'
            '      <odom_frame>odom</odom_frame>\n'
            '      <base_frame>%s</base_frame>\n'
            '      <odom_rate>%s</odom_rate>\n'
            '      <publish_odom>true</publish_odom>\n'
            '      <command_timeout>0.0</command_timeout>\n'
            '      <max_vx>3.0</max_vx>\n'
            '      <max_wz>4.0</max_wz>\n'
            '    </plugin>' % (drive_link, drive_link, args.drive_odom_rate))
        log('底盘驱动：link=%s topic=/cmd_vel_chassis odom_rate=%s Hz'
            % (drive_link, args.drive_odom_rate))

    close = xml.rindex('</model>')
    xml = xml[:close] + plugins + '\n  </model>' + xml[close + len('</model>'):]

    out_model_dir = os.path.join(args.out_dir, args.model_name)
    os.makedirs(out_model_dir, exist_ok=True)
    out_sdf = os.path.join(out_model_dir, 'model.sdf')
    with open(out_sdf, 'w') as f:
        f.write("<?xml version='1.0'?>\n" + xml + '\n')
    with open(os.path.join(out_model_dir, 'model.config'), 'w') as f:
        f.write("""<?xml version="1.0" ?>
<model>
  <name>%s</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>HzMi</name><email>hzmi@example.com</email></author>
  <description>在运行期由 scripts/make_robot_gz_sdf.py 从本仓 URDF 生成（Gazebo Fortress）。
真源是 URDF；这里只做 Fortress 化 + 插入我们的 MID-360 传感器实例。生成时间相关的
内容一律不写死，免得多份副本漂移。</description>
</model>
""" % args.model_name)
    print(out_sdf)
    log('wrote %s (+ model.config)' % out_sdf)


if __name__ == '__main__':
    main()
