#!/usr/bin/env python3
"""make_world_gz_sdf.py —— 把 gz 世界模板**渲染成具体一次运行**的世界文件。

为什么需要：`worlds/*.sdf` 里的机器人 `<include>` 是写死的
（`<uri>model://rm_robot11_gz</uri>` + 固定出生位姿）。`sim_backend:=gz` 现在支持
`robot:=robot11|chassis`，且不同场地出生点不同 ⇒ 必须在运行时替换这三行。

做法：只在 XML 结构上操作（不碰其余字节），替换
    · 第一个 `<include>` 里 uri 匹配 `model://rm_*_gz` 的（= 机器人）
        → `<uri>model://<robot-model></uri>`、`<name><robot-name></name>`、`<pose>`
    · `--robot-model none` 时**删掉**该 include（`robot:=none` 的台架用法）

用法：
  python3 make_world_gz_sdf.py --in worlds/rmuc2026_gz.sdf --out /tmp/w.sdf \
      --robot-model rm_chassis_gz --robot-name chassis --robot-pose "5.9 2.72 0.1025 0 0 0"
  # 不传 --robot-model ⇒ 原样复制（用于 robot11 + RMUC2026 的"输出逐字节相同"回归）
"""
from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='src', required=True)
    ap.add_argument('--out', dest='dst', required=True)
    ap.add_argument('--robot-model', default='',
                    help='机器人 model 名（如 rm_chassis_gz）；空 = 原样复制；none = 删掉机器人 include')
    ap.add_argument('--robot-name', default='',
                    help='`<name>`（如 chassis）；空 = 用 robot-model 去掉 rm_/_gz')
    ap.add_argument('--with-sensors', action='store_true',
                    help='插入 ignition-gazebo-sensors-system（gz_sensor:=native 的 gpu_lidar 必需；'
                         '默认关 ⇒ 模板/渲染结果与之前逐字节相同）')
    ap.add_argument('--robot-pose', default='',
                    help='出生位姿 "x y z r p y"；空 = 保持模板里的值')
    a = ap.parse_args()

    text = open(a.src, encoding='utf-8').read()
    if not a.robot_model and not a.with_sensors:
        open(a.dst, 'w', encoding='utf-8').write(text)
        print('[make_world_gz_sdf] 原样复制：%s → %s' % (a.src, a.dst))
        return 0

    root = ET.fromstring(text)
    world = root.find('world')
    if world is None:
        sys.exit('世界文件里没有 <world>：%s' % a.src)

    # ★ 2026-11-03（goal 轮 12）：native 模式必须补 Sensors 系统 ——
    #   本仓的世界模板是给"进程内 mid360 插件"用的（插件自己用 rmagine/embree 打射线），
    #   **没有** ignition-gazebo-sensors-system；gz 原生 gpu_lidar 没它就不出点云（实测：话题在、无数据）。
    if a.with_sensors:
        has = any('sensors-system' in (pl.get('filename') or '')
                  for pl in world.findall('plugin'))
        if not has:
            pl = ET.Element('plugin', {
                'filename': 'ignition-gazebo-sensors-system',
                'name': 'ignition::gazebo::systems::Sensors'})
            ET.SubElement(pl, 'render_engine').text = 'ogre2'
            world.insert(0, pl)
            print('[make_world_gz_sdf] 已插入 ignition-gazebo-sensors-system（render_engine=ogre2）')
        else:
            print('[make_world_gz_sdf] 世界已有 sensors-system，跳过插入')

    # native 模式不需要 rmagine/embree 的"自机排除"地图系统（那是给进程内 mid360 插件的，
    # 没有第三方后端时会刷 "Failed to load system plugin"）⇒ 摘掉。
    if a.with_sensors:
        for _pl in list(world.findall('plugin')):
            _fn = (_pl.get('filename') or '')
            if 'rmagine' in _fn or 'embree' in _fn:
                world.remove(_pl)
                print('[make_world_gz_sdf] 摘掉 %s（native 用不到）' % _fn)

    robot_inc = None
    for inc in world.findall('include'):
        uri = (inc.findtext('uri') or '').strip()
        if uri.startswith('model://rm_') and uri.endswith('_gz'):
            robot_inc = inc
            break
    if robot_inc is None and a.robot_model != 'none':
        sys.exit('世界模板里找不到机器人 <include>（uri 形如 model://rm_*_gz）：%s' % a.src)

    if a.robot_model == 'none':
        if robot_inc is not None:
            world.remove(robot_inc)
            print('[make_world_gz_sdf] robot:=none ⇒ 删掉机器人 include')
    else:
        name = a.robot_name
        if not name:
            name = a.robot_model
            name = name[3:] if name.startswith('rm_') else name
            name = name[:-3] if name.endswith('_gz') else name
        uri = robot_inc.find('uri')
        if uri is None:
            uri = ET.SubElement(robot_inc, 'uri')
        uri.text = 'model://%s' % a.robot_model
        nm = robot_inc.find('name')
        if nm is None:
            nm = ET.SubElement(robot_inc, 'name')
        nm.text = name
        if a.robot_pose:
            p = a.robot_pose.split()
            if len(p) not in (3, 6):
                sys.exit('--robot-pose 需要 3 或 6 个数，收到 %r' % a.robot_pose)
            pose = robot_inc.find('pose')
            if pose is None:
                pose = ET.SubElement(robot_inc, 'pose')
            pose.text = ' '.join(p)
        print('[make_world_gz_sdf] 机器人 include → model://%s / name=%s / pose=%s'
              % (a.robot_model, name, a.robot_pose or '(模板值)'))

    # ★ 实验开关（GZ_PHYS_GROUND=1）：给物理世界加一块**解析地面**（plane），
    #   用来替代"轮子-三角网格"接触。动机：cylinder-vs-trimesh 接触在 ODE 里很差，
    #   实测轮子打滑 2.5~3 倍；解析 plane 的接触稳定得多。视觉仍用原 mesh。
    if os.environ.get('GZ_PHYS_GROUND', '0') == '1':
        z = float(os.environ.get('GZ_PHYS_GROUND_Z', '0.059'))
        gm = ET.Element('model', {'name': 'phys_ground'})
        ET.SubElement(gm, 'static').text = 'true'
        lk = ET.SubElement(gm, 'link', {'name': 'ground_link'})
        c = ET.SubElement(lk, 'collision', {'name': 'ground_collision'})
        ET.SubElement(c, 'pose').text = '0 0 %.4f 0 0 0' % z
        g = ET.SubElement(c, 'geometry')
        pl = ET.SubElement(g, 'plane')
        ET.SubElement(pl, 'normal').text = '0 0 1'
        surf = ET.SubElement(c, 'surface')
        fr = ET.SubElement(surf, 'friction')
        ode = ET.SubElement(fr, 'ode')
        ET.SubElement(ode, 'mu').text = '1.5'
        ET.SubElement(ode, 'mu2').text = '1.5'
        # ⚠️ 必须挂到 <world> 里；挂到 <sdf> 根上 gz 会**忽略**（实测：模型直接自由落体）
        _w = root.find('world')
        (_w if _w is not None else root).append(gm)
        print('[make_world_gz_sdf] 加解析地面 plane z=%.3f（GZ_PHYS_GROUND=1）' % z)

    ET.indent(root, space='  ')
    open(a.dst, 'w', encoding='utf-8').write(
        '<?xml version="1.0" ?>\n' + ET.tostring(root, encoding='unicode') + '\n')
    print('[make_world_gz_sdf] 写出 %s' % a.dst)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
