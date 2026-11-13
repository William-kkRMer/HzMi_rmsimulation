#!/usr/bin/env python3
"""三套一键模式：`tdt` / `bit` / `std`（原版 nav2）。

为什么要有它：`bringup_zlio.launch.py` 有 135 个参数，切"用哪套规划+控制"
要同时改 `planner` / `nav` / `mask_impl` / `local_obstacle` 四个，容易漏。
这里把三套**已经定义好**的组合各包一层，只暴露常用的几个覆盖项。

    ros2 launch rm_nav_bringup bringup_mode.launch.py mode:=tdt
    ros2 launch rm_nav_bringup bringup_mode.launch.py mode:=bit
    ros2 launch rm_nav_bringup bringup_mode.launch.py mode:=std

    # 覆盖常用项（不给就用各 launch 的默认）
    ros2 launch rm_nav_bringup bringup_mode.launch.py mode:=tdt \\
        gui:=false map_yaml:=/abs/path/map.yaml stage_test:=none

| mode | planner | nav（控制器） | mask_impl | local_obstacle | 状态 |
|------|---------|---------------|-----------|----------------|------|
| `tdt`  | `tdt`   | `rm_tdt_local` | `costmap` | `stvl` | **主线，已验证** |
| `bit`  | `bit`   | `bit`          | `costmap` | `stvl` | **实验**：BIT 的 MINCO 插件对（见下方注意） |
| `std`  | `navfn` | `dwb`          | `costmap` | `stvl` | 原版 nav2，作对照基准 |

实现要点：
  · 只在**用户显式给了**参数时才透传（空字符串 = 不传，用下游默认），避免把 '' 覆盖下去；
  · 四个槽位参数（planner/nav/mask_impl/local_obstacle）**用户覆盖优先**于模式预设；
  · `bit` 模式额外起两条 `topic_tools/relay`：BIT 控制器订阅 `/aft_mapped_to_init`
    并发 `/cmd_vel_mpc`，而我们的里程计是 `/localization/odometry`、底盘吃 `/cmd_vel`。
"""
from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

_PRESETS = {
    'tdt': {'planner': 'tdt', 'nav': 'rm_tdt_local', 'mask_impl': 'costmap',
            'local_obstacle': 'stvl'},
    'bit': {'planner': 'bit', 'nav': 'bit', 'mask_impl': 'costmap',
            'local_obstacle': 'stvl'},
    'std': {'planner': 'navfn', 'nav': 'dwb', 'mask_impl': 'costmap',
            'local_obstacle': 'stvl'},
}

# 允许透传给 bringup_zlio 的参数（空值 = 不传）
_PASSTHROUGH = ('world', 'map_name', 'map_yaml', 'gui', 'stage_test', 'stage_period',
                'backend_config', 'robot', 'use_sim_time', 'nav_rviz')
# 四个槽位：模式预设 + 用户覆盖优先
_SLOTS = ('planner', 'nav', 'mask_impl', 'local_obstacle')


def _launch(context):
    mode = context.launch_configurations.get('mode', 'tdt').strip() or 'tdt'
    if mode not in _PRESETS:
        raise RuntimeError(
            "[bringup_mode] mode:=%r 不是可用取值（%s）"
            % (mode, ' | '.join(_PRESETS)))

    args = {}
    for k in _PASSTHROUGH:
        v = str(context.launch_configurations.get(k, '')).strip()
        if v:
            args[k] = v
    for k in _SLOTS:
        v = str(context.launch_configurations.get(k, '')).strip()
        args[k] = v or _PRESETS[mode][k]

    inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rm_nav_bringup'),
                         'launch', 'bringup_zlio.launch.py')),
        launch_arguments=args.items(),
    )

    actions = [inc]

    # BIT 实验模式：两条话题中继（不改 BIT 源码，也不改 bringup）
    # ★ 2026-11-13：本机**没有装 topic_tools**（实测 `ros2 pkg list` 无此包），
    #   所以改用仓库自带的 tools/scripts/nav/topic_relay.py。
    if mode == 'bit':
        # 用绝对路径（launch 里拿不到仓库根；这条路径是本机固定布局）
        # 用本文件位置推仓库根（source 与 symlink-install 下都成立），不再写死绝对路径
        relay_py = os.path.normpath(os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            '..', '..', '..', '..', 'tools', 'scripts', 'nav', 'topic_relay.py'))
        for typ, src, dst in (('odom', '/localization/odometry', '/aft_mapped_to_init'),
                              ('twist', '/cmd_vel_mpc', '/cmd_vel')):
            actions.append(Node(
                executable=sys.executable,
                arguments=[relay_py, '--type', typ, '--src', src, '--dst', dst],
                name='relay_bit_' + dst.strip('/').replace('/', '_'),
                output='screen',
                parameters=[{'use_sim_time': True}],
            ))
    return actions


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'mode', default_value='tdt',
        description='一键模式：tdt（T-DT 主线，默认）| bit（BIT MINCO 插件对，实验）| std（原版 nav2）'))
    # ★★★ 2026-11-13（**踩过的坑**）：**不要在这里声明** map_yaml / gui / stage_test 等
    #   透传参数。launch 的参数是"先声明者生效"：外层一旦用 default_value='' 声明了
    #   `map_yaml`，子 launch（bringup_zlio）的 DeclareLaunchArgument 就被忽略，
    #   于是它拿到空串 ⇒ 报"缺少先验栅格图: ''"（实测踩到，整个仿真没起来）。
    #   现在：只在用户**显式**给了这些参数时才透传（显式给的值本来就在 launch 上下文里，
    #   即使本文件没声明也能读到）；没给就完全交给子 launch 自己的默认值。
    ld.add_action(OpaqueFunction(function=_launch))
    return ld
