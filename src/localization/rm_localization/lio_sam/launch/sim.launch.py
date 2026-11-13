"""sim.launch.py —— 在本仓仿真里单独跑 LIO-SAM（3D 建图/回环），**不接管** map→odom。

为什么不放进 bringup_sim.launch.py：LIO-SAM 自带 `map→odom→base_link` 的 TF 发布，
而本仓契约是「`map→odom` 单一发布者 = AMCL/GICP」。所以第一阶段只把它当**3D 建图后端**：
本 launch 起两个节点，LIO-SAM 换成独立 frame 前缀不冲突……

为最小改动，这里仍用 map/odom（与 bringup_sim 的 mapping 模式同时跑时会**冲突**），
所以用法是：**先 `mode:=mapping mapper:=slam_toolbox lio:=small_point_lio` 把 2D/TF 跑起来**，
再单独起本 launch 时**只订阅**（本 launch 不发布 map→odom，见下）。

节点：
  1) livox_to_lio_sam：/livox/lidar (CustomMsg) → /lio_sam/points (Velodyne 格式 xyz/intensity/ring/time)
  2) lio_sam run：imageProjection / featureExtraction / mapOptimization / imuPreintegration
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    in_topic = LaunchConfiguration('in_topic')
    out_topic = LaunchConfiguration('out_topic')
    livox_z = LaunchConfiguration('livox_z')
    rviz = LaunchConfiguration('rviz')

    share = get_package_share_directory('lio_sam')

    # LIO-SAM 的 TF 一律改发到 /lio_sam/tf*：主栈（AMCL/GICP）保持 map→odom 单一发布者，
    # 这样"主栈 2D/TF + LIO-SAM 3D 建图"可以同时跑（见 README_SIM.md）。
    remaps = [('/tf', '/lio_sam/tf'), ('/tf_static', '/lio_sam/tf_static')]

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'params_sim.yaml')),
        DeclareLaunchArgument('in_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('out_topic', default_value='/lio_sam/points'),
        # ★ 2026-11-03：LIO-SAM 的 map 原点 = 它收到第一帧点云时的机器人位姿。
        #   spawn 时车会从 z=0.2 落到地面（~1-2 s），若这期间就启动 ⇒ 原点带 0.2 m 偏差。
        #   所以延迟 start_delay 秒再起（默认 6 s，覆盖 spawn 落定 + IMU 预热）。
        DeclareLaunchArgument('start_delay', default_value='6.0'),
        # base_link → livox_frame 的 z（URDF 常量；chassis 与 robot11 都是 0.09 m）
        DeclareLaunchArgument('livox_z', default_value='0.09'),
        # ★ 2026-11-04：**3D 建图专用 RViz**（Fixed Frame = odom，显示
        #   /lio_sam/mapping/map_global + cloud_registered + 轨迹）。
        #   为什么单独一个：主栈的 nav2.rviz 是给 2D 导航用的，混在一起很难判断"3D 图好不好"。
        DeclareLaunchArgument('rviz', default_value='false'),

        # ①b ★ 2026-11-03：把**机器人静态 TF** 也发到私有树 /lio_sam/tf_static。
        #   为什么必须：本 launch 把 LIO-SAM 的 /tf、/tf_static 全部重映射到 /lio_sam/tf*（为了不抢
        #   主栈 map→odom）。但 remap 对**订阅**同样生效 ⇒ LIO-SAM 的 imuPreintegration 在私有树上
        #   查不到 livox_frame（实测刷屏 `"livox_frame" passed to lookupTransform argument
        #   target_frame does not exist.`）。给它喂一条 base_link→livox_frame 即可自洽。
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='lio_sam_robot_static_tf', output='screen',
             arguments=['--x', '0', '--y', '0', '--z', livox_z,
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
             remappings=[('/tf_static', '/lio_sam/tf_static')]),

        # ① Livox CustomMsg → LIO-SAM 期望的 Velodyne 格式
        Node(package='cloud_accumulator', executable='livox_to_lio_sam',
             name='livox_to_lio_sam', output='screen',
             parameters=[{'use_sim_time': True,
                          'in_topic': in_topic,
                          'out_topic': out_topic}]),

        # ② LIO-SAM 本体（延迟启动，见 start_delay 说明）
        TimerAction(period=LaunchConfiguration('start_delay'), actions=[
        Node(package='lio_sam', executable='lio_sam_imuPreintegration',
             name='lio_sam_imuPreintegration', output='screen', parameters=[params],
             remappings=remaps),
        Node(package='lio_sam', executable='lio_sam_imageProjection',
             name='lio_sam_imageProjection', output='screen', parameters=[params],
             remappings=remaps),
        Node(package='lio_sam', executable='lio_sam_featureExtraction',
             name='lio_sam_featureExtraction', output='screen', parameters=[params],
             remappings=remaps),
        Node(package='lio_sam', executable='lio_sam_mapOptimization',
             name='lio_sam_mapOptimization', output='screen', parameters=[params],
             remappings=remaps),
        ]),

        # ② 启动前置门：没有 /clock 或 /livox/imu 就直说，别让 GTSAM 抛
        #    `IndeterminantLinearSystemException`（用户 2026-11-04 单独跑本 launch 时撞到的）
        Node(package='rm_utils', executable='topic_gate.py', name='lio_sam_topic_gate',
             output='screen',
             parameters=[{'topics': '/clock:rosgraph_msgs/msg/Clock,'
                                    '/livox/imu:sensor_msgs/msg/Imu,'
                                    '/livox/lidar:livox_ros_driver2/msg/CustomMsg',
                          'timeout_s': 25.0, 'strict': False}]),

        # ③ 可选的 3D 建图 RViz（默认关）
        Node(package='rviz2', executable='rviz2', name='rviz2_lio_sam_map',
             output='log',
             arguments=['-d', os.path.join(share, 'config', 'lio_sam_map.rviz'),
                        '-f', 'odom'],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(rviz)),
    ])
