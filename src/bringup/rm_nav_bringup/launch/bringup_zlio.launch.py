"""T-DT 全套 + z-localization 定位（替代我们的 LIO/AMCL 链）。

用法（两个工作空间都要 source）：
  source install/setup.bash
  source .tmp_cache/zlio_ws/install/setup.bash
  ros2 launch rm_nav_bringup bringup_zlio.launch.py gui:=True

链路：
  livox + imu → zlio 前端/后端 → map 系位姿 → rm_zlio_bridge → TF map→base_link
  规划：/map 先验图 → rm_tdt_planner（A*+MinSnap）→ /trajectory_plan
  局部：rm_tdt_local_controller（4 m Kastar）→ /cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _require_prior_grid(context, *args, **kwargs):
    """★ 先验栅格图必须存在：缺了就让 launch 立刻失败，而不是悄悄回落到旧地图。"""
    import os
    from launch.actions import LogInfo
    from launch.substitutions import LaunchConfiguration
    path = LaunchConfiguration('map_yaml').perform(context)
    if not path or not os.path.exists(path):
        raise RuntimeError(
            '缺少先验栅格图: %r\n'
            '请先生成：\n'
            '  /usr/bin/python3 tools/pcd_to_grid_map.py \\\n'
            '    --pcd .tmp_cache/prior/runtime_map/map.pcd \\\n'
            '    --out .tmp_cache/prior/runtime_map/grid\n'
            '（这条链不再使用 RMUC2027_v2.pgm）' % path)
    return [LogInfo(msg='[zlio] 使用先验栅格图: ' + path)]


def generate_launch_description():
    args = {
        'world': 'RMUC2027',
        # ★ 2026-11-08：这条链**不再使用**手画/官方的 2D 图。
        #   全局代价地图、规划器、目标点全部改用「先验 map.pcd 投影出来的 grid.pgm」，
        #   与 zlio 定位同源同帧（出生点 = (0,0)）。
        #   map_name 仅作为 sim launch 的内部占位（不参与这条链的地图加载）。
        'map_name': 'RMUC2027_v2',
        'planner': 'tdt',
        'nav': 'rm_tdt_local',
        'robot': 'chassis',
        'gui': 'True',
        'use_sim_time': 'true',
        'imu_acceleration_scale': '1.0',
        # ★★★ 2026-11-11：**zlio 发布点云的体素去重尺度**（zlio_node 参数 `output_resolution`）。
        #   源码 preparation.hpp:210 `if (output_resolution_ > 0) { 按该体素去重后发布 }`，
        #   注释原文：「设置后端点云采样尺度，**不影响在线滤波及入图采样**」。
        #   ⇒ 这是**只影响发布**的抽稀，等价于 BIT 的 `dense_publish_en`
        #      （BIT 为建图另发一条 `/cloud_registered_full`，我们这里调小即可）。
        #   默认 0.10 ⇒ 实测 2140 点/帧（不够投影层判"薄面 vs 墙"）；
        #   调到 0.02 ⇒ 接近"预处理后全量"，且不拖慢 LIO。
        #   ⚠ 不能设 0：源码是 `> 0` 才填充输出数组，设 0 会发布**空点云**。
        'output_resolution': '0.02',
        # ★ 2026-11-08：定位用的先验（zlio 自己建的 bundle）
        # 先验图缓存目录（仓库根下）。用本文件位置推导 ⇒ 仓库搬走也不断
        'prior_map': os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                  '..', '..', '..', '..', '.tmp_cache', 'prior', 'runtime_map'),
        # ★ 2026-11-08：仿真专用后端配置（uwb 关、body 外参单位阵）
        # 工具目录不在 install 里 ⇒ 用本文件位置推仓库根（source 与 install 下都成立）
        'backend_config': os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                       '..', '..', '..', '..', 'tools', 'configs',
                                       'zlio_backend_mapping_save.yaml'),
        # ★ 逐点帧内时间：运动时必须开，否则加速度大时跟踪会丢
        'livox_intra_frame_time': '1',
        # ★ 2026-11-08：默认起 nav2 的 RViz（要关就 nav_rviz:=False）
        'nav_rviz': 'True',
        # ★ 2026-11-08：**用 zlio 先验生成的地图**（map.pcd → pgm），与定位同源同帧。
        #   旧的 RMUC2027_v2.pgm 是另一份图，和先验对不上 ⇒ 全局/局部两张图不一致。
        # ★★ 2026-11-08（实测）：**默认用旧的那张手画图**（看起来更规整），
        #   它和 zlio 的先验在几何上是对齐的（ICP 中位 4.7 cm），偏移 = (0,0)。
        #   想改用「先验投影图」：map_yaml:=.tmp_cache/prior/runtime_map/grid.yaml
        # ★ 2026-11-13：默认图改为 **map/RMUC2027_cad.yaml**（CAD 点云生成 + 人工 P 过的那张，
        #   墙已按"薄处加粗 1 格"处理过；origin -20.40/-9.62 与仿真世界对齐）。
        #   想回到老图：map_yaml:=.../RMUC2027_v2_thick1.yaml
        # ★ 2026-11-08：旧默认（**墙加粗 1 格**的图 RMUC2027_v2_thick1）——
        #   原图墙只有 2 格(10cm)厚，正好等于 T-DT 非严格碰撞检测的 2 格容差 ⇒ 平滑轨迹可合法穿墙。
        #   加粗 1 格后厚度 2.8 格，容差不再是整堵墙。原图仍保留，可用 map_yaml:=...RMUC2027_v2.yaml 切回。
        # ★ 2026-11-13：改成**包内相对**（FindPackageShare），仓库搬去哪都对
        'map_yaml': PathJoinSubstitution([FindPackageShare('rm_nav_bringup'), 'map', 'RMUC2027_cad.yaml']),
        # ★ 2026-11-09：动态障碍层实现
        #   mask    = mask_builder（z 带一刀切，现状）
        #   terrain = terrain_layer（3D 体素 + 射线清空 + 衰减 + 逐柱语义：坡可走、横杆下可钻）
        'mask_impl': 'mask',
        'robot_height': '0.45',
        # ★ 2026-11-11：**分层单测开关**。默认 all = 四层状态一起打印。
        #   none / all / l0 / l1 / l2 / l3 / 逗号组合（如 l0,l3）
        #   每层状态来自 /nav/l{0..3}/status（rm_nav_interfaces/StageStatus），
        #   某层 FAILED 时监视器会立刻以 error 级别打一条，指明"是哪一层坏了"。
        'stage_test': 'all',
        'stage_period': '2.0',
        'mask_input_topic': '/localization/lio_world_cloud',
        # ★★★ 2026-11-11：局部代价图的障碍来源，**透传给 bringup_sim**。
        #   不补这一项时，run_sim.sh 传 local_obstacle:=stvl 会被判为“未使用的参数”。
        #   scan | cloud | both | stvl | stvl_both（含义见 bringup_sim 的同名声明）
        'local_obstacle': 'stvl',
        # ★ 2026-11-11（实测后定案）：**默认打开**我方适配器。
        #   曾尝试让 zlio 自己发 `odom → imu_link` 以去掉这个节点，但后端
        #   校验帧名不通过（见下方注释）⇒ 原生路线否决，适配器是唯一可行方案。
        'use_zlio_tf_adapter': 'true',
        # ★ 2026-11-12：`world_vel_to_body` 是 BIT(世界系 MPC) 专用；
        #   T-DT 控制器直接发机体系 ⇒ **必须关掉**，否则它会持续向
        #   `/cmd_vel_chassis` 发零速，与控制器抢同一条话题（实测：车不动）。
        'use_world_vel_to_body': 'false',
    }
    decls = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]

    # 仿真进程（gzserver）需要这个环境变量：逐点帧内时间
    intra_frame_env = SetEnvironmentVariable(
        'LIVOX_INTRA_FRAME_TIME', LaunchConfiguration('livox_intra_frame_time'))

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rm_nav_bringup'), 'launch', 'bringup_sim.launch.py')),
        launch_arguments={
            'sim_backend': 'classic',
            'world': LaunchConfiguration('world'),
            'map_name': LaunchConfiguration('map_name'),
            'map_yaml': LaunchConfiguration('map_yaml'),
            'mode': 'nav',
            # ★ 关掉我们的 LIO 与 relocalizer：TF map→base 交给 zlio_bridge
            'lio': 'none',
            # ★ 2026-11-08：**不能**传 localization:='' —— 传空串会让 nav2 整栈不启动
            #   （实测：传空 ⇒ planner/controller/bt_navigator 都不起；不传 ⇒ 正常起 5 个节点）
            'planner': LaunchConfiguration('planner'),
            'nav': LaunchConfiguration('nav'),
            'mask_impl': LaunchConfiguration('mask_impl'),
            'robot_height': LaunchConfiguration('robot_height'),
            'mask_input_topic': LaunchConfiguration('mask_input_topic'),
            'local_obstacle': LaunchConfiguration('local_obstacle'),
            'use_zlio_tf_adapter': LaunchConfiguration('use_zlio_tf_adapter'),
            'use_world_vel_to_body': LaunchConfiguration('use_world_vel_to_body'),
            'robot': LaunchConfiguration('robot'),
            'ground': 'patchwork',
            'local_frame': 'map',
            'spin_speed': '0.0',
            'gui': LaunchConfiguration('gui'),
            'nav_rviz': LaunchConfiguration('nav_rviz'),
        }.items(),
    )

    # 分层状态监视器（stage_test:=none 时完全不启动）
    stage_ok = PythonExpression([
        "'", LaunchConfiguration('stage_test'), "' != 'none' and '",
        LaunchConfiguration('stage_test'), "' != ''"
    ])
    stage_monitor = Node(
        package='rm_tdt_planner',
        executable='nav_stage_monitor.py',
        name='nav_stage_monitor',
        output='screen',
        condition=IfCondition(stage_ok),
        parameters=[{
            'stages': LaunchConfiguration('stage_test'),
            'period': LaunchConfiguration('stage_period'),
        }],
    )

    # IMU 预处理：重力对齐 + 补协方差（仿真 IMU 协方差全 0，会让滤波器奇异）
    imu_prep = Node(
        package='rm_zlio_bridge',
        executable='imu_prep',
        name='imu_prep',
        output='screen',
        parameters=[{
            'align_gravity': True,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # 前端（★ 队列必须有界：开大会让后验落后好几秒 ⇒ 高频预测一路漂）
    frontend = Node(
        package='frontend',
        executable='zlio_node',
        name='zlio_node',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([FindPackageShare('frontend'), 'config', 'lio.yaml']),
            'input_type': 'livox',
            'lidar_topic': '/livox/lidar',
            'imu_topic': '/livox/imu_level',
            # ★★★ 2026-11-11（**已实测否决**）：**不要改 body_frame**。
            #   试过 `body_frame: imu_link`（只改前端、后端保持 `body`），
            #   结果后端仍报：
            #     `invalid ZLIO observation frame/time/point layout`
            #   ⇒ 后端会**校验前端公布的帧名**，不认 imu_link。
            #   ⇒ TF 仍由我方的 zlio_tf_adapter 补（use_zlio_tf_adapter 默认 true）。
            #   注：zlio 自己那条 `odom → imu` 是悬空边（URDF 里没有 imu 这个 link），
            #   它存在但不碍事——我们只需要 `odom → base_link` 这条边。
            'imu_acceleration_scale': LaunchConfiguration('imu_acceleration_scale'),
            'output_resolution': LaunchConfiguration('output_resolution'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'reset_mode': 'warm',
            'coordinated_reset': True,
            # ★ 放宽输入队列（本机 CPU 紧张时默认值会溢出）
            # ★ 队列必须给大：他们前端一旦溢出会把 InputQueue 置为 Failed 并**永久丢弃**后续输入
            #   （线程栈显示全部阻塞在 futex/DDS 等待，看起来像"卡死"）。实测 200000 后不再溢出。
            'maximum_queue_events': 8192,
            'maximum_buffered_points': 2000000,
            'imu_history_seconds': 600.0,
            'maximum_imu_interval': 0.5,
        }],
    )

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('backend'), 'launch', 'backend.launch.py'])),
        launch_arguments={
            'config_file': LaunchConfiguration('backend_config'),
            'mode': 'localization',
            'prior_map': LaunchConfiguration('prior_map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': 'false',
        }.items(),
    )

    # ★★ 2026-11-08（实测二分）：**zlio 节点必须在 nav2 起来之后再启动**。
    #   同一 launch 里先加 zlio_node（哪怕它立刻失败退出）会让 nav2 整栈不启动（实测 nav2=0，
    #   去掉前端后 nav2=3~5）。用 TimerAction 延后 60 s 启动 zlio 三个部件即可共存。
    # ★ 2026-11-08：位姿由 zlio 后端**自己发** `map→base_link`（output.legacy + body.frame=base_link），
    #   不再需要自建 TF 桥/静态对齐（已删除）。
    zlio_stack = TimerAction(period=60.0, actions=[imu_prep, frontend, backend])

    from launch.actions import OpaqueFunction
    guard = OpaqueFunction(function=_require_prior_grid)

    return LaunchDescription([*decls, guard, intra_frame_env, sim, stage_monitor, zlio_stack])
