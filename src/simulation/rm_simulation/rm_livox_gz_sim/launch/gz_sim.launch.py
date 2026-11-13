# =============================================================================
# gz_sim.launch.py —— `sim_backend:=gz` 的 opt-in 入口（Gazebo Fortress）
# =============================================================================
# 它做四件事，别的一概不碰（感知/LIO/nav2 仍由 bringup_sim.launch.py 起，与 Classic 相同）：
#   1. `ign gazebo` 起 Fortress（世界 = 我们的 SDF 1.9；GUI 可选；HOME 自动重定向）；
#   2. ros_gz_bridge 桥 **/clock**（这是唯一必须桥的）；
#   3. robot_state_publisher + joint_state_publisher（TF 契约与 Classic 逐字相同）；
#   3b. **每次运行独占一个 gz transport partition**（GZ_PARTITION/IGN_PARTITION）。
#       不隔离的后果是实测过的（docs/fortress_sim_backend.md §8.2）：同机另一个
#       Fortress 世界会把自己的 /clock 与 /mid360/imu_gz 送进我们的插件
#       ⇒ /livox/imu 的戳变成**两个仿真钟交错** ⇒ small_point_lio 每秒每条都报
#       `imu loop back`、永不初始化（既无 odom→base_link 也无 /cloud_registered）。
#   4. 运行期把 robot 的 URDF 派生成 Fortress 的 SDF 模型（scripts/make_robot_gz_sdf.py），
#      并把 sensor 模型里的 @MID360_CSV@ 换成仓库里那份 17 MB 的 mid360.csv 真路径。
#
# 单独用（不开感知链）：
#   ros2 launch rm_livox_gz_sim gz_sim.launch.py world:=RMUC2026 gui:=False
#   ros2 launch rm_livox_gz_sim gz_sim.launch.py gz_world:=bench gz_robot:=none gui:=False
#
# 通过 bringup（推荐，整栈 + 端到端）：
#   ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz \
#       world:=RMUC2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 \
#       gui:=False spin_speed:=0.0
#   （★ 2026-10-27：`robot11_mount` 已删除 —— 安装方式固定 = xacro 默认的 `sensor`。）
# =============================================================================
import os
import subprocess
import sys
import tempfile
import uuid
import shutil

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, AppendEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'rm_livox_gz_sim'

# world 槽位 -> 我们的 SDF（Classic 的 world 名到 gz 世界的映射，纯增量：未知值报错）
WORLD_MAP = {
    'RMUC2026': 'rmuc2026_gz.sdf',
    # ★ 2026-11-03（goal 轮 2）：RMUC2027 场地（模板里的机器人 include 由 make_world_gz_sdf.py
    #   在运行期替换成当次 robot 的 model/name/pose）。
    'RMUC2027': 'rmuc2027_gz.sdf',
    'RMUC2026_bench': 'rmuc2026_gz_bench.sdf',
}

#: 各场地的出生点 (x, y) —— 与 Classic 的 spawn_pose 一致（PCD/map 的 meta.yaml 里也记着）。
WORLD_SPAWN_XY = {
    'RMUC2026': (10.925, 2.525),
    'RMUC2027': (5.9, 2.72),
}

#: 各机器人的 base_link 离地高度（m）= URDF 里最低碰撞点到 base_link 原点的距离。
#: 写进世界 `<pose>` 的 z ⇒ 出生即"站好"，避免坠落污染 LIO 的重力初始化。
#:   robot11 = 0.1025（见 docs/tilted_lidar_fidelity.md §J/§K）
#:   chassis = 0.098  （轮 cylinder r=0.058、轮心 z=-0.04 ⇒ 最低点 -0.098；与 h(URDF)=0.188、
#:                       livox 相对 base_link z=0.09 自洽：0.098+0.09=0.188）
ROBOT_SPAWN_Z = {
    'robot11': 0.1025,
    'chassis': 0.098,
}

#: 各场地 mesh 的**可行驶地面高度**（m，世界系）—— 不是 0！
#: 实测 `RMUC2027_world/meshes/RMUC2027.stl`：出生点 (5.9, 2.72) 附近 1.5 m 内顶点 z 中位
#: **0.059 m**，全场 z<200mm 的顶点中位 **0.069 m**。
#: 不补这个偏移 ⇒ 车底盘按"地面=0"放置 ⇒ **轮子陷进场地 6 cm**（用户 2026-11-04 看到的就是这个），
#: 而且雷达贴地扫 ⇒ 点云异常 ⇒ LIO 里程计跑飞（RViz 里"动得特别快"）。
WORLD_FLOOR_Z = {
    'RMUC2027': 0.059,
    'RMUC2026': 0.0,
}

# 由 tools/scripts/gzsim/fetch_and_patch_mid360.sh 编出来的后端前缀
GZSIM_WS_DEFAULT = os.path.join(os.path.expanduser('~'), 'gzsim_ws')
# 本仓的 scratch 约定（用户按 fetch 脚本的默认值编到仓库内时走这条）
REPO_GZSIM_WS = None  # 运行期由 _repo_root() 算


def _repo_root():
    """从本包的 install/share 路径反推仓库根（…/install/rm_livox_gz_sim/share/…）。"""
    here = os.path.dirname(os.path.realpath(__file__))
    # share/rm_livox_gz_sim/launch/gz_sim.launch.py -> 上溯 3 层到 install/
    p = here
    for _ in range(2):
        p = os.path.dirname(p)
    # p = <prefix>/share ; prefix = <prefix>
    prefix = os.path.dirname(p)
    if os.path.basename(prefix) == 'install':
        return os.path.dirname(prefix)
    return None


def _find_backend_prefix(log):
    """找 gz 后端（rmagine + rmagine_gazebo_plugins + mid360_simulation_plugin_ros2）前缀。

    顺序：GZSIM_PREFIX 环境变量 -> ament 索引 -> 仓库内 scratch 默认位置。
    找不到就抛一条可操作的错误（不是 Python traceback）。
    """
    cands = []
    env = os.environ.get('GZSIM_PREFIX')
    if env:
        cands.append(env)
    for pkg in ('mid360_simulation_plugin_ros2', 'rmagine_gazebo_plugins'):
        try:
            share = get_package_share_directory(pkg)
            # <prefix>/share/<pkg> -> <prefix>
            cands.append(os.path.dirname(os.path.dirname(share)))
        except PackageNotFoundError:
            pass
    root = _repo_root()
    if root:
        cands.append(os.path.join(root, '.tmp_gzlivox2', 'gzsim_ws', 'install'))
    cands.append(GZSIM_WS_DEFAULT)
    for c in cands:
        if os.path.isfile(os.path.join(c, 'mid360_simulation_plugin_ros2', 'lib',
                                       'libmid360_system.so')):
            log('gz 后端前缀 = %s' % c)
            return c
    raise RuntimeError(
        '找不到 Gazebo Fortress 的 MID-360 后端（libmid360_system.so）。\n'
        '  先跑一次（一次就够，源码不进本仓库）：\n'
        '      tools/scripts/gzsim/fetch_and_patch_mid360.sh\n'
        '  或把已有构建指过来：export GZSIM_PREFIX=/path/to/gzsim_ws/install\n'
        '  当前找过：\n    ' + '\n    '.join(cands))


def _render_model(share, out_models, csv, qos, ignore_self, log):
    """把 models/rm_mid360_gz 渲染到运行期目录（替换 @MID360_CSV@/@QOS@）。"""
    src = os.path.join(share, 'models', 'rm_mid360_gz')
    dst = os.path.join(out_models, 'rm_mid360_gz')
    os.makedirs(dst, exist_ok=True)
    text = open(os.path.join(src, 'model.sdf')).read()
    text = (text.replace('@MID360_CSV@', csv).replace('@QOS@', qos)
                .replace('@IGNORE_SELF@', ignore_self))
    for tok in ('@MID360_CSV@', '@QOS@', '@IGNORE_SELF@'):
        if tok in text:
            raise RuntimeError('%s 没替换干净' % tok)
    open(os.path.join(dst, 'model.sdf'), 'w').write(text)

    # 世界文件里也引用了 model://rm_mid360_gz（台架世界），运行期这份优先（在
    # GZ_SIM_RESOURCE_PATH 最前面）。
    open(os.path.join(dst, 'model.config'), 'w').write(
        '<?xml version="1.0" ?>\n<model>\n  <name>rm_mid360_gz</name>\n'
        '  <version>1.0</version>\n  <sdf version="1.9">model.sdf</sdf>\n'
        '  <description>runtime-rendered by rm_livox_gz_sim/launch/gz_sim.launch.py'
        '</description>\n</model>\n')
    log('sensor 模型渲染到 %s（csv=%s qos=%s ignore_self=%s）'
        % (dst, csv, qos, ignore_self))


def _materialize(context, *args, **kwargs):
    log_lines = []

    def log(msg):
        log_lines.append(msg)
        print('[gz_sim.launch] ' + msg, file=sys.stderr)

    share = get_package_share_directory(PKG)
    # ★ 2026-11-03（goal 轮 12）：**先**决定传感器来源，再决定要不要第三方后端。
    #   native 模式（gz 原生 gpu_lidar）**不需要** libmid360_system.so，
    #   原来无条件先去 find 后端 ⇒ native 根本走不到。现在按模式分流。
    sensor_src = context.perform_substitution(LaunchConfiguration('gz_sensor')).strip() or 'mid360'
    if sensor_src not in ('mid360', 'native'):
        raise RuntimeError('gz_sensor 只能是 mid360 | native，收到 %r' % sensor_src)
    if sensor_src == 'mid360':
        backend = _find_backend_prefix(log)
    else:
        backend = ''
        log('gz_sensor:=native ⇒ 不需要第三方 libmid360_system.so（跳过后端查找）')

    world_sel = context.perform_substitution(LaunchConfiguration('gz_world'))
    world_arg = context.perform_substitution(LaunchConfiguration('world'))
    gui = context.perform_substitution(LaunchConfiguration('gui'))
    robot = context.perform_substitution(LaunchConfiguration('robot'))
    robot_desc = context.perform_substitution(LaunchConfiguration('robot_description'))
    qos = context.perform_substitution(LaunchConfiguration('gz_qos'))
    ignore_self = context.perform_substitution(LaunchConfiguration('gz_ignore_self'))
    verbose = context.perform_substitution(LaunchConfiguration('gz_verbose'))
    gz_home = context.perform_substitution(LaunchConfiguration('gz_home'))
    csv_override = context.perform_substitution(LaunchConfiguration('gz_csv'))
    runtime_dir = context.perform_substitution(LaunchConfiguration('gz_runtime_dir'))
    extra_guis = context.perform_substitution(LaunchConfiguration('gz_extra_gui'))
    partition = context.perform_substitution(LaunchConfiguration('gz_partition'))
    custom_msg_tag = context.perform_substitution(LaunchConfiguration('gz_custom_msg_tag'))
    drive = context.perform_substitution(LaunchConfiguration('gz_drive'))

    # ── 硬要求校验（缺了给可操作的报错，不静默跑错） ───────────────────────
    # ★ 2026-11-03（goal 轮 1）：支持 robot:=chassis。
    #   生成器本来就是"任意 URDF → SDF"的通用路径（`--urdf -` 读 stdin），
    #   所以只需要把 chassis 放进白名单 + 给它自己的 mesh 解析根（见下面 res_paths）。
    if robot not in ('', 'none', 'robot11', 'chassis'):
        raise RuntimeError(
            'sim_backend:=gz 目前只支持 robot:=robot11 | chassis（其余槽位在 gz 侧没有 '
            'Fortress 化的模型生成路径）。收到 robot=%r。' % robot)
    # ★ 2026-10-27（契约 §4.15）：`robot11_mount` 已删除。安装方式只剩一种（= 原来的
    #   `sensor` 语义）：URDF 关节 rpy=0 + 倾角在 `livox_frame→imu_link` + 插件把点旋进
    #   水平的 `livox_frame`；gz 侧传感器 link pose 仍由下面的 `--tilt-rpy` 钉成 −0.5236。
    #   ⇒ 这里不再校验 mount（它没有第二个取值可校验了）。

    # ── 世界文件 ─────────────────────────────────────────────────────────────
    if world_sel == 'bench' or world_arg not in WORLD_MAP:
        if world_sel == 'bench':
            world_file = os.path.join(share, 'worlds', 'rmuc2026_gz_bench.sdf')
        else:
            raise RuntimeError(
                'sim_backend:=gz 目前只支持 world:=RMUC2026 | RMUC2027（可选 gz_world:=bench '
                '起台架世界）。收到 world=%r；其余场地在 gz 侧还没有 SDF 1.9 资产 —— '
                '加一份即可（照 worlds/rmuc2027_gz.sdf 的模板），见 docs/fortress_sim_backend.md。'
                % world_arg)
    else:
        world_file = os.path.join(share, 'worlds', WORLD_MAP[world_arg])

    # ── 运行期目录 ───────────────────────────────────────────────────────────
    if not runtime_dir:
        runtime_dir = os.path.join(tempfile.gettempdir(),
                                   'rm_livox_gz_sim_%s' % uuid.uuid4().hex[:8])
    models_dir = os.path.join(runtime_dir, 'models')
    os.makedirs(models_dir, exist_ok=True)

    # ── HOME 重定向（这条是硬坑：~/.ignition 只读会让 GUI abort 并 SIGKILL server）
    if not gz_home:
        gz_home = os.path.join(runtime_dir, 'home')
    for sub in ('', '.ignition', '.cache', '.config', '.local/share', '.runtime'):
        os.makedirs(os.path.join(gz_home, sub), exist_ok=True)
    os.chmod(os.path.join(gz_home, '.runtime'), 0o700)
    log('HOME（仅 gz 进程）= %s' % gz_home)

    # ── CSV ─────────────────────────────────────────────────────────────────
    if csv_override:
        csv = csv_override
    else:
        # 真源 = Classic 插件用的那一份（不复制、不提交副本）
        try:
            livox_share = get_package_share_directory('ros2_livox_simulation')
            csv = os.path.join(livox_share, 'scan_mode', 'mid360.csv')
        except PackageNotFoundError:
            csv = ''
        if not csv or not os.path.isfile(csv):
            raise RuntimeError(
                '找不到 mid360.csv（Classic 插件那份）。先 colcon build 本仓并 source '
                'install/setup.bash，或用参数 gz_csv:=/abs/path/mid360.csv 指定。')
    log('mid360.csv = %s' % csv)

    _render_model(share, models_dir, csv, qos, ignore_self, log)

    # ── 机器人模型（从 robot_description 派生） ──────────────────────────────
    actions_extra_env = {}
    if robot not in ('', 'none'):
        if not robot_desc.strip():
            # 单独跑本 launch（不经 bringup）时自己跑一次 xacro：robot11 只有这一个模型文件。
            # ★ 2026-10-27（契约 §4.15）：**不再传 `livox_mount`** —— 安装方式只剩一种，且已烘进
            #   生成物 xacro（`<xacro:arg name="livox_mount" default="sensor"/>` + 常量 property）
            #   ⇒ 取默认值 = 与原来传 `livox_mount:=sensor|urdf` 逐字节相同的生成物。
            # ⚠️ 安装倾角**不在这里传**：它是生成物 xacro 的 `livox_tilt_rpy` 默认值，源头 =
            #   上游 URDF 的 body_to_livox.rpy（契约 §4.12；measurements §31）。这里再写一个字面值
            #   就是第二个所有者，上游换支架后这条独立路径会偷偷用旧角度。
            import subprocess
            try:
                urdf_dir = os.path.join(get_package_share_directory('rm_nav_bringup'), 'urdf')
            except PackageNotFoundError:
                raise RuntimeError(
                    'sim_backend:=gz 需要 robot_description（bringup 会传），且本机没装 '
                    'rm_nav_bringup ⇒ 无法自己跑 xacro。\n'
                    '  单独跑本 launch 时：加 robot:=none（只起台架世界）。')
            xacro_file = os.path.join(urdf_dir, 'sentry_robot_robot11_sim.xacro')
            cmd = ['xacro', xacro_file]
            log('robot_description 为空 ⇒ 自己跑：%s' % ' '.join(cmd))
            r = subprocess.run(cmd, capture_output=True, text=True)
            if r.returncode != 0:
                raise RuntimeError('xacro 失败：\n' + (r.stderr or r.stdout)[-3000:])
            robot_desc = r.stdout
        gen = os.path.join(share, 'scripts', 'make_robot_gz_sdf.py')
        import subprocess
        model_name = 'rm_%s_gz' % (robot or 'robot')
        cmd = [sys.executable, gen, '--urdf', '-', '--out-dir', models_dir,
               '--model-name', model_name, '--csv', csv, '--qos', qos,
               '--imu-frame', 'imu_link', '--ignore-self', ignore_self,
               '--custom-msg-tag', custom_msg_tag, '--drive', drive,
               '--sensor', sensor_src,
               # ★ 2026-11-04：雷达安装倾角（rad，rpy 空格分隔）。空 = 各模型默认。
            #   由 bringup 的 `lidar_tilt_rpy` 传进来；必须与 URDF 的雷达 origin 同值。
            # ★ 2026-11-04：`--motion visual` = dynamic + 无碰撞体 + 关重力
               #   ⇒ gz GUI 里车会跟着动（Fortress 6 不支持 <kinematic>，这是等效绕法）。
               #   环境变量 GZ_DRIVE_MOTION=static 可退回"最稳但 GUI 不动"。
               # ⚠️ 2026-11-04 实测：`physics`（真动态+保留碰撞）会与场地 mesh 接触求解发散
               #   ⇒ 168 条 NaN、车自己乱飞、LIO 被 NaN 打死（RViz 没点云就是这个）。
               #   ⇒ 默认回到 `static`（最稳）；`visual` 可用（位姿会动）但 gz GUI 不一定刷新；
               #     `physics` 保留但**不要**当默认。
               # ★ 2026-11-04 新增 `safe` 档并设为默认：**保留 collision**（GUI 当它是普通动态模型）
               #   但 `collide_bitmask=2` ⇒ 与场地（category=1）位与为 0 ⇒ 不做接触求解 ⇒ 不发散；
               #   每根 link 关重力 ⇒ 不掉。实测：NaN 0、/odom 91 Hz、位姿真在动（0.5 m/1 s）。
               '--motion', os.environ.get('GZ_DRIVE_MOTION', 'safe'),
               # ★ 2026-11-04：PolarBear(pb2025) 的做法 = SDF 里真轮子关节 + diff-drive 插件。
               #   `wheels` 走物理（GUI 会跟随），`planar` 仍是运动学 teleport（旧行为）。
               '--drive-mode', os.environ.get('GZ_DRIVE_MODE', 'planar')]
        # 30° 斜装：robot11 的生成物 xacro 已把 roll 烘进 `livox_frame→imu_link` 与插件
        # `<tilt_rpy>`（`base_link→livox_frame` rpy=0）。这里再显式钉一次，保证「不管 URDF
        # 怎么写，gz 侧的传感器 link pose 一定是 -0.5236」（B① 决策）。
        # ★ 2026-11-04：倾角不再只服务 robot11。优先级：
        #   launch 参数 gz_tilt_rpy > 环境变量 GZ_TILT_RPY > robot11 的 30° 默认 > 不倾斜。
        #   **必须与 URDF 的 lidar_mount_rpy 一致**（见 sentry_robot_chassis_sim.xacro 注释），
        #   否则 gz 的点云朝向与 TF/外参对不上。
        _tilt = (os.environ.get('GZ_TILT_RPY', '') or '').strip()
        _tilt_arg = (LaunchConfiguration('tilt_rpy').perform(context) or '').strip() if 'tilt_rpy' in context.launch_configurations else ''
        if _tilt_arg:
            _tilt = _tilt_arg
        if not _tilt and robot == 'robot11':
            _tilt = '-0.523598775598293 0 0'
        if _tilt:
            cmd += ['--tilt-rpy', _tilt]
        log('生成 %s 的 Fortress SDF：%s' % (robot, ' '.join(cmd)))
        log('驱动模型 motion = %s（safe = 保留碰撞体但不与场地碰 ⇒ GUI 正常 + 不发散；'
            'visual = 无碰撞体；static = GUI 不动；physics = 会发散）'
            % os.environ.get('GZ_DRIVE_MOTION', 'safe'))
        r = subprocess.run(cmd, input=robot_desc, capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError('make_robot_gz_sdf.py 失败：\n' + (r.stderr or r.stdout)[-3000:])
        for line in (r.stderr or '').strip().splitlines():
            log('  ' + line)
    else:
        model_name = None
        log('robot:=none ⇒ 不起机器人（台架世界，只有传感器）')

    # ── 世界渲染：把模板里的机器人 <include>（model/name/pose）换成**当次运行**的 ──────
    #  ★ 2026-11-03（goal 轮 2）：模板里写死 `model://rm_robot11_gz` + 出生位姿，
    #  现在 robot:=chassis / world:=RMUC2027 都要用同一份世界 ⇒ 渲染到 runtime_dir。
    #  渲染是"结构替换"，其余字节不碰；robot11+RMUC2026 的渲染结果与模板逐字节相同
    #  （`--robot-model` 为空时直接复制，见 make_world_gz_sdf.py）。
    _xy = WORLD_SPAWN_XY.get(world_arg)
    _fl = WORLD_FLOOR_Z.get(world_arg, 0.0)
    _z = None if ROBOT_SPAWN_Z.get(robot) is None else ROBOT_SPAWN_Z[robot] + _fl
    if model_name and _xy and _z is not None:
        render_py = os.path.join(share, 'scripts', 'make_world_gz_sdf.py')
        rendered = os.path.join(runtime_dir, 'world_rendered.sdf')
        rcmd = [sys.executable, render_py, '--in', world_file, '--out', rendered,
                '--robot-model', model_name, '--robot-name', (robot or 'robot'),
                '--robot-pose', '%s %s %s 0 0 0' % (_xy[0], _xy[1], _z)]
        # ★ native 模式：世界模板里没有 Sensors 系统（本仓模板是给进程内 mid360 插件用的）
        if sensor_src == 'native':
            rcmd.append('--with-sensors')
        log('渲染世界：%s' % ' '.join(rcmd))
        log('出生 z = 机体偏移 %.3f + 场地地面 %.3f = %.3f m' %
            (ROBOT_SPAWN_Z.get(robot, 0.0), _fl, _z))
        rr = subprocess.run(rcmd, capture_output=True, text=True)
        if rr.returncode != 0:
            raise RuntimeError('make_world_gz_sdf.py 失败：\n' + (rr.stderr or rr.stdout)[-3000:])
        for line in (rr.stdout or '').strip().splitlines():
            log('  ' + line)
        world_file = rendered
    else:
        # 没有出生点表/robot:=none ⇒ 用模板原样（`robot:=none` 时模板里的 include 会去加载
        # rm_robot11_gz 而该模型不存在 ⇒ 显式渲染成"无机器人"世界）
        if not model_name:
            render_py = os.path.join(share, 'scripts', 'make_world_gz_sdf.py')
            rendered = os.path.join(runtime_dir, 'world_rendered.sdf')
            rr = subprocess.run([sys.executable, render_py, '--in', world_file,
                                 '--out', rendered, '--robot-model', 'none'],
                                capture_output=True, text=True)
            if rr.returncode == 0:
                world_file = rendered

    # ── gz transport partition（**必须每次运行唯一**，理由见文件头 3b） ──────
    # 空 = 本 run 随机一个（默认，最安全）；指定 = 固定名（只有"故意要让两个世界
    # 共享 gz 话题"的对照实验才这么干）。
    if not partition:
        partition = 'rm_gz_%s' % uuid.uuid4().hex[:10]
    log('gz transport partition = %s（GZ_PARTITION / IGN_PARTITION 同时设）' % partition)

    # ── 环境变量（Fortress 两个世代的名字都设：本机实测两个都设才稳，见
    #    livox_sensor_prior_art.md §10.7） ────────────────────────────────────
    res_paths = [models_dir]
    # 场地 / 障碍物 / robot11 的 model:// 解析根（Classic 靠 GAZEBO_MODEL_PATH，这里显式给）
    try:
        _hzmi = get_package_share_directory('hzmi_rm_simulation')
        res_paths.append(os.path.join(_hzmi, 'meshes'))
        # ★ 2026-11-03（goal 轮 2）：场地 model（model://RMUC2026_world / RMUC2027_world）
        #   在 world/ 下，不在 meshes/ ⇒ 必须把 world/ 也放进解析根，否则 include 找不到。
        res_paths.append(os.path.join(_hzmi, 'world'))
    except PackageNotFoundError:
        pass
    # 机器人自身 mesh 的 model:// 解析根：按槽位取各自的 description 包
    # （robot11 → robot11；chassis → chassis_description）。两者都不在就跳过。
    _ROBOT_DESC_PKG = {'robot11': 'robot11', 'chassis': 'chassis_description'}
    _desc_pkg = _ROBOT_DESC_PKG.get(robot)
    if _desc_pkg:
        try:
            _desc_root = os.path.dirname(get_package_share_directory(_desc_pkg))
            res_paths.append(_desc_root)
            # ★ 2026-11-03（goal 轮 14）：**硬校验** mesh 真的在 —— 少了它 gz 里"看不到车模"
            #   而日志只有一行 Ogre "Cannot load null mesh"，非常容易被忽略。
            _mesh_dir = os.path.join(_desc_root, _desc_pkg, 'meshes')
            if not os.path.isdir(_mesh_dir):
                raise RuntimeError(
                    '%s 的 mesh 目录不存在：%s\n'
                    '  ⇒ gz 里会"看不到车模"（Ogre: Cannot load null mesh）。\n'
                    '  修：colcon build --symlink-install --packages-select %s\n'
                    '  或确认 install/%s/share/%s/meshes 里有 STL'
                    % (_desc_pkg, _mesh_dir, _desc_pkg, _desc_pkg, _desc_pkg))
            log('%s 的 mesh 根 = %s' % (_desc_pkg, _mesh_dir))
        except PackageNotFoundError:
            raise RuntimeError(
                '没装 %s ⇒ %s 的 mesh 在 gz 里解析不到（表现：看不到车模）。\n'
                '  修：colcon build --symlink-install --packages-select %s'
                % (_desc_pkg, robot, _desc_pkg))
    try:
        # 兼容旧行为：robot11 的包始终加进去（对 chassis 无害，只是多一条搜索路径）
        res_paths.append(os.path.dirname(get_package_share_directory('robot11')))
    except PackageNotFoundError:
        pass
    res_paths.append(os.path.join(share, 'models'))
    plugin_paths = []
    if backend:      # ★ native 模式没有第三方后端目录，别往里塞空路径
        plugin_paths = [
            os.path.join(backend, 'rmagine_gazebo_plugins', 'lib', 'rmagine_gazebo_plugins'),
            os.path.join(backend, 'mid360_simulation_plugin_ros2', 'lib'),
            # 上游装出来的 rpath 目录（rmagine 本体在这里）
            os.path.join(backend, 'rmagine', 'lib'),
        ]
    # 我们自己的底盘驱动插件（本仓包 rm_gz_planar_move，`<prefix>/lib/librm_gz_planar_move.so`）
    try:
        _planar_share = get_package_share_directory('rm_gz_planar_move')
        plugin_paths.append(os.path.dirname(os.path.dirname(_planar_share)))
        # 上面那一层是 <prefix>（share/rm_gz_planar_move → <prefix>），.so 在 <prefix>/lib
        plugin_paths[-1] = os.path.join(plugin_paths[-1], 'lib')
    except PackageNotFoundError:
        log('⚠ 没装 rm_gz_planar_move ⇒ gz 侧没有底盘驱动（/cmd_vel_chassis 无人消费）')
    for k in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'IGN_FILE_PATH'):
        actions_extra_env.setdefault(k, os.pathsep.join(res_paths))
    for k in ('GZ_SIM_SYSTEM_PLUGIN_PATH', 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH'):
        actions_extra_env[k] = os.pathsep.join(plugin_paths)
    log('GZ_SIM_SYSTEM_PLUGIN_PATH = %s' % actions_extra_env['GZ_SIM_SYSTEM_PLUGIN_PATH'])
    # ★ 2026-11-04：第三方后端（mid360 插件）依赖 `librmagine-embree.so.2` 等运行期库。
    #   只把目录塞进 GZ_SIM_SYSTEM_PLUGIN_PATH 是**不够**的（那只管 gz 插件查找），
    #   动态链接器还要 LD_LIBRARY_PATH 才找得到依赖：
    #   实测 `libmid360_system.so: librmagine-embree.so.2: cannot open shared object file`。
    if backend:
        _ld_dirs = [os.path.join(backend, 'rmagine', 'lib'),
                    os.path.join(backend, 'rmagine_gazebo_plugins', 'lib'),
                    os.path.join(backend, 'mid360_simulation_plugin_ros2', 'lib')]
        _ld = os.pathsep.join([d for d in _ld_dirs if os.path.isdir(d)])
        _old = os.environ.get('LD_LIBRARY_PATH', '')
        actions_extra_env['LD_LIBRARY_PATH'] = (_ld + os.pathsep + _old) if _old else _ld
        log('LD_LIBRARY_PATH(+=) = %s' % _ld)
    # 两个拼法都设：ignition-transport11（Fortress）读 IGN_PARTITION，gz-transport12+
    # 读 GZ_PARTITION；与 GZ_SIM_SYSTEM_PLUGIN_PATH 同一套"两代都设"的策略。
    actions_extra_env['GZ_PARTITION'] = partition
    actions_extra_env['IGN_PARTITION'] = partition

    ld = [LogInfo(msg='[gz_sim.launch] ' + m) for m in
          ('world = %s' % world_file, 'runtime dir = %s' % runtime_dir)]
    for k in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'IGN_FILE_PATH',
              'GZ_SIM_SYSTEM_PLUGIN_PATH', 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
              # ★ 2026-11-04：第三方后端的运行期依赖（librmagine-embree / libembree4）
              #   要靠 LD_LIBRARY_PATH 才能被动态链接器找到——漏了这一项时
              #   插件报 `librmagine-embree.so.2: cannot open shared object file`。
              'LD_LIBRARY_PATH',
              'GZ_PARTITION', 'IGN_PARTITION'):
        # ⚠️ 2026-11-04：**必须容忍缺键**。native 模式没有后端 ⇒ 上面不会写 LD_LIBRARY_PATH；
        #   直接 `actions_extra_env[k]` 会抛 KeyError: 'LD_LIBRARY_PATH'（用户实跑撞到的就是这个）。
        if k not in actions_extra_env:
            continue
        ld.append(SetEnvironmentVariable(k, actions_extra_env[k]))

    # ── ign gazebo ───────────────────────────────────────────────────────────
    cmd = ['ign', 'gazebo', '-r', '-v', '3']
    if gui not in ('True', 'true', '1'):
        cmd.append('-s')            # 无 GUI（server only）
    cmd.append(world_file)
    gz_env = {
        'HOME': gz_home,
        'XDG_CACHE_HOME': os.path.join(gz_home, '.cache'),
        'XDG_CONFIG_HOME': os.path.join(gz_home, '.config'),
        'XDG_DATA_HOME': os.path.join(gz_home, '.local', 'share'),
        'XDG_RUNTIME_DIR': os.path.join(gz_home, '.runtime'),
        'LIBGL_ALWAYS_SOFTWARE': '1',      # 无 GPU 机器：llvmpipe
    }
    log('gz 命令：' + ' '.join(cmd))
    ld.append(ExecuteProcess(cmd=cmd, output='screen', additional_env=gz_env))

    # ── /clock 桥（唯一必须桥的东西） ─────────────────────────────────────────
    bridge_cfg = os.path.join(share, 'config',
                              'ros_gz_bridge_native.yaml' if sensor_src == 'native'
                              else 'ros_gz_bridge.yaml')
    log('gz_sensor = %s ⇒ 桥表 %s' % (sensor_src, os.path.basename(bridge_cfg)))
    if sensor_src == 'native':
        # /gz/lidar(PointCloud2, gz scoped frame) → /livox/lidar(CustomMsg, frame=livox_frame)
        ld.append(Node(package='cloud_accumulator', executable='pc2_to_custom',
                       name='pc2_to_custom', output='screen',
                       parameters=[{'use_sim_time': True,
                                    'in_topic': '/gz/lidar/points',
                                    'out_topic': '/livox/lidar',
                                    # 感知链（patchwork/linefit）吃这条 ⇒ native 模式必须补齐
                                    'out_pc2_topic': '/livox/lidar/pointcloud',
                                    'frame_id': 'livox_frame',
                                    # ⚠️ gz 的 gpu_lidar 无逐点时间。合成时间**默认关闭**：
                                    #   2026-11-04 实测打开（且第一版按整体点序铺开）⇒ LIO 输出 NaN
                                    #   ⇒ lio_tf_adapter 发 NaN TF ⇒ LIO-SAM 的 GTSAM
                                    #   `IndeterminantLinearSystemException` 崩溃。
                                    #   现在改成 ring-major（按方位角）算法，但要**先单独验证**再默认开：
                                    #   想试就 -p synthetic_time_hz:=10.0 -p synthetic_h_samples:=640
                                    # 用环境变量开关，避免为了试一次就改代码：
                                    #   export GZ_SYNTH_TIME_HZ=10   （0 或缺省 = 关闭）
                                    'synthetic_time_hz': float(os.environ.get('GZ_SYNTH_TIME_HZ', '0') or 0),
                                    'synthetic_h_samples': int(os.environ.get('GZ_SYNTH_H', '640') or 640),
                                    'tag': 0}]))
        log('gz_sensor:=native ⇒ 起 pc2_to_custom（/gz/lidar → /livox/lidar，frame 覆盖为 livox_frame）')

    ld.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                   name='gz_clock_bridge', output='screen',
                   arguments=['--ros-args', '-p',
                              'config_file:=%s' % bridge_cfg]))

    # ── TF：与 Classic 逐字相同的两个发布者 ─────────────────────────────────
    if robot not in ('', 'none'):
        use_sim_time = LaunchConfiguration('use_sim_time')
        ld.append(Node(package='joint_state_publisher', executable='joint_state_publisher',
                       name='joint_state_publisher', output='screen',
                       parameters=[{'use_sim_time': use_sim_time},
                                   {'robot_description': robot_desc}]))
        ld.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                       name='robot_state_publisher', output='screen',
                       parameters=[{'use_sim_time': use_sim_time},
                                   {'robot_description': robot_desc}]))
    return ld


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
        'tilt_rpy', default_value=os.environ.get('GZ_TILT_RPY', ''),
        description='雷达安装倾角 rpy（rad，空格分隔）。空 = 模型自己的默认。'),
    DeclareLaunchArgument('world', default_value='RMUC2026',
                              description='与 Classic 同名；gz 侧目前只有 RMUC2026'),
        DeclareLaunchArgument('gz_world', default_value='',
                              description="'' = 用 world 槽位；'bench' = 台架世界（只有传感器，"
                                          '没有机器人；docs 里所有契约/性能数字都是它跑出来的）'),
        DeclareLaunchArgument('robot', default_value='',
                              description="''/none = 不起机器人；robot11 = 从 robot_description "
                                          '派生 Fortress 的 SDF 模型'),
        # ★ 2026-10-27（契约 §4.15）：`robot11_mount` 已删除（安装方式固定 = xacro 默认的
        #   `sensor`；本 launch 不再声明/读取它）。
        DeclareLaunchArgument('robot_description', default_value=''),
        DeclareLaunchArgument('gui', default_value='True'),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('gz_qos', default_value='best_effort',
                              description='点云发布 QoS：best_effort（默认，SensorDataQoS）|'
                                          'reliable（只用于复现"慢消费者堵死 gz 线程"）'),
        DeclareLaunchArgument('gz_verbose', default_value='3'),
        DeclareLaunchArgument('gz_ignore_self', default_value='true',
                              description='true（默认）= 自机排除生效（robot 不进 BVH）；'
                                          'false = **只用于对照实验**，让传感器看得见自己的车体'),
        DeclareLaunchArgument('gz_home', default_value='',
                              description='仅 gz 进程的 HOME。空 = 运行期临时目录'
                                          '（本机 ~/.ignition 只读会让 GUI abort）'),
        DeclareLaunchArgument('gz_csv', default_value='',
                              description='空 = 用 ros2_livox_simulation 包里那份 mid360.csv'),
        DeclareLaunchArgument('gz_runtime_dir', default_value='',
                              description='空 = /tmp/rm_livox_gz_sim_<rand>；调试时指定可回看生成物'),
        DeclareLaunchArgument('gz_extra_gui', default_value=''),
        DeclareLaunchArgument('gz_sensor',
                              default_value=os.environ.get('GZ_SENSOR', 'mid360'),
                              choices=['mid360', 'native'],
                              description='传感器来源：mid360（默认，第三方 libmid360_system.so，'
                                          '非重复扫描图案+逐点时间）| native（gz 原生 gpu_lidar，'
                                          '无第三方依赖，规则网格/无逐点时间，只用于管线验证）'),
        DeclareLaunchArgument('gz_drive', default_value='true',
                              description='true（默认）= 给 gz 机器人加**运动学底盘驱动**'
                                          '（/cmd_vel_chassis → 位姿 + /odom_ground_truth，'
                                          'Classic 的 planar_move 的对应实现）；false = 静态底座'
                                          '（A/B 对照用）'),
        DeclareLaunchArgument('gz_custom_msg_tag', default_value='0',
                              description='/livox/lidar（CustomMsg）的每点 tag。0（默认）= 与 '
                                          'Classic 逐字节一致；16 = 上游写死的 0x10（**只用于'
                                          '复现 small_point_lio 全丢点**）'),
        DeclareLaunchArgument('gz_partition', default_value='',
                              description='gz transport partition（GZ_PARTITION/IGN_PARTITION）。'
                                          '空 = 本次运行随机一个（**默认，必须**：同机另一个 '
                                          'Fortress 世界会把 /clock 与 /mid360/imu_gz 送进来，'
                                          '导致 /livox/imu 戳交错、small_point_lio 刷 '
                                          '`imu loop back`）；只有"故意共享 gz 话题"的对照实验'
                                          '才显式给一个名字'),
        OpaqueFunction(function=_materialize),
    ])
