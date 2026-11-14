# 仿真 / 场地 / 后端

> Gazebo 世界与后端、场地资产、仿真迁移

> **2026-11-13 合并**：本文件由 **7 份**同主题文档合并而成，原文全部保留、按来源分节（`## 来源：xxx`）。原件已删除，可从 git 历史取回。

**目录**
1. `gazebo_sim_migration_plan.md`
2. `worlds.md`
3. `worlds_2027.md`
4. `round_summary_gz_frames.md`
5. `round_summary_hybrid_default.md`
6. `session_digest.md`
7. `system_status.md`

---


## 来源 1/7：`sim/gazebo_sim_migration_plan.md`


> **文档性质：可行性 spike（探针）+ 迁移清单**。本文**不改任何功能性代码/配置/launch/world**。
> 所有实测都在 `TMPDIR` 等价的未跟踪临时目录 `.tmp_gzsim/` 里完成（`Xvfb` 复用 `.tmp_gzgui/tools/usr/bin/Xvfb`）。
>
> **一句话结论**：**这条迁移值得试，而且没有想象中贵 —— 但它换不来"MID-360 的 CSV 逐射线图案"。**
> 新 Gazebo 的 `gpu_ray` 是**光栅化**实现，30000 条射线的内存代价从 Classic 的 **2.74 GiB 塌到 ≈6 MiB**
> （本机实测，2880 → 30000 条只涨 5.8 MiB）、内存/射线的量级差 **≈435×**，而且**无 GPU 机器上照样跑得动
> （含 GUI）**；代价是**传感器图案的自由度从"任意 3 万条射线"退化成“只有水平×垂直规则网格”**，
> 要保住 MID-360 就必须**自己写一个 gz system**。
>
> **裁决速查**
>
> | 问题 | 结论 | 置信度 |
> |---|---|---|
> | Humble 官方配哪个新 Gazebo | **Fortress（`ignition-gazebo6` / gz-sim6）** | 高（官方配对表 + 本机已装） |
> | 本环境能不能装 | **能，而且已经装好了**（`ros-humble-ros-gz` 0.244.26 + `libignition-gazebo6` 6.18.0） | 高（apt 实测） |
> | 能不能用 Garden/Harmonic | 能拿到包，但 **`apt install ros-humble-ros-gzharmonic` 会 `Remv` 掉 Gazebo Classic + `ros-humble-gazebo-ros-pkgs` + `ros-humble-ros-gz*`** ⇒ **会摧毁现有基准台** | 高（`apt-get -s` 实测） |
> | 新 Gazebo 支持任意逐射线图案（MID-360 CSV）吗 | **不支持**。SDF `<lidar>` 规范全文只有规则网格；连 Jetty 全新的 CPU `cpu_lidar` 也只生成规则网格 | **很高**（规范全文 + 上游源码逐行） |
> | 无 GPU 支持软件渲染吗 | **支持，且本机实测可用**（Mesa llvmpipe / Xvfb，GL 4.5 core） | 高（本机实测） |
> | 没有 DISPLAY 呢 | **`ign gazebo -s --headless-rendering` 无 DISPLAY 也能出完整传感器数据** | 高（本机实测，点云逐字节同尺寸） |
> | 有"完全不碰渲染器"的纯 CPU 射线吗 | **只有 Jetty（gz-sim10）有**（`gz-sim-cpu-lidar` + `gz-physics` 射线），而 **Jetty 没有 Humble 的 ros_gz 包** | 高（上游源码 + apt 实测） |
> | GUI 能留吗 | **能**。GUI 是独立进程 + 独立 gz-rendering 实例，`gui:=` 槽位语义可以完整保留 | 高（实测窗口 + 上游架构） |
> | 30k 射线代价是"消除"还是"搬家" | **真的消除**（不是搬家）：从 **每射线一个 ODE 几何体**（O(rays) 内存/CPU）变成 **一张光栅化纹理**（O(1) 内存） | 中高（本机标定 + 上游实现） |

---

## 0. 状态、范围与证据分级

| 项 | 说明 |
|---|---|
| 本文测的机器 | 28 核 / 31 GiB / **无 `/dev/dri`、无加载的 NVIDIA 驱动**（`nvidia-smi` 失败）⇒ **本身就是一台"没有 GPU 的电脑"**，正好用来回答用户提的第三个问题 |
| 已执行 | apt 可用性、`apt-get download`、**Fortress PoC（带 GUI / 无头 / 无 DISPLAY 三种）**、30000 射线标定、源码级规范核对 |
| 未执行 | 真机世界（RMUC2026/RMUL2026 STL）迁移、`ros_gz_bridge` 端到端、nav2 整栈 A/B、Harmonic 实测（见 §8 诚实清单） |
| 证据分级 | ①**本机实测**（可复现，附录 A 给命令）②**本机安装物**（apt 包 / 头文件 / 已装规范文件）③**上游源码/规范**（带 URL）④**官方文档**（带 URL） |

---

## 1. 版本配对：ROS 2 Humble 到底配哪个新 Gazebo

### 1.1 官方配对矩阵（一手引用）

官方页 <https://gazebosim.org/docs/latest/ros_installation/> 给出的兼容表（原文照抄关键行）：

|  | **GZ Fortress (LTS)** | **GZ Harmonic (LTS)** | Gz Ionic | Gz Jetty (LTS) |
|---|---|---|---|---|
| ROS 2 Rolling | ❌ | ⚡ | ⚡ | ✅ |
| ROS 2 Kilted | ❌ | ⚡ | ✅ | ❌ |
| ROS 2 Jazzy (LTS) | ❌ | ✅ | ❌ | ❌ |
| **ROS 2 Humble (LTS)** | **✅** | **⚡** | ❌ | ❌ |

- `✅` = 推荐组合；`⚡` = 可以，但"**use with caution**，需要一些功夫"；`❌` = 不可能。
- **所以：Humble 的官方答案就是 Fortress。** 同页明确写："Usually, the latest version of Gazebo is available at
  the beginning of each ROS release cycle (for example **Gazebo Fortress for ROS 2 Humble**)."
- ⚠️ **Garden 对 Humble 是 `❌`**（不是 ⚡）。虽然 `packages.osrfoundation.org` 上确实存在
  `ros-humble-ros-gzgarden`，但官方配对表把它排除在 Humble 之外 —— 这条容易踩。
- Humble 想用 Harmonic 的话，官方给的唯一路径是**非官方二进制包**
  `ros-humble-ros-gzharmonic`（来自 `packages.osrfoundation.org`），并且原文警告：
  "These packages **conflict with `ros-humble-ros-gz*` packages**"。

### 1.2 各版本 EOL（一手引用）

官方 EOL 表 <https://gazebosim.org/docs/all/releases/>：

| 名称 | 发布 | **EOL** | 备注 |
|---|---|---|---|
| Jetty | 2025-09 | **2031-05** | LTS |
| Ionic | 2024-09 | 2026-12 | |
| Harmonic | 2023-09 | **2029-05** | LTS |
| Garden | 2022-09 | **2024-11** | **已 EOL** |
| **Fortress** | **2021-09** | **2027-05** | **LTS** |
| Edifice / Dome / Citadel | 2021-03 / 2020-09 / 2019-12 | 均已 EOL | |

ROS 2 侧：REP-2000 <https://www.ros.org/reps/rep-2000.html> 写 **"Humble Hawksbill (May 2022 – May 2027)"**。

> **一个很好用的巧合**：**Fortress EOL = 2027-05，Humble EOL = 2027-05**。
> 也就是说，**如果目标只是"跟着 Humble 生命周期走完"，Fortress 的生命周期刚好够，不需要为了续命而跳到 Harmonic。**

### 1.3 `ros_gz` 版本号 ↔ Gazebo 版本

**本机实测**（`apt-cache policy`）：已装 `ros-humble-ros-gz-sim 0.244.26-1jammy.20260907.235058`，
其依赖里明确写 `libignition-gazebo6-dev`。

```
$ apt-cache depends ros-humble-ros-gz-sim | grep ignition
  依赖: libignition-msgs8
  依赖: libignition-transport11
  依赖: libignition-gazebo6-dev        <-- Fortress = ign-gazebo 6
  依赖: libignition-math6-dev
```

⇒ **`ros-humble-ros-gz` `0.244.x` ≡ Fortress（gz-sim6）**。这是硬证据，不是推测。

各发行版对应的 `ros_gz` 包名与底层库名（**本机 apt 实测 + 上游命名**）：

| Gazebo | gz-sim | Humble 上的 ROS 包名 | 底层 apt 库 | 命令 |
|---|---|---|---|---|
| **Fortress** | gz-sim6 | `ros-humble-ros-gz`（+ `-sim` / `-bridge` / `-interfaces` / `-image`） | `libignition-gazebo6-dev` | **`ign gazebo`** |
| Garden | gz-sim7 | `ros-humble-ros-gzgarden*` | `libgz-sim7-dev` | `gz sim` |
| Harmonic | gz-sim8 | `ros-humble-ros-gzharmonic*` | `libgz-sim8-dev` | `gz sim` |
| Ionic | gz-sim9 | **无 Humble 包** | — | `gz sim` |
| Jetty | gz-sim10 | **无 Humble 包** | — | `gz sim` |

本机实测的候选版本：
`ros-humble-ros-gz → 0.244.26`（ROS 源）、`ros-humble-ros-gzgarden → 0.244.11`（OSRF 源）、
`ros-humble-ros-gzharmonic → 0.244.12`（OSRF 源）；
`libgz-sim7-dev → 7.9.0`、`libgz-sim8-dev → 8.15.0` 在 `packages.osrfoundation.org/gazebo/ubuntu-stable` 里都存在。

### 1.4 Ignition → Gazebo 改名与命令差异（会真实咬人的一条）

上游在 **Garden（gz-sim7）** 做了 Ignition→Gazebo 改名（见上方命名表；Fortress 仍用 `ignition-*` 库名与 `ign gazebo`；
`libignition-*` 与 `gz-*` 在 Fortress 的头文件里是**同一份**，本机 `/usr/include/ignition/gazebo6/gz/sim/System.hh`
就是 `#include <gz/sim/System.hh>` 的 shim —— 实测）。

**本机实测的一个坑**：

```
$ gz sim --versions
Invalid arguments
$ ign gazebo --versions
6.18.0
```

⇒ **Fortress 上不存在 `gz sim` 这个子命令**（`ign-tools 1.5.0` 只提供 `ign gazebo`）。
**所有从网上抄来的 `gz sim …` 命令在 Fortress 上会直接报错**，
而 `gz topic` / `ign topic` 两种写法都能用。迁移脚本里必须处理这个分支。

### 1.5 结论

- **首选 Fortress**，理由三条：①官方 ✅ 配对；②**本机已经装好**（零安装成本）；
  ③EOL 与 Humble 同步（2027-05），生命周期不是短板。
- **Harmonic 是"以后再说"的选项**，因为它在 Humble 上会与 Classic 打架（§2.3 实测）。
- **Jetty 不用考虑**：它是唯一有纯 CPU 射线的版本，但 **Humble 没有它的 ros_gz 包**（§2.1 实测）。

---

## 2. 环境实测：这台机器上到底有什么

### 2.1 apt 可用性（一手实测）

```bash
$ apt-cache policy ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
ros-humble-ros-gz:          候选 0.244.26-1jammy.20260908.015605   已安装：(无)   [mirrors.aliyun.com/ros2]
ros-humble-ros-gz-sim:      候选 0.244.26-1jammy.20260907.235058   已安装：同版本 [aliyun / dpkg]
ros-humble-ros-gz-bridge:   候选 0.244.26-1jammy.20260907.225444   已安装：同版本 [aliyun / dpkg]
```

**结论：`ros-humble-ros-gz-sim` 与 `ros-humble-ros-gz-bridge` 本来就已经装在这台机器上了**
（只有 meta 包 `ros-humble-ros-gz` 没装）。已装的核心库：

| 包 | 版本 |
|---|---|
| `libignition-gazebo6` / `-dev` / `-plugins` | **6.18.0** |
| `libignition-gui6` / `-dev` | 6.9.0 |
| `libignition-rendering6` / `-core-dev` / **`-ogre1`** / **`-ogre2`** | 6.6.4 |
| `libignition-msgs8` / `libignition-transport11` / `libignition-common4` / `libignition-fuel-tools7` | 8.7.1 / 11.4.2 / 4.9.0 / 7.3.1 |
| `sdformat12`（含 `sdformat12-sdf`） | 12.9.0 |
| Gazebo **Classic** `gazebo` / `libgazebo11` | **11.10.2** ← 与 Fortress **并存** |

已装的 Fortress system 插件：**147 个 `.so`**（`dpkg -L libignition-gazebo6-plugins`）。

> **注意一个易混点**：机器上同时有 Gazebo **Classic 11** 与 **Fortress**。
> 命令层面 `gazebo`/`gzserver`/`gzclient` = Classic，`ign gazebo` = Fortress。二者**不冲突**。

**没有的**（实测）：

```bash
$ apt-cache policy ros-humble-ros-gzjetty ros-humble-ros-gzionic
ros-humble-ros-gzjetty:  <不存在>
ros-humble-ros-gzionic:  <不存在>
```

⇒ **Jetty / Ionic 在 Humble 上没有官方 ros_gz 二进制包**。这条直接掐死了"用 Jetty 的纯 CPU 射线"这条路
（除非自己编 `ros_gz` + 自己编 gz-sim10，代价远超本 spike 范围）。

### 2.2 `apt-get download`（无 root）实测

**成功**。5 个 Fortress ros_gz deb 全部下到 `.tmp_gzsim/dl_fortress/`，**共 4.0 MiB**：

| 文件 | 大小 |
|---|---|
| `ros-humble-ros-gz-bridge_0.244.26-…deb` | 3,566,464 B |
| `ros-humble-ros-gz-sim_0.244.26-…deb` | 276,070 B |
| `ros-humble-ros-gz-interfaces_0.244.26-…deb` | 273,824 B |
| `ros-humble-ros-gz-image_0.244.26-…deb` | 55,936 B |
| `ros-humble-ros-gz_0.244.26-…deb` | 7,078 B |

⇒ **无 root 的 `apt-get download` 在本机完全可用**，所以"拿包来研究/验证"这件事没有障碍。
（`apt-get download` 只是下载 `.deb`，不会安装，不碰系统。**未做任何 `dpkg -i`**。）

### 2.3 ⚠️ Harmonic 路径的致命冲突（**这条决定"能不能保住现有基准台"**）

`apt-get -s install ros-humble-ros-gzharmonic`（**只模拟，不执行**）：

```
127 packages, 35.1 MiB 待下载
Remv gazebo [11.10.2+dfsg-1]
Remv ros-humble-gazebo-dev / -plugins / -ros / -ros-pkgs [3.9.0-1jammy]
Remv ros-humble-gazebo-ros2-control
Remv ros-humble-velodyne-simulator / -gazebo-plugins
Remv ros-humble-desktop-full
Remv ros-humble-ros-gz-sim / -bridge / -image / -interfaces / -sim-demos        <-- 连 Fortress 一起删
Remv ros-humble-ros-ign-image
```

⇒ **`apt install ros-humble-ros-gzharmonic` 会卸载 Gazebo Classic 11 与整个 `gazebo_ros_pkgs`。**
也就是说，**它和"不能丢掉现有可工作的 Classic 基准台"是直接冲突的**。

> **含义**：Harmonic（以及任何非 Fortress 的版本）**必须走"另一台机器 / 容器 / docker"**，
> 不能和现有 bench 装在同一套 `/opt/ros/humble` 里。
> 这也反过来把 **Fortress 推成唯一"零风险并行共存"的选项** —— 因为它**已经装好了**，
> 且没有对 Classic 做任何改动。

### 2.4 GPU / 显示现实（**本机就是一台无 GPU 机器**）

```bash
$ ls /dev/dri
ls: 无法访问 '/dev/dri': 没有那个文件或目录        # 没有 DRM 设备节点
$ nvidia-smi -L
NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver.
$ lspci | grep -i vga
01:00.0 VGA compatible controller: NVIDIA Corporation Device 2860 (rev a1)   # 有卡但驱动没起
```

**但软件渲染栈是完整的**（在 Xvfb 上实测 `glxinfo -B`）：

```
OpenGL vendor string:   Mesa
OpenGL renderer string: llvmpipe (LLVM 15.0.7, 256 bits)
Accelerated:            no
OpenGL core profile version string: 4.5 (Core Profile) Mesa 23.2.1-1ubuntu3.1~22.04.4
OpenGL ES profile version string:   OpenGL ES 3.2 Mesa 23.2.1
```

⇒ **无 GPU 但仍然拿到 OpenGL 4.5 core 上下文**，靠的是 Mesa `llvmpipe`。
本机 `/usr/lib/x86_64-linux-gnu/dri/` 里 `swrast_dri.so` / `kms_swrast_dri.so` 都在，
`libEGL_mesa.so.0`、`libgbm.so.1` 也都在。

---

## 3. 传感器分析（**最关键的一节**）

### 3.1 新 Gazebo 的传感器：不是"一个系统一个传感器"，而是**"一个 Sensors 系统管全部"**

**上游事实（源码目录，实测 404/200 探测 + GitHub 树负载）**：

| 版本 | `src/systems/` 里的传感器系统 | 说明 |
|---|---|---|
| **Fortress (gz-sim6)** | `sensors-system`（唯一），`imu-system`、`air-pressure-system`、`altimeter-system`、`magnetometer-system`、`logical-camera-system`、`thermal-system`、`contact-system` | **没有任何 lidar 专用系统** |
| Garden (gz-sim7) | `sensors`（只有 `Sensors.cc`） | 同上 |
| Harmonic (gz-sim8) | `sensors`（只有 `Sensors.cc`） | 同上 |
| Ionic (gz-sim9) | `sensors`（只有 `Sensors.cc`） | 同上 |
| **Jetty (gz-sim10)** | `sensors` **+ `src/systems/cpu_lidar`** ← **唯一新增** | 见 §3.7 |

**本机 Fortress 实测（一手，比上网查更硬）**：

```bash
$ dpkg -L libignition-gazebo6-plugins | grep '\.so$' | grep -iE 'lidar'
libVisualizeLidar.so                     # 只是 GUI 可视化插件，不是传感器系统
$ ls /usr/include/ignition/gazebo6/gz/sim/ | grep -i lidar
(空)                                      # 头文件里也没有 lidar-system
```

⇒ **Fortress 里 LiDAR 只有一条路：`<sensor type="gpu_ray">`，由 `Sensors` 系统驱动，底层是 gz-rendering。**

各 SDF `type=` 与系统插件的对应（Fortress 实测 + 上游）：

| SDF `type` | 系统插件 | 是否需渲染器 |
|---|---|---|
| `camera` / `depth_camera` / `rgbd_camera` / `segmentation_camera` / `thermal_camera` / `wide_angle_camera` | `ignition-gazebo-sensors-system` | **是** |
| `gpu_ray` / `gpu_lidar` | `ignition-gazebo-sensors-system` | **是**（光栅化） |
| `ray` / `lidar`（CPU） | **Fortress 无实现** ← 见 §3.5 | —（本就不存在） |
| `imu` | `ignition-gazebo-imu-system` | 否 |
| `air_pressure` / `magnetometer` / `altimeter` / `contact` / `force_torque` / `navsat` / `logical_camera` | 各自的 `*-system` | 否 |

### 3.2 **SDF `<lidar>` 规范全文**（这就是"图案自由度"的全部）

**本机安装物**（一手、权威、无需联网）：`/usr/share/sdformat12/1.9/lidar.sdf`（Fortress 用的 sdformat12 规范，
**全文 78 行**）。它的**全部**元素如下（这是逐行枚举，不是摘抄）:

```xml
<element name="lidar">
  <element name="scan" required="1">
    <element name="horizontal" required="1">
      <element name="samples"    type="unsigned int" default="640" required="1"/>
      <element name="resolution" type="double"       default="1"   required="1"/>
      <element name="min_angle"  type="double"       default="0"   required="1"/>
      <element name="max_angle"  type="double"       default="0"   required="1"/>
    </element>
    <element name="vertical" required="0">
      <element name="samples"    type="unsigned int" default="1" required="1"/>
      <element name="resolution" type="double"       default="1" required="0"/>
      <element name="min_angle"  type="double"       default="0" required="1"/>
      <element name="max_angle"  type="double"       default="0" required="1"/>
    </element>
  </element>
  <element name="range" required="1">
    <element name="min" type="double" default="0" required="1"/>
    <element name="max" type="double" default="0" required="1"/>
    <element name="resolution" type="double" default="0" required="0"/>
  </element>
  <element name="noise" required="0">
    <element name="type"   type="string" default="gaussian" required="1"/>
    <element name="mean"   type="double" default="0.0" required="0"/>
    <element name="stddev" type="double" default="0.0" required="0"/>
  </element>
  <element name="visibility_mask" type="unsigned int" default="4294967295" required="0"/>
</element>
```

**整个规范里没有任何一个元素能表达"第 i 条射线的方向"**。水平/竖直各自只有一个
`min_angle`/`max_angle`/`samples` 的三元组 ⇒ **只能是等间距规则网格（矩形 FOV 的均匀采样）**。

补充实测：在整套 SDF 规范里检索"图案类"元素 ——

```bash
$ grep -rlniE '<element name="(pattern|directions|rays|ray_directions|csv|points)"' \
      /usr/share/sdformat12/1.9/ /usr/share/sdformat9/1.7/
(无输出)
```

⇒ **不存在** `pattern` / `directions` / `ray_directions` / `csv` 之类的元素。
（对比：Classic 11 用的 `sdformat9/1.7/lidar.sdf` 是同样的结构 —— 也就是说，
**我们现在的 MID-360 图案从来不是 SDF 给的，而是我们自己的插件绕过了 SDF 采样**。）

### 3.3 证据 A：`gz::rendering::GpuRays` 的 API **只有规则网格**

**本机安装头文件**（一手）：`/usr/include/ignition/rendering6/gz/rendering/GpuRays.hh`。
它的**全部** setter（逐行枚举）：

```cpp
SetClamp(bool)                    Clamp()
SetIsHorizontal(bool)             IsHorizontal()
SetRayCountRatio(double)          RayCountRatio()
SetAngleMin(double)               AngleMin()            // 水平起始角
SetAngleMax(double)               AngleMax()            // 水平结束角
SetRayCount(int)                  RayCount()            // 水平采样数
SetVerticalRayCount(int)          VerticalRayCount()    // 竖直采样数
SetVerticalAngleMin(double)       VerticalAngleMin()
SetVerticalAngleMax(double)       VerticalAngleMax()
SetHorizontalResolution(double)   HorizontalResolution()
SetVerticalResolution(double)     VerticalResolution()
ConnectNewGpuRaysFrame(...)       Data() / Copy(float*)
```

⇒ **`GpuRays` 没有任何"给我一组任意射线方向"的入口。** 它只能表达"水平 N 条 × 竖直 M 条、等间距"。
（上游同一份：<https://github.com/gazebosim/gz-rendering/blob/gz-rendering6/include/gz/rendering/GpuRays.hh>）

`RayQuery` 倒是**逐条任意方向**（本机 `/usr/include/ignition/rendering6/gz/rendering/RayQuery.hh`）：

```cpp
class RayQuery : public virtual Object {
  virtual void SetOrigin(const math::Vector3d &) = 0;
  virtual void SetDirection(const math::Vector3d &) = 0;
  virtual void SetFromCamera(const CameraPtr &, const math::Vector2d &) = 0;
  virtual RayQueryResult ClosestPoint() = 0;      // {distance, point, objectId}
};
```

**但它是从"场景"里造出来的**：`Scene.hh` 里 `virtual RayQueryPtr CreateRayQuery() = 0;`
—— 即 `RayQuery` 依附于一个 `Scene`，而 `Scene` 来自某个渲染引擎 ⇒ **它不是"免渲染器"的离线射线。
（这是 §4.4 的关键分界线。）**

### 3.4 证据 B：**连 Jetty 全新的 CPU `cpu_lidar` 也只生成规则网格**

Jetty（gz-sim10）新增了 `src/systems/cpu_lidar/`，插件名 `gz-sim-cpu-lidar-system`，
类 `gz::sim::systems::CpuLidar`，**由 `gz-physics` 做射线求交、完全不走渲染器**。
但它生成射线的代码是（`gz-sensors10/src/CpuLidarSensor.cc`，上游源码）：

```cpp
// Load(): 预计算 unit vector —— 双层 for，纯规则网格
for (unsigned int v = 0; v < vSamples; ++v) {
  const double inclination = vMin + v * vStep;
  for (unsigned int h = 0; h < hSamples; ++h) {
    const double azimuth = hMin + h * hStep;
    this->dataPtr->unitVectors.emplace_back(
        std::cos(inclination) * std::cos(azimuth),
        std::cos(inclination) * std::sin(azimuth),
        std::sin(inclination));
  }
}

// GenerateRays(): 只会把上面那批 unit vector 拉成 (rMin, rMax) 线段
std::vector<std::pair<Vector3d, Vector3d>> CpuLidarSensor::GenerateRays() const {
  ...
  for (const auto &unitVec : this->dataPtr->unitVectors)
    rays.emplace_back(unitVec * rMin, unitVec * rMax);
  return rays;
}
```

而且：

```cpp
// gz-sensors10/include/gz/sensors/CpuLidarSensor.hh
public: std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>>
  GenerateRays() const;                    // <-- 注意：没有 virtual
```

⇒ **双重否定**：①射线集来自规则网格；②`GenerateRays()` **不是 `virtual`**，
`unitVectors` 是私有 `dataPtr` 成员 ⇒ **子类也无法覆写图案**。
（上游：<https://github.com/gazebosim/gz-sensors/blob/gz-sensors10/src/CpuLidarSensor.cc>、
<https://github.com/gazebosim/gz-sim/tree/gz-sim10/src/systems/cpu_lidar>）

### 3.5 证据 C：Fortress **根本没有 CPU `ray` 传感器**（运行期一手证据）

PoC 世界（`.tmp_gzsim/poc_fortress.sdf`）里同时放了一个 `type="gpu_ray"` 和一个 `type="ray"`。
启动日志立刻给了答案：

```
[Wrn] [SdfEntityCreator.cc:917] Sensor type LIDAR not supported yet.
                                Try using a GPU LIDAR instead.
```

并且订阅 `/poc/cpu_ray` **超时**（`exit=124`，20 s 一条都没有），而 `/poc/gpu_lidar` 正常出 360×8 的
`LaserScan` 与 `PointCloudPacked`。

> **这一条是本调研里最有实践价值的发现**：Fortress（**Humble 的官方配对**）**没有纯 CPU 的 LiDAR 路径**。
> 所以在 Fortress 上，**LiDAR ⇔ 渲染器 ⇔ GL 上下文**，没有第二条路。
> 想"完全不碰渲染器做 LiDAR"必须上 Jetty，而 Jetty 没有 Humble 包（§2.1）。

### 3.6 社区侧交叉印证（上游 issue）

| issue | 标题 | 说明 |
|---|---|---|
| [gz-sim#1958](https://github.com/gazebosim/gz-sim/issues/1958) | **Custom lidar sensor** | 用户问怎么自定义 LiDAR |
| [gz-sim#2645](https://github.com/gazebosim/gz-sim/issues/2645) | **Build my own Lidar Sensor with custom scanning shape** | 用户问**自定义扫描形状** —— 正是我们的 MID-360 问题 |

（这两个 issue 的正文在本沙箱里没能完整抓到——GitHub 只回了导航框架、正文被截断；
**它们的存在与标题可见，但结论未逐句核对**，已列入 §8 诚实清单。）

### 3.7 **如果要做：自定义 gz system 必须实现什么**（精确接口）

#### 3.7.1 系统插件接口（**本机 Fortress 头文件逐行**）

`/usr/include/ignition/gazebo6/gz/sim/System.hh` 全文（去注释）：

```cpp
namespace ignition { namespace gazebo {
class System { public: System() = default; public: virtual ~System() = default; };

class ISystemConfigure {
  public: virtual void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) = 0;
};

class ISystemConfigureParameters {
  public: virtual void ConfigureParameters(
      ignition::transport::parameters::ParametersRegistry &_registry,
      EntityComponentManager &_ecm) = 0;
};

class ISystemPreUpdate {
  public: virtual void PreUpdate(const UpdateInfo &_info,
                                 EntityComponentManager &_ecm) = 0;
};

class ISystemUpdate {
  public: virtual void Update(const UpdateInfo &_info,
                              EntityComponentManager &_ecm) = 0;
};

class ISystemPostUpdate {
  public: virtual void PostUpdate(const UpdateInfo &_info,
                                  const EntityComponentManager &_ecm) = 0;  // 注意 const
};
}}
```

**要点**：
- `PreUpdate` 拿**可变** `EntityComponentManager&`（可以 `CreateComponent`）；
  `PostUpdate` 拿 **`const EntityComponentManager&`**（只能读）。
  ⇒ **"写传感器数据"必须发生在 `PostUpdate`**（读 `RaycastData` 结果 → 发布）。
- `UpdateInfo` 成员（本机 `Types.hh`）：`simTime`、`realTime`、`dt`、`paused`、`iterations`、`residualStepSize`。
- 插件用宏自注册：

```cpp
GZ_ADD_PLUGIN(CpuLidar, System, CpuLidar::ISystemPreUpdate, CpuLidar::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(CpuLidar, "gz::sim::systems::CpuLidar")
```

（Fortress 里对应 `IGNITION_ADD_PLUGIN`；Jetty 的 `cpu_lidar` 实测就是上面这两行。）

#### 3.7.2 传感器侧的必需件

| 需要什么 | Fortress 对应物 | 本机路径 / 引用 |
|---|---|---|
| 传感器工厂 | `sensors::SensorFactory` | `gz/sensors/SensorFactory.hh`（gz-sensors6） |
| 传感器抽象 | `gz::sim::Sensor`（`Entity()` / `Name()` / `Pose()` / `Topic()` / `Parent()`） | `/usr/include/ignition/gazebo6/gz/sim/Sensor.hh` |
| 组件 | `components::Sensor`、`components::SensorTopic`、`components::ParentEntity`、`components::WorldPose` | `/usr/include/ignition/gazebo6/gz/sim/components*` |
| 拿渲染场景 | `gz/sim/rendering/RenderUtil.hh`、`SceneManager.hh`、`Events.hh` | `/usr/include/ignition/gazebo6/gz/sim/rendering/` |
| 发射 gz-transport 话题 | `gz::transport::Node::Advertise<gz::msgs::LaserScan>` / `PointCloudPacked` | gz-transport11（本机已装） |

#### 3.7.3 **Jetty 给出的"正确答案"就是一份可直接照抄的参考实现**

Jetty 的 `CpuLidar` 把"CPU LiDAR"拆成了**三个组件 + 一个物理后端**，这个架构才是最省事的落点：

```
CpuLidar::PreUpdate(ecm)                       // 系统侧
  ├─ 对新传感器: CreateComponent(SensorTopic)
  ├─ rays = sensor->GenerateRays()             // ① 生成射线集
  ├─ CreateComponent(entity, components::RaycastData{rays})
  └─ CreateComponent(entity, components::NeedsRaycast(true))

（gz-physics 的 raycast 系统读 RaycastData/NeedsRaycast，做求交，写回 results）
                                                 // ② 求交：跟渲染器完全无关

CpuLidar::PostUpdate(ecm)
  ├─ Each<CpuLidar, RaycastData, WorldPose>
  │    → sensor->SetPose(worldPose)
  │    → sensor->SetRaycastResults(results)      // ③ 填 ranges/intensities
  └─ sensor 自己发布 LaserScan / PointCloudPacked
```

**所以要支持 MID-360 的 CSV 图案，最小改动点是 ①**：把 `unitVectors` 的来源从"规则网格"
换成"读 `mid360.csv`"。**但**因为 `GenerateRays()` 非虚 + `unitVectors` 私有，
**不能靠子类**，只能在下面三条里选一条：

| 路线 | 做法 | 工作量 | 风险 |
|---|---|---|---|
| **R1（务实）** | **自己写一个 gz system**，不复用 `gz-sensors::CpuLidarSensor`：自己读 CSV → 自己持有一组方向 → 自己在 `PostUpdate` 里用 `RayQuery` 逐条打，或自己接进 `RaycastData` 那条链 | **L** | 需自己实现发布（`LaserScan`/`PointCloudPacked`）、时间戳、`frame_id`、`<topic>` 语义 |
| R2（上游补丁） | **fork `gz-sensors`**，给 `CpuLidarSensor` 加一个"从 CSV/显式方向表载入 `unitVectors`"的口子（把 `GenerateRays()` 改虚或加 `SetRayDirections()`） | **M**（改动小但**要维护 fork**） | 版本升级要重放补丁；且必须让上游接受才可持续 |
| R3（渲染器侧） | fork `gz-rendering` 的 OGRE2 `GpuRays`，把它那张"射线方向打包纹理"换成 CSV 烘焙的任意方向表（上游文件 `ogre2/src/Ogre2GpuRays.cc` + `media/` 下的 GLSL） | **L+** | 最深的改动，但**唯一能保住"光栅化 O(1) 内存"优势**的路线；每次渲染库升级都要重做 |

> **本 spike 不做 R1/R2/R3 中的任何一个** —— 只把"必须做什么"钉死，供后续单独立项。

**还有一个"小便宜"路线（R0，值得单独提）**：
如果 MID-360 的**非重复性**不是刚性需求，可以只用 `gpu_ray` 的规则网格去逼近：
`<horizontal><samples>100</samples></horizontal>` × `<vertical><samples>300</samples></vertical>`
= 30000 条，**零代码改动**（本机 1000×30 已实测跑通，RTF≈1.0）。代价是**图案变了** —— 属于
**感知契约变更**，必须另开任务做量化验收（同 `docs/gazebo_gui_troubleshooting.md` §6.4.1 末尾对
"降 `<samples>`" 的口径：**用户已否决降采样，必须量化验收**）。R0 只作为"想要先把链路跑通"的临时手段。

### 3.8 传感器结论

1. **新 Gazebo 的官方传感器体系不支持任意逐射线图案 —— 一个都不支持（含 Jetty）。**
   依据是 SDF `<lidar>` 规范全文（4 个角度参数 + 2 个采样数）+ `GpuRays` API 全集 + Jetty `cpu_lidar` 源码。
2. 我们现在的 MID-360 图案**之所以存在，正是因为 Classic 上我们用了自己的插件绕过 SDF**。
   迁移到新 Gazebo **等于把这个"绕过去"的能力交回去** ⇒ **必须自己再写一个 system 才能拿回来**。
3. **Fortress 连 CPU `ray` 都没有** ⇒ LiDAR 与渲染器强绑定（这是 §4 的支点）。

---

## 4. 无 GPU 分析

### 4.1 三条路径的定义与实测归属

| 路径 | 机制 | 需要 DISPLAY？ | 需要 GPU？ | 本机实测 |
|---|---|---|---|---|
| **A. X + llvmpipe** | Xvfb 虚拟屏 + Mesa 软件光栅化（GLX） | 需要（虚拟屏即可） | **不需要** | ✅ **已跑通（含 GUI 窗口）** |
| **B. `--headless-rendering`** | EGL 无窗口上下文（软件 EGL） | **不需要** | **不需要** | ✅ **已跑通（无 DISPLAY）** |
| **C. 纯 CPU 射线（不走渲染器）** | `gz-physics` 射线求交 | 不需要 | 不需要 | ❌ **Humble 上拿不到**（只有 Jetty 有，无 Humble 包） |

`--headless-rendering` 的语义在**本机 `ign gazebo --help` 里有一手文字**：

```
--headless-rendering     Run rendering in headless mode
--render-engine [arg]    Ignition Rendering engine plugin to load for both the server and the GUI.
                         Gazebo will use OGRE2 by default. (ogre2)
--render-engine-gui      … for the GUI
--render-engine-server   … for the server
-s                       Run only the server (headless mode). This overrides -g, if it is also present.
-g                       Run only the GUI.
```

⇒ 渲染引擎可**分别**为 server / GUI 指定（`--render-engine-server` / `--render-engine-gui`），
这给了"GUI 用贵的引擎、传感器用便宜的引擎"这种调优空间。
本机可用的引擎插件只有 **`ogre`（OGRE 1.9）与 `ogre2`（OGRE 2.1+）**，
**没有 `optix`**（`ls /usr/lib/x86_64-linux-gnu/ | grep ignition-rendering6-optix` 为空）。

### 4.2 路径 A 实测：**无 GPU 机器上 GUI 能起来并渲染**

在 Xvfb `:99`（1280×800×24）+ `LIBGL_ALWAYS_SOFTWARE=1` 下：

```bash
$ DISPLAY=:99 xwininfo -root -children
     5 children:
     0x200012 "Gazebo": ("ign-gazebo-gui" "Gazebo GUI")  1000x845+0+0  +0+0
     0x200014 "Gazebo GUI": ()  1x1+0+0  +0+0
     0x400001 "ign gazebo gui": ("ign gazebo gui" "Ign gazebo gui")  10x10+10+10  +10+10
     0x200004 "Qt Selection Owner for ign-gazebo-gui": ()  3x3+0+0
```

GUI 插件全部加载成功（`3D View` / `MinimalScene` / `World control` / `World stats` / `Entity Tree` …），
日志里唯一的渲染相关警告是：

```
[Wrn] [Ogre2RenderTarget.cc:574] Anti-aliasing level of '8' is not supported;
                                 valid FSAA levels are: [ 0 4 ]. Setting to 0
```

⇒ **llvmpipe 的 FSAA 只有 0 和 4 两档**，`gui.config` 默认要 8 ⇒ 被静默降级。这是无 GPU 机器上的
**画质降级点**（不是功能故障）。

**一个有趣的重复发现**：本仓 `docs/gazebo_gui_troubleshooting.md` §6.3 早就量到 Classic 的
`gzclient` 在软件光栅化下要 **444–609 % CPU** —— 与本次 Fortress GUI 量到的 **1269 % CPU**
是同一个现象（**软件光栅化是多线程且极吃 CPU**），只是 Fortress 的 GUI 更重。

### 4.3 路径 B 实测：**连 DISPLAY 都不需要**

`ign gazebo -r -v 3 -s --headless-rendering <world>`，**显式 `unset DISPLAY`**：

- 启动日志**零错误**（无 `EGL`、`Failed`、`Unable` 字样）；
- `/world/poc/stats` 里 **`real_time_factor: 1`**；
- `/poc/gpu_lidar/points` **正常出数据**，且与"有 Xvfb"的那次**逐字节同尺寸**：

| 运行 | `points.txt` 字节数 | `width` × `height` |
|---|---|---|
| `-s` + Xvfb + llvmpipe | **3,624,688** | 1000 × 30 |
| `-s --headless-rendering` + **无 DISPLAY** | **3,624,688** | 1000 × 30 |

⇒ **"无 GPU + 无显示器"的服务器上，`--headless-rendering` 的软件 EGL 路径可用，
而且传感器数据是真的（不是空点云）。** 这是"无 GPU 也可以做 CI/无头回归"的直接依据。

### 4.4 路径 C：**纯 CPU 射线在新 Gazebo 里只有 Jetty 有，而 Humble 拿不到**

- 机制（Jetty）：`components::RaycastData` + `components::NeedsRaycast` 交给 **`gz-physics`**，
  由物理引擎对碰撞几何求交 —— **完全绕过 gz-rendering**。这确实是"不需要渲染器的 CPU 射线"。
- **但**：①它 **只有 gz-sim10（Jetty）** 有（§3.1 表）；②`apt-cache policy ros-humble-ros-gzjetty`
  → **不存在**（§2.1 实测）；③Jetty 也不支持任意图案（§3.4）。
- **`RayQuery` 不是路径 C**：它由 `Scene::CreateRayQuery()` 创建（本机 `Scene.hh:1124`），
  依附于渲染引擎 ⇒ 仍然要一个 GL（或软件 GL）上下文。
  ⇒ **"不碰渲染器的纯 CPU 射线"在 Humble 可行方案里没有。**

### 4.5 性能代价（无 GPU 的代价到底在哪）

| 项 | 有/无 GPU | 实测（本机，无 GPU） |
|---|---|---|
| `llvmpipe` 峰值 × 满 3D GUI | 无 GPU | **`ign gazebo gui` 1269 % CPU**（≈12.7 核） |
| 纯 server（无 GUI 窗口） | 无 GPU | **`ign gazebo server` 22.9–23.5 % CPU** |
| 传感器是否可用 | 无 GPU | **可用**（`gpu_ray` 出真数据） |
| 画质 | 无 GPU | FSAA 8→0（被降级） |
| 有无 GPU 的差距 | — | **本机无法对照量**（这台机器没 GPU）⇒ 已列入 §8 |

⇒ **无 GPU 的代价几乎全在 GUI 上**：把 GUI 关掉（`gui:=False`），server 侧只要 ~23 % 单核；
**留着 GUI 就得多付十几核**。但这正是用户要的"GUI 必须留着"的代价，且**它不阻塞传感器**。

### 4.6 无 GPU 结论

1. **支持软件渲染路径，而且是双保险**：路径 A（Xvfb + llvmpipe）与路径 B（`--headless-rendering` 软件 EGL）
   **本机都实测跑通**，且路径 B **连显示器都不要**。
2. **无 GPU 不等于无传感器**：`gpu_ray` 在无 GPU 机器上**出真数据**（点云字节数逐字节一致）。
3. **没有可用的"纯 CPU 射线"退路**：路径 C 只在 Jetty，Humble 拿不到；`RayQuery` 仍要 GL 上下文。
4. **代价**：GUI 在软件光栅化下极吃 CPU（十几核），且 FSAA 从 8 降到 0；
   server 侧很便宜（~23 % 单核）。
5. ⇒ **"没有 GPU 的电脑" 不是迁移的阻碍**，但**"想同时开大 GUI + 30000 射线" 在无 GPU 机器上会很吃力**（§5.5）。

---

## 5. PoC 实测（**已执行**）

### 5.1 环境与方法

| 项 | 值 |
|---|---|
| 引擎 | **Fortress `ign gazebo` v6.18.0**（`ros_gz_sim` 0.244.26 已装但 PoC 直接用 `ign gazebo` CLI） |
| 世界 | `.tmp_gzsim/poc_fortress.sdf`（1 个 ground plane + 2 面墙 + 1 个静止 sensor rig），以及生成器 `.tmp_gzsim/gen_world.py` 产出的 `w_2880.sdf` / `w_30k.sdf` |
| 传感器 | `gpu_ray`（水平×竖直规则网格）、`ray`（CPU，用于证伪）、`imu` |
| 虚拟屏 | `.tmp_gzgui/tools/usr/bin/Xvfb :99 -screen 0 1280x800x24`（**复用既有 scratch 里的 Xvfb**，未新装） |
| 环境 | `LIBGL_ALWAYS_SOFTWARE=1`；**`HOME` 重定向到 `.tmp_gzsim/home`**（见 §5.2 的坑） |
| 脚本 | `.tmp_gzsim/run_poc.sh`（GUI）、`.tmp_gzsim/run_scale.sh`（标定，三种模式） |

> **⚠️ 一个很有价值的踩坑（会原样搬到真迁移里）**：沙箱里 `$HOME` 只读，Fortress 的 GUI 会因为
> ```
> [Err] [Filesystem.cc:471] Failed to create directory [/home/weicheng/.ignition/gazebo]: Read-only file system
> [Err] [Gui.cc:86] Failed to create the default config folder […/gazebo/6]
> terminate called after throwing an instance of 'std::logic_error'
>   what():  basic_string::_M_construct null not valid
> ```
> **直接 abort**，而且 `ign gazebo` 会把 server 一起 `SIGKILL`。
> ⇒ **`$HOME/.ignition/` 必须可写**（或把 `HOME` 指到别处）。
> 这条对"CI/容器里跑 `ign gazebo`"是硬前提。

### 5.2 结果 1：**GUI 起得来，有真窗口**（回答"GUI 能不能留"）

见 §4.2：`xwininfo` 里有 `("ign-gazebo-gui" "Gazebo GUI") 1000x845`，GUI 插件全部加载，
`[Msg] Loaded plugin [3D View] / [MinimalScene] / [World control] / [World stats] …`。

**⇒ GUI 能留。** 而且在**没有 GPU** 的机器上也能留。

### 5.3 结果 2：传感器发布正常

`ign topic -l` 打出来的话题（节选）：

```
/clock
/gazebo/resource_paths
/gui/camera/pose
/poc/gpu_lidar                 <-- LaserScan
/poc/gpu_lidar/points          <-- PointCloudPacked（Fortress 同时发两种）
/poc/imu
/sensors/marker
/stats
/world/poc/{clock,pose/info,dynamic_pose/info,scene/info,state,stats}
```

`/poc/gpu_lidar` 内容（实测）：`angle_min: -3.14159`、`angle_max: 3.14159`、
`angle_step: 0.017501894150417828`、`count: 360`、`vertical_count: 8`、
`frame: "sensor_rig::link::gpu_lidar_3d"`。

`/poc/imu` 内容（实测）：`linear_acceleration { z: 9.8 }`、`orientation { w: 1 }`。
（`z: 9.8` 是重力项，符合预期。）

**⚠️ 话题命名与 Classic 不同**：Fortress 默认 `frame_id` 形如
`<model>::<link>::<sensor>`（`::` 分隔），而 Classic 是 `<link>_<sensor>` 之类。
**`ros_gz_bridge` 的配置和下游 `frame_id` 假设必须跟着改**（§6.7）。

### 5.4 结果 3：CPU `ray` 在 Fortress 不存在（证伪）

```
[Wrn] [SdfEntityCreator.cc:917] Sensor type LIDAR not supported yet. Try using a GPU LIDAR instead.
```
+ `/poc/cpu_ray` 订阅超时（`exit=124`）。

### 5.5 结果 4：**30000 射线标定（本调研最硬的一个数字）**

三组运行，`server-only`（`-s`，无 GUI 窗口）以隔离传感器成本，采样 `/world/poc/stats`：

| 运行 | 射线数 | 模式/环境 | **RTF** | **RSS** | **CPU** | 点云 |
|---|---|---|---|---|---|---|
| `rays2880_server` | 360×8 = **2,880** | `-s` + Xvfb + llvmpipe | **1.0001** | **505,492 KB ≈ 494 MiB** | 23.5 % | 360×8，`points.txt` 347,555 B |
| `rays30k_server` | 1000×30 = **30,000** | `-s` + Xvfb + llvmpipe | **0.9999** | **511,396 KB ≈ 499 MiB** | 23.5 % | 1000×30，`points.txt` 3,624,688 B |
| `rays30k_headless_nox` | 1000×30 = **30,000** | `-s --headless-rendering`，**无 DISPLAY** | **1** | — | — | 1000×30，3,624,688 B（同上） |

点云元数据证明射线真的打了（不是空跑）：

```
# rays30k_server/points.txt
width: 1000
height: 30
point_step: 32
row_step: 32000
```

**关键 delta**：射线数 **×10.4**（2,880 → 30,000），
- **RTF 从 1.0001 → 0.9999（无变化）**
- **RSS 只涨 5,904 KB ≈ 5.8 MiB**

⇒ **≈0.218 KiB / 条射线**。

**与 Classic 对照**（本仓 `docs/gazebo_gui_troubleshooting.md` §6.4.1 的实测：
2.74 GiB / 30000 条 = **≈94.8 KiB/条**，根因是 `livox_ode_multiray_shape.cpp` 的 `AddRay()`
**每条射线新建一个 `ODECollision` + 一个 `ODERayShape`**）：

| 实现 | 内存/条射线 | 机制 |
|---|---|---|
| Classic 自研插件（现状） | **≈94.8 KiB** | 每射线一个 ODE 几何体（`[heap]` 2.78 GiB） |
| **Fortress `gpu_ray`** | **≈0.218 KiB** | 光栅化：一张射线方向纹理 + 一张深度图 |
| **倍数** | **≈435×** | |

> **⚠️ 必须诚实标注的口径差异（否则这个倍数会被误读）**：
> ①Classic 的 94.8 KiB/条 是在**真机世界 + `robot11` + 一个完整栈**下量的；
> ②本次 0.218 KiB/条 是在**只有静态几何、无物理动力学**的微型世界里量的，
> 而且射线数是 **1000×30 规则网格**，**不是 MID-360 的 30000 条任意方向**；
> ③两者 RTF 不可直接比（Classic 0.32–0.46 含 SLAM/LIO 整栈；本次 ≈1.0 是世界几乎不干活）。
> **可以安全下的结论**是：**"每射线一个碰撞体"→"一张纹理"这个架构变化，
> 使射线数不再线性地放大内存与单线程世界更新成本**；**不能**下的结论是"整栈 RTF 会从 0.4 变成 1.0"。

### 5.6 结果 5：无 DISPLAY 的 `--headless-rendering`（见 §4.3）

无 DISPLAY、零报错、RTF=1、点云与有屏那次**逐字节同尺寸**。

### 5.7 PoC 数字总表

| 场景 | GUI 窗口 | RTF | RSS(gzserver) | RSS(gzclient) | CPU(gui/server) | 传感器 |
|---|---|---|---|---|---|---|
| GUI + 2880 射线（Xvfb+llvmpipe） | ✅ 1000×845 | 0.79–0.85 | 523 MiB | **582 MiB** | **1269 %** / 22.9 % | ✅ |
| `-s` + 2880 射线 | — | 1.0001 | 494 MiB | — | — / 23.5 % | ✅ |
| `-s` + 30000 射线 | — | 0.9999 | 499 MiB | — | — / 23.5 % | ✅ |
| `-s --headless-rendering` + 30000 射线，无 DISPLAY | — | 1 | — | — | — | ✅ |

### 5.8 PoC 的边界（**明确没测什么**）

- 没测**真机世界**（RMUC2026 / RMUL2026 STL，几十 MB 级 mesh）—— 世界加载耗时与内存未量。
- 没测**机器人动力学**（本次全 `static`，物理成本≈0）。
- 没测 `ros_gz_bridge` 端到端（`/scan`、`/livox/lidar` 到 ROS 侧）。
- 没测 **MID-360 的 30000 条任意方向**在新 Gazebo 上到底什么代价（因为**做不到**，§3）。
- 没测 **Harmonic**（会摧毁 Classic，§2.3）。
- 没测 **有 GPU 机器的对照**（本机无 GPU）。
- 没测 `ogre`（OGRE 1.9）引擎与 `ogre2` 的差异。

---

## 6. 迁移清单（逐类，**路径/数量来自本仓实测**）

规模基线：本仓 tracked 文件 **1,646**；`src/simulation/rm_simulation` **143** 个文件；`src/bringup/rm_nav_bringup` **94** 个文件；
`tools/scripts` 非 `__pycache__` 文件 **125** 个。

### 6.0 总表

| # | 类别 | 涉及文件/条目 | 工作量 | 风险 | 能否渐进/加开关 |
|---|---|---|---|---|---|
| 1 | URDF/xacro `<gazebo>` 插件块 | **5** 个 xacro（3 个 sentry robot + 1 个 waking robot + 1 个 mid360） | **M** | 中 | ✅ 可（新 slot） |
| 2 | **自定义 Livox 插件** | **12** 文件（`livox_laser_simulation_RO2/`）+ **17 MB / 800,001 行** `mid360.csv` | **L** | **极高** | ⚠️ 难（核心能力） |
| 3 | 驱动插件 `libgazebo_ros_planar_move.so` | 出现在 **4** 个 xacro（共 6 处）+ 里程计/TF 契约 | **L** | 高 | ✅ 可（新 slot） |
| 4 | `.world` → SDF | **5** 个 `.world` + **18** 个 `model.sdf` | **M** | 中 | ✅ 可（新世界目录） |
| 5 | 障碍物 C++ 插件 | **14** 个 `obstacle*.cc`（Gazebo Classic C++ API） | **L** | 中高 | ✅ 可（只影响 `RMUL2024_world_dynamic_obstacles`） |
| 6 | launch 文件 | `bringup_sim.launch.py`（1300+ 行）、`rm_simulation.launch.py` | **M** | 中 | ✅ 可（`sim_backend:=` 新槽） |
| 7 | 话题/TF/`ros_gz_bridge` | 现存 **0** 个 bridge 配置；`base_link_fake` 出现在 **67** 处 | **M** | **高** | ✅ 可（bridge 独立进程） |
| 8 | 感知链（linefit→p2l→`/scan`） | 应**零改动**（话题契约由 bridge 维持） | **S** | 低 | ✅ |
| 9 | nav2 params | **18** 个 `nav2_params*.yaml`（多数只受话题名影响） | **S** | 低 | ✅ |
| 10 | 地图资产 | `map/` 242 MB + `PCD/` 118 MB，**格式与后端无关** | **—** | 无 | ✅ 完全不动 |
| 11 | `tools/scripts/**` 实验台 | **57 / 125** 个非 pyc 脚本含 `gazebo|gzserver|gzclient` | **M** | 中 | ✅ 可（双后端分支） |

### 6.1 URDF/xacro 的 `<gazebo>` 插件块

**实测清单**（含 `libgazebo_ros_*` 或 `<gazebo` 的**我们的**文件，共 5 个）：

| 文件 | 行数 | 关键内容 |
|---|---|---|
| `src/bringup/rm_nav_bringup/urdf/sentry_robot_sim.xacro` | 266 | `libgazebo_ros_planar_move.so`（`cmd_vel:=cmd_vel_chassis`、`odom:=odom_ground_truth`）、`libgazebo_ros_imu_sensor.so`（`~/out:=/livox/imu`） |
| `src/bringup/rm_nav_bringup/urdf/sentry_robot_hzmirm_sim.xacro` | 398 | 同上（"与默认模型逐字同款"） |
| `src/bringup/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro` | **892** | **内联了一段 `<sensor type="ray" name="livox_frame">`**（`samples 100`×`360`、`<samples>30000</samples>`、`<topic>/livox/lidar</topic>`）+ `planar_move` + `imu_sensor` |
| `src/simulation/rm_simulation/hzmi_rm_simulation/urdf/simulation_waking_robot.xacro` | 259 | `planar_move`（`odom:=odom`）、`imu_sensor` |
| `src/simulation/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro` | 106 | **`<sensor type="ray">` + `libros2_livox.so` + CSV 路径 + `<samples>30000</samples>`** |

**本仓用到的 `gazebo_ros` 插件频次**（实测）：

| 插件 | 出现次数 | 新 Gazebo 对应 |
|---|---|---|
| `libgazebo_ros_imu_sensor.so` | **10** | `ignition-gazebo-imu-system` + `ros_gz_bridge` |
| `libgazebo_ros_ray_sensor.so` | **6** | `sensors-system` (`gpu_ray`) + bridge |
| `libgazebo_ros_planar_move.so` | **6** | **无直接对应** → `ignition-gazebo-diff-drive-system`（语义不同，见 §6.3） |
| `libgazebo_ros_joint_state_publisher.so` | **6** | `ignition-gazebo-joint-state-publisher-system` |
| `libgazebo_ros_diff_drive.so` | **6** | `ignition-gazebo-diff-drive-system` |
| `libgazebo_ros_camera.so` | 3 | `sensors-system` + bridge |
| `libgazebo_ros_p3d.so` | **12**（12 个 obstacle `model.sdf`） | **无直接对应** → 需替换为 `PosePublisher`/自研 |
| `libros2_livox.so`（我们的） | 2 | **无对应** → §6.2 |

**工作量 M / 风险中 / 可渐进**：新增一份 `*_gz_sim.xacro`（或 `sim_backend:=gz` 分支），
**不动现有 xacro** ⇒ Classic 基准台零影响。

### 6.2 ⚠️ 自定义 Livox 插件（**最高风险项**）

**现状（本仓实测）**：

| 文件 | 作用 |
|---|---|
| `src/simulation/rm_simulation/livox_laser_simulation_RO2/src/livox_ode_multiray_shape.cpp` + `include/ros2_livox/livox_ode_multiray_shape.h` | **ODE 多射线形状**：`AddRay()` **每条射线** `new` 一个 `ODECollision` + `ODERayShape` ← **2.74 GiB 的根因** |
| `src/simulation/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp` + `include/ros2_livox/livox_points_plugin.h` | `gazebo::RayPlugin`：读 SDF、读 CSV、`CreateCollision("multiray")`、`reserve(samplesStep/downSample)`、逐条 `AddRay`、发布点云 |
| `include/ros2_livox/csv_reader.hpp` | 读 `mid360.csv` |
| `src/csv_main.cpp` | CSV 工具（离线） |
| `scan_mode/mid360.csv` | **800,001 行 / 17 MB** 的原始图案表 |
| `urdf/mid360.xacro` | `<samples>30000</samples>`、`<downsample>1</downsample>`、`<csv_file_name>…/mid360.csv</csv_file_name>`、`<topic>mid360</topic>` |
| `CMakeLists.txt` / `package.xml` | 依赖 `gazebo` + `gazebo_ros` |

**技术事实**：SDF `<lidar>` 规范不支持任意方向（§3.2），`GpuRays` 无相应 API（§3.3），
`RayQuery` 要 GL 上下文（§4.4），Jetty 的 `cpu_lidar` 也只用规则网格（§3.4）。
⇒ **必须自己写一个 gz system**；三条路线的取舍与接口细节见 **§3.7.3（R0/R1/R2/R3）**。

**工作量 L / 风险极高 / 渐进性差**：这是**唯一的"功能净损失"项**，
也是**决定"这次迁移是等价替换还是降级"的那一项**。

### 6.3 驱动插件（`libgazebo_ros_planar_move.so`）

现状：`<plugin name="mecanum_controller" filename="libgazebo_ros_planar_move.so">`，
`<remapping>cmd_vel:=cmd_vel_chassis</remapping>`、`<remapping>odom:=odom_ground_truth</remapping>`。

**为什么是 L（不是 S）**：
- `planar_move` 是**平面假运动学**（直接给速度、产生真值里程计），**不是**轮式动力学；
- 新 Gazebo 最接近的是 `ignition-gazebo-diff-drive-system`（**差速轮**）——
  对**麦轮底盘**语义不对（本仓是 `mecanum_controller`，另有 `mecanum_drive-system` 可用）；
- 而且 **`odom` 话题名 / `cmd_vel_chassis` 重映射 / TF 广播行为（`odom→base_link`）**
  都要在新系统里重新对齐 —— 本仓的 TF 契约是
  **`map→odom→base_link→base_link_fake`**，`base_link_fake` 在 **67** 处被引用（实测），
  **单发布者规则**（`docs/tf_interface_contract.md`）必须保持。

⇒ **工作量 L / 风险高 / 可渐进**（新 slot + 桥接 `cmd_vel_chassis`）。

### 6.4 `.world` → SDF

**实测：5 个 `.world` + 18 个 `model.sdf`**（全在 `src/simulation/rm_simulation/hzmi_rm_simulation/`）：

| 世界 | 体积 | 内容 |
|---|---|---|
| `world/RMUC2024_world/RMUC2024_world.world` | 8.0 K / 152 行 | 单一 STL：`model://RMUC2024_world/meshes/RMUC_2024.stl` |
| `world/RMUC2026_world/RMUC2026_world.world` | 12 K / 266 行 | 单一 STL：`model://RMUC2026_world/meshes/RMUC2026.stl` |
| `world/RMUL2024_world/RMUL2024_world.world` | 12 K / 309 行 | STL `RMUL_2024.stl` |
| `world/RMUL2024_world/RMUL2024_world_dynamic_obstacles.world` | 12 K / 359 行 | **`libobstacle{1..4}.so`** + `model://obstacles/obstacle*`（见 §6.5） |
| `world/RMUL2026_world/RMUL2026_world.world` | 12 K / 253 行 | STL `RMUL2026.stl` |

**好消息**：这些世界**几乎没有 Gazebo Classic 专有插件**（只有 dynamic_obstacles 那个有），
主要就是 `<include>` STL + `<physics>` + `<light>`。**主要工作量是 SDF 版本迁移与 `<plugin>` 头改写**
（`libgazebo_ros_*` → `ignition-gazebo-*`），**不是重写**。

**另一个坑（本仓已踩过，会原样复发）**：`model://` 的解析根在 Classic 靠
`gazebo_ros` 扫各包 `package.xml` 的 `<export><gazebo_ros gazebo_model_path=…/></export>` 拼出
`GAZEBO_MODEL_PATH`（见 `docs/gazebo_gui_troubleshooting.md` §3、`robot11_description/package.xml` 的长注释）。
**新 Gazebo 走 `GZ_SIM_RESOURCE_PATH` / `IGN_GAZEBO_RESOURCE_PATH`，机制完全不同**
⇒ `model://RMUC2026_world/meshes/RMUC2026.stl` 这类 URI 在迁移后**默认解析不到**，必须重新配环境变量。
（本次 PoC 用的是 `ign gazebo` CLI，日志里确实出现了 `/gazebo/resource_paths` 话题 —— 说明资源路径是独立一套。）

**工作量 M / 风险中 / 可渐进**（新世界目录，不动旧 `.world`）。

### 6.5 障碍物 C++ 插件（**容易被漏掉的一块**）

**实测 14 个 `obstacle*.cc`**（`src/simulation/rm_simulation/hzmi_rm_simulation/meshes/obstacles/obstacle_plugin/`），
外加 12 个 `obstacles/obstacle*/model.sdf`（引用 `libgazebo_ros_p3d.so`）。

它们**直接 include Gazebo Classic 的 C++ API**（一手）：

```cpp
#include <ignition/math.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
namespace gazebo { class Obstacles: public ModelPlugin { ... } }
// 用到的 Classic 类型：PoseKeyFrame ×14、PoseAnimationPtr ×14、PoseAnimation ×14 …（实测计数）
```

⇒ **这些必须重写为 `gz::sim::System`**（或干脆用新 Gazebo 的声明式动画/`TrajectoryFollower` 替代）。
**只影响 `RMUL2024_world_dynamic_obstacles` 这一个世界**，所以**可以完全延后**。

**工作量 L / 风险中高 / 可渐进**（世界级开关，与主链路解耦）。

### 6.6 launch 文件

| 文件 | 现状要点（实测） |
|---|---|
| `src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py` | **1300+ 行**。槽位 `world:=`（默认 `RMUL2026`）、`robot:=`、`gui:=`（**默认 `True`**）、`gazebo_offline:=`（默认 `False`）、`use_sim_time`、`lio_rviz`、`nav_rviz` + `lio/nav/localization/ground/planner/nav` 等。**`gui:=` 的语义是"起 gzclient"，RViz 由 `nav_rviz`/`lio_rviz` 单独控**（注释里明确：`gui:=False` = 只起 gzserver） |
| `src/simulation/rm_simulation/hzmi_rm_simulation/launch/rm_simulation.launch.py` | 世界/机器人启动、`_gazebo_model_path_setup`（OpaqueFunction，**必须在 include gzserver/gzclient 之前**跑） |
| `src/bringup/rm_nav_bringup/launch/cartographer_sim.launch.py` | 建图 slot |

上游入口：`/opt/ros/humble/share/gazebo_ros/launch/{gazebo,gzserver,gzclient}.launch.py`
→ 迁移后改用 `ros_gz_sim` 的 `gz_sim.launch.py` + `create`（`/opt/ros/humble/lib/ros_gz_sim/create` 已存在，实测）。

**关键映射**：

| Classic 概念 | 新 Gazebo |
|---|---|
| `gzserver.launch.py` | `ros_gz_sim` 的 `gz_sim.launch.py`（默认起 server+GUI；`gz_args` 控制） |
| `gzclient.launch.py` | 同一 `gz_sim` 进程树的 GUI（或 `gz sim -g` 独立进程） |
| `spawn_entity.py` | `ros_gz_sim` 的 `create` 可执行（实测存在） |
| `gui:=True/False` | **可原样保留**（对应"起不起 GUI"），语义不变 |
| `gazebo_offline:=` | 需重做（新 Gazebo 的 model 解析与 Fuel 机制不同） |
| `GAZEBO_MODEL_PATH` | **`GZ_SIM_RESOURCE_PATH` / `IGN_GAZEBO_RESOURCE_PATH`** |

**工作量 M / 风险中 / 可渐进**：建议加 **`sim_backend:=classic|gz`** 槽，
`classic` 为默认值 ⇒ **现有命令 `ros2 launch rm_nav_bringup bringup_sim.launch.py` 逐字不变。**

### 6.7 话题名 / TF / `ros_gz_bridge`（**风险最高的一类**）

**实测：本仓现存 `ros_gz_bridge` 配置文件 = 0 个**（`grep -rl ros_gz_bridge src/` 无命中）
⇒ 这部分是**纯新增**，不是改写。

**必须跨过的差异**：

| 项 | Classic 现状 | 新 Gazebo | 桥接动作 |
|---|---|---|---|
| LiDAR 点云 | `/livox/lidar/pointcloud`（自研插件发 `PointCloud2`） | `/poc/gpu_lidar/points`（`PointCloudPacked`）或 `/livox/lidar` | `ros_gz_bridge` 加 `PointCloudPacked`↔`PointCloud2` |
| `/scan` 来源 | `pointcloud_to_laserscan`（`src/perception/rm_perception/pointcloud_to_laserscan/`） | **不变**（p2l 仍订阅 ROS `PointCloud2`） | 保持桥后话题名一致 ⇒ **p2l 零改动** |
| `frame_id` | `<link>_<sensor>` 类 | `<model>::<link>::<sensor>`（**实测**） | ★ **`frame_id` 改写或 TF 补帧**，否则 `tf2` 全线报"frame 不存在" |
| `/clock` | `gazebo_ros` 发 | gz-transport `/clock` | 必须桥（`use_sim_time` 依赖它） |
| `/livox/imu` | `libgazebo_ros_imu_sensor.so`（`~/out` 重映射） | `imu-system` + bridge | 话题名对齐 |
| `cmd_vel_chassis` / `odom_ground_truth` | `<remapping>` | bridge | **`ros_gz_bridge` 不做 remapping**，需用 ROS 侧 `topic_tools`/节点重映射 |
| TF | **`map→odom→base_link→base_link_fake`**（`base_link_fake` **67 处引用**）| 新 Gazebo 的 `odom` TF 由 drive system 发 | **单发布者规则**（`tf_interface_contract.md`）**必须逐条复核**，否则 TF 树出现双发布 |

`ros_gz_bridge` 的用法（上游，`ros_gz_bridge` 包已装）：
```bash
ros2 run ros_gz_bridge parameter_bridge /TOPIC@ROS_MSG@GZ_MSG
# 例：/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```
（`/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge` 实测存在。）

**工作量 M / 风险高 / 可渐进**（bridge 是独立进程，可加可撤）。
**这是"最容易静默出错"的一类**：话题桥上了但 `frame_id` 不对 ⇒ RViz 看着"什么都没有"，
和本仓 `docs/gazebo_gui_troubleshooting.md` §2 描述的"现象①"是同一类坑。

### 6.8 感知链（linefit → pointcloud_to_laserscan → `/scan`）

- `src/perception/rm_perception/linefit_ground_segementation_ros2/`、`src/perception/rm_perception/patchwork_ground_segmentation/`
  —— 都是 **ROS 侧节点**，吃 `PointCloud2`、吐 `/segmentation/obstacle`，**与仿真后端无关**。
- `src/perception/rm_perception/pointcloud_to_laserscan/config/{laserscan_params,laserscan_params_sensor_frame}.yaml`
  + `launch/pointcloud_to_laserscan_launch.py` —— 同上。
- `bringup_sim.launch.py:1652` 起 `pointcloud_to_laserscan_node`。

⇒ **只要 bridge 后话题名与 `frame_id` 保持契约，感知链应该零改动。**
**工作量 S / 风险低 / 天然可渐进** —— 这也是**选择"用 bridge 保持话题契约"而不是"改下游"的核心理由**。

### 6.9 nav2 params 与地图资产

- **18 个 `nav2_params*.yaml`**（`src/nav/rm_navigation/rm_navigation/params/` 17 个 +
  `src/bringup/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml`）。
  其中 **10 个左右直接含 `base_link_fake`**（实测）。
  ⇒ 只要 TF 帧名与话题名不变，**这些文件应零改动**。**工作量 S / 风险低**。
- **地图资产**：`src/bringup/rm_nav_bringup/map/` **242 MB**、`src/bringup/rm_nav_bringup/PCD/` **118 MB**，
  格式是 `.pgm/.yaml/.pcd/.posegraph/.data` —— **与仿真后端完全无关**。
  **工作量 — / 风险无 / 完全不动**（守 `docs/map_assets.md` 的规则）。

### 6.10 `tools/scripts/**` 实验台

- `tools/scripts` 下非 `__pycache__` 文件 **125** 个；其中 **57** 个含 `gazebo|gzserver|gzclient`（实测）。
- 典型需改的：`tools/scripts/mapping/run_mapping_headless.sh`、
  `tools/scripts/localization/run_*_ab.sh`、`tools/scripts/diag/*`、
  `tools/scripts/control/{start_sentinel,watch_stack}.sh`、`tools/scripts/regress/*`。
- 本仓已知的**一个具体隐患**（`bringup_sim.launch.py:1169` 注释原话）：
  "本仓多个 bench/ab 脚本收尾会 `pkill -9 -x gzclient`（**全机**范围，不看归属）"。
  迁移后进程名变成 `ign-gazebo-gui` / `ign gazebo server`，
  **旧 `pkill` 会失效（安静地不杀）或误杀**；反之新脚本写 `pkill -x gz` 也可能误伤。

**工作量 M / 风险中 / 可渐进**：建议**所有 harness 加 `SIM_BACKEND` 环境变量分支 + 进程匹配改成"按 PID"**，
不要引入新的全机 `pkill`。**本次任务严格遵守了"只杀自己的 PID"**（`.tmp_gzsim/run_*.sh`
用 `ps` 前后差分求 PID 集合，只 kill 该集合，实测留下的遗留进程为 `(none)`）。

### 6.11 **"保持不动"清单**（**保住现有基准台的关键**）

| 类别 | 具体 | 理由 |
|---|---|---|
| Gazebo Classic 11 全部 | `gazebo` 11.10.2、`ros-humble-gazebo-ros-pkgs`、所有 `.world`/xacro | 与 Fortress **并存**（实测），不动就是零风险 |
| `ros-humble-ros-gz*` | 已装 0.244.26 | **已经装好，不需要动** |
| `.world` / `model.sdf` 全部 23 个 | 新建 `*_gz/` 目录另放 | 新世界与旧世界同源不同目录，可 A/B |
| 全部 xacro | 新增 `*_gz_sim.xacro` | `robot:=` 槽位加新值，旧值逐字不变 |
| 地图资产 | `map/` 242 MB + `PCD/` 118 MB | 后端无关 |
| nav2 params | 18 个 yaml | 只要契约不变 |
| 感知链全部包 | linefit / patchwork / pointcloud_to_laserscan | ROS 侧，与后端无关 |
| `libros2_livox.so` 与 `mid360.csv` | 原样保留 | **在 Fortress 路径上被替换之前，Classic 路径仍要用它** |
| `.gitignore`、`tools/scripts/{mapping,diag,regress}` 功能脚本、`src/localization/rm_localization/**`、`src/nav/rm_navigation/**/params`、`docs/{map_assets,session_digest,kalman_slam_tutorial,tilted_lidar_fidelity}.md` | — | **本任务明确不碰**（其它任务在改） |

---

## 7. 分阶段推荐计划（P0 → P4）

### 7.0 总体形状

```
P0 可行性(本 spike 已完成大部分) → P1 机器人 → P2 传感器系统 → P3 桥接 → P4 A/B 验收
```

**贯穿全过程的硬约束**：
- **默认值永不改**：新增 `sim_backend:=classic|gz`，`classic` 为默认 ⇒ 现有命令与基准台逐字不变。
- **新世界/新 xacro 放新目录、新 slot**，旧资产**只读不改**。
- **Harmonic/Jetty 一律进容器**，不与 `/opt/ros/humble` 混装（§2.3 实测会 `Remv` Classic）。

---

### P0 — 可行性（**本 spike 已覆盖大半**）

| 项 | 内容 |
|---|---|
| **入口判据** | 有 Fortress 可用（`ign gazebo --versions` 出 6.18.0）—— **本机已满足** |
| **出口判据** | ①`gz-gui` 能起窗口（无 GPU 也行）②`gpu_ray` 能出发真数据 ③知道任意图案"能不能"（**不能**，§3）④知道无 GPU 代价（§4） |
| **本 spike 状态** | ✅ **① ② ③ ④ 全部达成**（§4/§5） |
| **剩余待做** | 真机世界（RMUC2026 STL）加载一次，量加载耗时与 RSS；`ros_gz_bridge` 冒烟一条 `/clock` |
| **工作量** | 小（1–2 天） |
| **风险** | 低 |
| **不许做** | 不碰任何既有文件 |

### P1 — 机器人（把车放进新 Gazebo，**先不管 LiDAR 图案**）

| 项 | 内容 |
|---|---|
| **入口判据** | P0 全部出口达成 |
| **做法** | 新增 `src/bringup/rm_nav_bringup/urdf/sentry_robot_*_gz_sim.xacro`：`<gazebo>` 块换成 `ignition-gazebo-{imu,diff-drive|mecanum-drive,joint-state-publisher}-system`；LiDAR 先用**规则网格 `gpu_ray`（R0）**占位；`ros_gz_bridge` 先把 `/clock`、`/livox/imu`、`odom`、`cmd_vel_chassis` 桥起来 |
| **出口判据（必须全部满足）** | ①`ign gazebo` 起得来且 GUI 有窗口 ②TF 树仍是 **`map→odom→base_link→base_link_fake`**，**每个帧单发布者**（`ros2 run tf2_tools view_frames` + 发布者计数）③`/livox/imu` 频率≈100 Hz ④`cmd_vel_chassis` 能驱动底盘、`odom_ground_truth` 有合理数值 ⑤`/clock` 桥通、`use_sim_time` 一致 |
| **工作量** | 中（1–2 周） |
| **风险** | 中（TF 单发布者 / `cmd_vel` 重映射语义） |
| **可逆** | ✅ 全程 `sim_backend:=gz`，默认仍 `classic` |

### P2 — 传感器系统（**决定"等价迁移"还是"降级"**）

| 项 | 内容 |
|---|---|
| **入口判据** | P1 出口全部达成（车已经在新 Gazebo 里跑起来，只是 LiDAR 图案不对） |
| **必须做的决策** | **R0（规则网格，零代码，改图案）vs R1/R2/R3（自研 system，保图案）** —— 见 §3.7.3 |
| **若选 R1（推荐先做）** | 新增独立包（如 `rm_livox_gz`）：读 `mid360.csv` → 一组方向 → 每帧求交（`RayQuery` 或接 `RaycastData`）→ 发 `LaserScan` + `PointCloudPacked`；**`base_link_fake`/`frame_id`/`<topic>` 语义必须与 Classic 逐字对齐** |
| **出口判据** | ①**点云条数 = 30000**、10 Hz、`frame_id` 与 Classic 一致 ②**与 Classic 同一位姿下的点云做几何比对**（本仓已有 `tools/scripts/regress/linefit_replay_probe.py`、`robot11_livox_fov.py` 这类探针可复用）③**RSS/RTF 不劣于 Classic**（Classic 基线：gzserver 3.2 GiB、RTF 0.32–0.46） |
| **工作量** | **L**（R1）／L+（R3） |
| **风险** | **极高** —— 这是整个迁移的成败点 |
| **建议** | **先花 1 天做 R1 的"只出 30000 个方向、不算物理"的最小骨架**，确认"能发对格式 + 能读 CSV"再谈性能 |

### P3 — 桥接（全面对齐话题/TF 契约）

| 项 | 内容 |
|---|---|
| **入口判据** | P2 出真点云 |
| **做法** | 写 `ros_gz_bridge` 配置：`PointCloudPacked↔PointCloud2`、`LaserScan`、`Imu`、`Clock`、`Twist`、`Odometry`、`TFMessage`；对照 `docs/tf_interface_contract.md` 逐条复核 |
| **出口判据** | ①**下游一行不改**就能收到 `/scan`、`/livox/lidar`、`/livox/imu`、`odom`/TF ②`base_link_fake` **仍然只有一个发布者** ③`pointcloud_to_laserscan` → `/scan` 频率与 Classic 一致 ④RViz 里 TF 树与点云都正常（**不是"看着什么都没有"**） |
| **工作量** | 中 |
| **风险** | **高**（静默失败：桥上了但 `frame_id` 不对） |
| **可逆** | ✅ bridge 是独立进程 |

### P4 — A/B 验收（**证明不劣于基准台**）

| 项 | 内容 |
|---|---|
| **入口判据** | P3 出口全部达成 |
| **做法** | 复用 `tools/scripts/**` 既有 harness（**加 `SIM_BACKEND` 分支，不用全机 `pkill`**），同世界/同机器人/同参数跑 classic vs gz |
| **出口判据（量化）** | ①**RTF**：gz ≥ classic（classic 基线 0.32–0.46 @ robot11）②**RSS**：gz 显著优于 classic 的 3.2 GiB ③**建图/定位产物可比**：同一条 `mode:=mapping` 路线的 ATE / `/scan` 丢帧率 / GICP fitness 在容差内 ④**GUI 可用性**：在无 GPU 机器上开 GUI 不崩、操作不卡到不可用 ⑤**感知等价性**：点云几何比对通过 |
| **工作量** | 中 |
| **风险** | 中（口径不一致会得出假结论——见 §5.5 的口径警告） |
| **交付** | 一份带数字的迁移验收报告 + 默认值是否切换的决策 |

---

### 7.5 GPU-less 路径结论（**用户第三个问题**）

| 场景 | 可行方案 | 代价 |
|---|---|---|
| **有 GPU 的机器** | 原生（`ogre2` + GPU） | 基线 |
| **没有 GPU，但要 GUI** | **方案 A：`Xvfb` + `LIBGL_ALWAYS_SOFTWARE=1`（Mesa llvmpipe）** —— **本机实测跑通并出真数据** | **GUI 软件光栅化极吃 CPU：`ign gazebo gui` 实测 1269 %（≈12.7 核）；FSAA 从 8 降到 0** |
| **没有 GPU，也不要 GUI（CI/无头回归）** | **方案 B：`ign gazebo -s --headless-rendering`（软件 EGL）** —— **本机实测：无 DISPLAY、零报错、点云逐字节同尺寸** | server 侧很便宜（实测 **23.5 % 单核**） |
| **想要"完全不碰渲染器"** | **方案 C：只有 Jetty 有（`gz-sim-cpu-lidar` + `gz-physics`），但 Humble 没有 Jetty 的 `ros_gz` 包** | **本环境不可用** |

**⇒ 对"可能有的电脑没有 GPU"的答复**：
**能跑，不需要 GPU**；两条软件路径（虚拟屏 + 软件 GL / `--headless-rendering` 软件 EGL）**本机都实测通过**。
**真正的代价在 GUI 的 CPU 上（十几核）**，而不在"能不能跑"。
如果某台机器连软件 GL 都紧张，**`gui:=False` 只留 RViz** 是现成的降级档（Classic 那边同一个开关已在用）。
👉 **但注意：因为 Fortress 没有 CPU `ray`（§3.5），LiDAR ⇒ 渲染器 ⇒ 软件 GL 这条依赖在无 GPU 机器上是刚性的**，
**不存在"关掉渲染器只留 LiDAR"的省法**（除非上 Jetty，而 Humble 拿不到）。

### 7.6 **两个必答问题**

**Q1：迁移真的能消掉 30000 射线的成本，还是只是搬家？**

**答：真的消掉，不是搬家。** 机制上，
- **Classic 现状**：`livox_ode_multiray_shape.cpp` 的 `AddRay()` **每条射线**创建一个 `ODECollision` + 一个 `ODERayShape`，
  落在 glibc 堆上（`[heap]` 2.78 GiB），且**世界更新是单线程**的，射线求交必须在同一个线程里逐条做
  ⇒ **成本是 O(rays)，而且是串行的**（这正是本仓记录的"瓶颈是 Classic 世界更新单线程"）。
- **新 Gazebo `gpu_ray`**：射线方向被打包成一张纹理，**一次光栅化 pass** 出整张深度图
  ⇒ **内存 O(1)**（与射线数基本无关），**成本不随射线数线性增长**。
- **本机实测支持这个判断**：射线数 ×10.4，**RSS 只涨 5.8 MiB**，**RTF 从 1.0001 到 0.9999（不动）**。

⚠️ **诚实的限定**：**这不等于"整栈 RTF 会从 0.4 变成 1.0"。**
本仓的 0.32–0.46 是在**完整栈（SLAM/LIO/nav2 + 真机世界 + robot11）**下量的；
本次 PoC 是**静态微型世界**（物理成本≈0）。**能安全说的是**：
"**射线数不再线性放大内存与单线程世界更新成本**"；
"**整栈 RTF 提升多少**"必须靠 **P4 的 A/B 真跑**才能给数字。
另外：**如果最后必须走 R1/R2/R3 自研 system，那么"消除"的结论要重新评估** ——
自研 CPU 射线（R1）本质上是**把 ODE 的活搬到自己手里**，很可能**又变成 O(rays) 的 CPU 成本**
（虽然能省掉"每射线一个碰撞体"的内存，因为可以只存方向向量）；
**只有 R3（改造 OGRE2 `GpuRays` 的方向纹理）才保得住"光栅化 O(1)"这个优势**。
**这是 P2 阶段最重要的技术判断。**

**Q2：GUI 能留下吗？**

**答：能，而且不需要任何妥协。**
- **架构上**：新 Gazebo 的 GUI 是**独立进程**（实测：`ign gazebo gui` 与 `ign gazebo server` 是两个 PID），
  各持**独立的 gz-rendering 实例**，可以分别指定引擎
  （`--render-engine-gui` / `--render-engine-server`）。**"GUI 必须留着"这条诉求在新 Gazebo 上是原生支持的**，
  不存在"无头替代 GUI"的被迫选择。
- **实测上**：本机（**无 GPU**）在 Xvfb + llvmpipe 下拿到了 **1000×845 的真实 GUI 窗口**，
  GUI 插件（3D View / Entity Tree / World control / World stats / Transform control …）全部加载。
- **可保留的开关语义**：现有 `gui:=True/False` 可以**逐字保留**
  （`gui:=False` = 只起 server，与 Classic 的"只起 gzserver"语义一一对应）。
- **代价（要提前说清）**：无 GPU 机器上 GUI 的软件光栅化要吃 **十几核 CPU**（实测 1269 %），
  画质上 FSAA 从 8 被迫降到 0。**本仓 Classic 的 `gzclient` 本来就在软件光栅化下吃 444–609 % CPU**
  （`docs/gazebo_gui_troubleshooting.md` §6.3），所以这不是新问题，只是**更贵一些**。

---

## 8. 诚实：**本次没能验证的东西**

| # | 未能验证 | 原因 / 现状 |
|---|---|---|
| 1 | **Gazebo Classic 11 的精确 EOL 日期** | 官方 releases 页只列新 Gazebo（Fortress 起）。**未抓到 Classic 11 EOL 的一手 URL** ⇒ 本文不引用该日期。 |
| 2 | **gz-sim#1958 / #2645 的正文与结论** | GitHub issue 页在本沙箱只返回导航框架、正文被截断（`web_fetch` 返回 truncated）。**只确认了标题与存在性**，未逐句核对上游对"自定义扫描形状"的回复。 |
| 3 | **有 GPU 与无 GPU 的性能对照** | 本机**没有 GPU**（无 `/dev/dri`、NVIDIA 驱动未加载）⇒ 无对照基线。所有"无 GPU 数字"都是绝对值，不是"损失了多少"。 |
| 4 | **Harmonic 的实际表现** | 按任务要求"不丢现有 bench"，**没有安装**（`apt-get -s` 显示会卸载 Classic）。**未实测**。 |
| 5 | **真机世界（RMUC2026 / RMUL2026 STL）在新 Gazebo 里的加载时间与内存** | PoC 用的是微型世界。世界 mesh 2.19 MB 级，加载耗时未量。 |
| 6 | **机器人动力学（有质量/关节/摩擦）下的 RTF** | PoC 世界全 `static`，物理成本≈0。**P1 才能量**。 |
| 7 | **`ros_gz_bridge` 端到端（含 `PointCloudPacked` ↔ `PointCloud2`、`frame_id` 改写）** | 未搭。这是 P3 的核心工作。 |
| 8 | **`RayQuery` 在软件 GL（llvmpipe）下的逐条射线吞吐** | 未测。**这是 R1 路线可行性的关键未知数**——如果要自己写 system 用 `RayQuery` 打 30000 条，需要单独基准。 |
| 9 | **`ogre`（OGRE 1.9）与 `ogre2` 的差异** | 两者本机都装了，只用了 `ogre2`（Fortress 默认）。 |
| 10 | **`gz sim` 与 `ign gazebo` 的命令差异全集** | 只验证了 `gz sim` 在 Fortress 上不可用（`Invalid arguments`）。其它子命令（`gz topic` / `ign topic`）只验证了可用。 |
| 11 | **13 个 obstacle C++ 插件的行为** | 只做了静态清单与 API 依赖分析，**未运行**（只在 `RMUL2024_world_dynamic_obstacles` 用到）。 |
| 12 | **`base_link_fake` 的 67 处引用逐条复核** | 只统计了数量，未逐条判断哪些会受迁移影响。 |
| 13 | **`mid360.csv` 的 800,001 行与 `<samples>30000</samples>` 的精确对应关系** | 只确认了 CSV 行数/体积与插件读 `samples`/`downsample` 的代码位置，**未逐行读 `csv_reader.hpp` 验证抽样规则**。 |
| 14 | **`--headless-rendering` 是否真的走了 EGL** | 实测"无 DISPLAY 能跑通 + 出真数据"是硬事实；但**没抓 EGL 调用栈**证明它用的是 EGL 而非别的机制。 |
| 15 | **上游 issue 中"是否有人在改任意图案支持"** | 未找到明确的 roadmap/PR。 |

---

## 附录 A 复现命令（全部在 `.tmp_gzsim/`，全程不动功能代码）

```bash
cd <repo>

# ---- 环境事实 ----
apt-cache policy ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
apt-cache policy ros-humble-ros-gzgarden ros-humble-ros-gzharmonic libgz-sim8-dev
apt-get download --print-uris libgz-sim8-dev ros-humble-ros-gzharmonic   # 只看 URI/大小
apt-get -s install ros-humble-ros-gzharmonic | grep -i Remv              # ★ 看会不会删 Classic
ign gazebo --versions; gz sim --versions        # 后者在 Fortress 上 "Invalid arguments"
glxinfo -B                                       # 需要 DISPLAY

# ---- 规范全文（一手） ----
grep -nE '<element name=|<attribute name=' /usr/share/sdformat12/1.9/lidar.sdf

# ---- PoC：带 GUI ----
bash .tmp_gzsim/run_poc.sh gui 50                # 内部起 Xvfb :99 + LIBGL_ALWAYS_SOFTWARE=1
cat .tmp_gzsim/run_gui/windows.txt               # 应看到 "Gazebo GUI" 1000x845
cat .tmp_gzsim/run_gui/stats.txt                 # real_time_factor
cat .tmp_gzsim/run_gui/topics.txt                # /poc/gpu_lidar, /poc/gpu_lidar/points, /poc/imu

# ---- PoC：30000 射线标定 + 无 DISPLAY 无头 ----
python3 .tmp_gzsim/gen_world.py 360  8 .tmp_gzsim/w_2880.sdf
python3 .tmp_gzsim/gen_world.py 1000 30 .tmp_gzsim/w_30k.sdf
bash .tmp_gzsim/run_scale.sh rays2880_server        .tmp_gzsim/w_2880.sdf server   xvfb 70
bash .tmp_gzsim/run_scale.sh rays30k_server         .tmp_gzsim/w_30k.sdf  server   xvfb 70
bash .tmp_gzsim/run_scale.sh rays30k_headless_nox   .tmp_gzsim/w_30k.sdf  headless nox  60

# ---- 结果核对 ----
grep -E '^(width|height|point_step)' .tmp_gzsim/scale_rays30k_server/points.txt   # 1000 x 30
tail -3 .tmp_gzsim/scale_rays30k_server/stats.txt                                 # RTF ~1.0
```

**注意**：
- 沙箱里 `/tmp` 是**每次 bash 调用独立**的 ⇒ **Xvfb 与 `ign gazebo` 必须在同一次调用里起**，脚本已按此写。
- `$HOME/.ignition` **只读会导致 GUI 直接 abort**（§5.1）⇒ 脚本把 `HOME` 重定向到 `.tmp_gzsim/home`。
- 脚本收尾用 **`ps` 前后差分求 PID 集合**只 kill 自己的进程，**不做全机 `pkill`**（实测遗留 = `(none)`）。

## 附录 B 引用 URL 清单

**官方文档**
- 版本配对矩阵：<https://gazebosim.org/docs/latest/ros_installation/>
- Gazebo 各版本 EOL：<https://gazebosim.org/docs/all/releases/>
- 传感器教程（`gz sim`）：<https://gazebosim.org/docs/latest/sensors/>
- ROS 2 各发行版 EOL（REP-2000）：<https://www.ros.org/reps/rep-2000.html>

**上游源码 / 规范**
- gz-sim10 `cpu_lidar` 系统：<https://github.com/gazebosim/gz-sim/tree/gz-sim10/src/systems/cpu_lidar>
- gz-sim10 `CpuLidar.cc`：<https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/systems/cpu_lidar/CpuLidar.cc>
- gz-sensors10 `CpuLidarSensor.cc`（`GenerateRays` / `unitVectors`）：<https://github.com/gazebosim/gz-sensors/blob/gz-sensors10/src/CpuLidarSensor.cc>
- gz-rendering6 `GpuRays.hh`（只有规则网格 API）：<https://github.com/gazebosim/gz-rendering/blob/gz-rendering6/include/gz/rendering/GpuRays.hh>
- gz-rendering6 `RayQuery.hh`（逐条任意方向，但依赖 `Scene`）：<https://github.com/gazebosim/gz-rendering/blob/gz-rendering6/include/gz/rendering/RayQuery.hh>
- gz-sim6 `System.hh`（`ISystemConfigure/PreUpdate/PostUpdate`）：<https://github.com/gazebosim/gz-sim/blob/gz-sim6/include/gz/sim/System.hh>
- SDFormat `<lidar>` 规范：<https://sdformat.org/spec?ver=1.9&elem=lidar>（该页是 JS 渲染，**权威文本用本机 `/usr/share/sdformat12/1.9/lidar.sdf`**）

**上游 issue（自定义图案）**
- <https://github.com/gazebosim/gz-sim/issues/1958> Custom lidar sensor
- <https://github.com/gazebosim/gz-sim/issues/2645> Build my own Lidar Sensor with custom scanning shape
- <https://github.com/gazebosim/gz-sim/issues/2595> Gazebo in Windows Docker cannot use Nvidia GPU, falls back to using CPU

**本仓内部真值**
- `docs/gazebo_gui_troubleshooting.md` §6.3 / §6.4.1（Classic 基线：gzclient 483.7 MiB / 444–609 %；
  gzserver 3199.9 MiB / 66.9 %；Livox 插件 +2.74 GiB ≈ 94.8 KiB/条，根因 `AddRay()`）
- `docs/tf_interface_contract.md`（TF 契约、单发布者）
- `docs/worlds.md`（场地资产）、`docs/robot_models.md`（模型槽位）、`docs/ground_segmentation_slots.md`（感知链）

---

## 9. 第二枪（`.tmp_gzsim2/`）：R0/R1/R3 收口 —— 修掉一个"看起来通过"的假数字、量出 CPU 射线、回答 GUI 与 10 Hz 的关系

> 本节是 §5（`.tmp_gzsim/`，第一枪）的续篇。第一枪用的是**微型世界 + 规则网格**，只回答了
> "Fortress 能不能跑、30000 条射线贵不贵"。第二枪换到**真机世界**（`RMUC2026.stl`）+ **真 MID-360 图案**
> （`mid360.csv`）+ **真机器人位姿**，并补上第一枪明确没测的四件事（§8 的 #5 / #6 / #8 / #13）：
>
> 1. **R1（纯 CPU 逐射线 BVH）到底多少 µs/ray**，1/2/4/8/16 线程怎么摊，30k@10 Hz 够不够；
> 2. **R0（Fortress `gpu_ray` 规则网格）在 30k / 2880 × headless / GUI 下的真实数字**；
> 3. **GUI 把点云从 10 Hz 拖到 8.35 Hz 的机制**（是 RTF 掉了，还是 publisher/时钟耦合错了）；
> 4. **R3（改造 OGRE2 `GpuRays` 的方向表）在源码层是否现实**、要动几个包、能不能上游。
>
> 一句话结论：**R1 在"没有 GPU 的电脑"上是可行的（30k 条 12.5–29.4 ms/帧 @1 线程，
> 4 线程 5.6–13.6 ms，预算 100 ms 有 3.4–18× 余量）；R0 已经能跑且便宜，但它给不了 MID-360 的图案；
> R3 技术上成立、但必须 fork 2–3 个包且精度受立方图重采样限制；GUI 与"10 Hz 点云"在 sim 时间里
> 完全兼容，代价是 wall-clock 吞吐掉到 0.82–0.84×。**
>
> ⚠️ **同时必须记一笔**：第一枪留下的 R1 数字（`us_per_ray=0.348`、`fits_10Hz=YES`）是
> **无效的**——世界 STL 是毫米、探针按米算，**0% 命中**却"通过"了。见 §9.1.2。

### 9.0 本节与 §5 的口径差异（**别把两节的数字混着比**）

| | §5（第一枪，`.tmp_gzsim/`） | 本节（第二枪，`.tmp_gzsim2/`） |
|---|---|---|
| 世界 | 微型世界（1 个地面 + 2 面墙） | **真机世界** `RMUC2026.stl`（2,295,084 B，45,900 三角面，137,700 顶点，1 个 submesh） |
| 世界真实尺寸 | 未量 | **29.150 × 16.050 × 1.730 m**（mesh 原始 bbox 29150×16050×1730 **mm**） |
| 传感器图案 | 规则网格 1000×30 | **真 `mid360.csv` 的 30,000 行滚动窗口**（方位 0–360° 均匀、俯仰 −7.16°…+52.16°，85.3% 在地平线以上） |
| 测的东西 | 端到端 RTF / RSS / CPU（`gpu_ray`） | ①**射线求交本身**（CPU BVH，R1）②端到端（`gpu_ray`，R0）③源码（R3） |
| 能否互换 | **不能**：本节 R1 是"求交成本"，§5 是"整机世界更新成本" | |

### 9.0.1 本节关闭 / 未关闭的 §8 条目

| §8 # | 第一枪未验证 | 本节状态 |
|---|---|---|
| #5 | 真机世界（RMUC2026 STL）的加载时间与内存 | **关闭（部分）**：`gz-common::MeshManager` 加载 **16.4–16.9 ms**（45,900 面 / 2.29 MB），BVH 建树 **22.1–24.0 ms**；world 真实尺寸 **29.150×16.050×1.730 m**。整机 RSS 见 §9.2。 |
| #6 | 有质量/关节/摩擦的动力学下的 RTF | **关闭（部分）**：P1-lite（含 `DiffDrive` + `Physics` + `IMU`）headless RTF **0.999**（§9.3.1）。完整栈（SLAM/LIO/nav2）仍未测。 |
| #8 | `RayQuery` 在 llvmpipe 下的逐条射线吞吐 | **未关闭**：第一枪那组 `us_per_ray=0.348 / engine_ms=117.3` 与 CPU BVH 的 0-hit 数字**同源**（mm/m 单位错），**同样无效**；本轮没重测（见 §9.6 #5）。**但 R1 的答案已由 CPU BVH 给出**（§9.1），`RayQuery` 不再是 R1 的唯一选项。 |
| #13 | `mid360.csv` 的 800,001 行与 `<samples>30000</samples>` 的对应关系 | **关闭**：`csv_reader.hpp:20-45` 逐行读三元组；`livox_points_plugin.cpp:407-427` 是**滚动窗口** `[currStartIndex, currStartIndex+30000)`，`index = k % 800000` ⇒ **26.7 帧一个循环**。本节用 `RAY_START` 复现了第 0/10/20 帧（命中率 16.70/17.05/16.72 %）。 |

### 9.1 R1：纯 CPU 逐射线 BVH（**本节最硬的数字**）

#### 9.1.1 方法（被测对象 / 坐标 / 射线 / 线程 / 重复次数）

**探测器**：`.tmp_gzsim2/cpu_ray_bvh.cc`（438 行 C++17，依赖 `ignition-common4` 的 `MeshManager`
加载 STL、`bullet` 的 `btBvhTriangleMeshShape` 做求交、`ignition-msgs8` 只为复用常量）。
它**不碰渲染器、不需要 GL/DISPLAY**，是 R1 路线（"自己写一个 gz system 逐条打射线"）的成本上界估计。

**世界坐标变换**（与 `world/RMUC2026_world/model.sdf:35-40` 完全一致）：

```
world = mesh_local × 0.001 + (0, 0, +1.6413436)
```

- `0.001`：STL 是**毫米**；
- `+1.6413436`：mesh 局部里**可行驶地面顶面**在 z = −1.64134362793，抬起来让地面落在 world z = 0
  （mesh bbox 的 z 是 −1.84134…−0.11134 m，即底板 0.2 m 厚 + 场地高 1.53 m）；
- 变换后实测 bbox：`(-14.575, -6.401, -0.200) … (14.575, 9.649, 1.530)`，即 **29.150 × 16.050 × 1.730 m**。

**传感器位姿**：`RMUC2026` 的出生点是 `(10.925, 2.525, 0.2)`（`rm_simulation.launch.py:134-139`），
`base_link → livox_frame` 是 `xyz="0.12 0 0.175"`（`simulation_waking_robot.xacro:17`），落地后
雷达光心约在 **0.26 m**。所以标称位姿取 **(10.925, 2.525, 0.26)**，另测 4 个场景（§9.1.4）。
`max_range` 取 **200 m**（Classic 的 `mid360.xacro:9` 就是 `laser_max_range=200.0`），
另做 10/30/70/200 m 敏感性（§9.1.4 末）。

**射线**：直接读 `src/simulation/rm_simulation/livox_laser_simulation_RO2/scan_mode/mid360.csv`，
方向约定与 Classic 插件**逐字一致**（`livox_points_plugin.cpp:420-423`）：

```
el = 90° − zenith_csv ,  dir = (cos el·cos az, cos el·sin az, sin el)
```

**只取连续 30,000 行**，因为 Classic 插件是**滚动窗口**（`livox_points_plugin.cpp:407-427`：
`end_index = currStartIndex + samplesStep; currStartIndex += samplesStep; index = k % maxPointSize`），
80 万行 / 3 万 = **26.7 帧一个循环**。本探针用 `RAY_START` 复现第 0/10/20 帧（§9.1.4）。
⚠️ **不能只取前 4000 行**：CSV 是**按时间排序**的，前 4000 行几乎全在高仰角带
（前 300 行的 zenith 是 37.8°–46.2°），据此算命中率会得到 0% 的假结果——这一点我在扫描位姿时踩过。

**线程**：每个线程各自一份 `btCollisionDispatcher/btDbvtBroadphase`，**共享只读的
`btBvhTriangleMeshShape`**（BVH 只建一次，22–24 ms）。每档跑 5 次取**最小值**（min 对噪声最稳）。

**构建**：

```bash
g++ -O2 -std=c++17 -Wall cpu_ray_bvh.cc -o cpu_ray_bvh \
    $(pkg-config --cflags --libs ignition-common4 ignition-common4-graphics ignition-msgs8 ignition-utils1) \
    $(pkg-config --cflags --libs bullet) -lpthread
```

（`ignition-common4-graphics` 必须显式加：`MeshManager` 在 graphics 库里，`ignition-common4.pc`
的 `Libs` 不含它；`ignition-utils1` 是 `Mesh.hh` 里 `ImplPtr.hh` 的依赖。）

#### 9.1.2 ⚠️ 上一枪的 R1 数字**无效**：mm/m 单位 bug，以及"0 命中会静默通过"

第一枪跑的是 `scale=1.0`（把毫米 STL 当米），于是：

```
SCENE bounds=(-14575.00 -6400.66 -1841.34)-(14575.00 9649.34 -111.34) origin=(0.00 1624.34 -1841.04)
SANITY ray0 dir=(-0.011 -0.613 0.790) hit=0 dist=-1.000
RESULT threads=1 rays=3000 done=3000 hit=0.0% total_ms=1.04 us_per_ray=0.348 frame_30k_ms=10.4 fps_30k=95.8 fits_10Hz=YES
```

逐条对账：

| 症状 | 根因 |
|---|---|
| `bounds` 是 `±14575`（不是 ±14.575） | **世界是 29.15 mm 宽**（不是 29.15 m） |
| `origin.z = -1841.04` | 传感器"离地 0.26"落在 mm 世界里 = 离地 0.26 mm |
| `max_range=70.0` 但几乎打不到东西 | 70 单位 = **70 mm**，比场地小 400 倍 |
| `hit=0.0%` 却打印 `fits_10Hz=YES` ⚠️ | **没有"几何有效性"断言**：射线全不中 ⇒ 求交立即返回 ⇒ 计时**异常地快** ⇒ 看起来"通过" |
| 唯一命中的那次 `dist=1233.000` | 在"把原点抬到 AABB 上方"的实验里打到 1233 **mm**，正好说明单位是毫米 |

**这就是为什么必须把"几何正确"变成硬闸门**：本节的探针在打印任何计时之前先打一组 sanity 射线，
**任何一条不满足就 `exit 9` 并拒绝给结论**：

| sanity 射线 | 期望 | 作用 |
|---|---|---|
| 正下方 `(0,0,-1)` | **命中，距离 ≈ 传感器高度 ±0.15 m** | 一举抓住 mm/m、z 偏移、mesh 未变换三类错误 |
| 正上方 `(0,0,+1)` | **不命中**（这个场地 mesh 没有天花板） | 反向断言：`expect=miss` 也是断言，避免"全 miss"被当成正常 |
| 俯角 5° | 命中 ≈ `z/sin5°`（z=0.26 ⇒ 2.983 m），允许 −30% 以内（可能有台阶/矮物） | **与距离相关的**标尺校验，独立于墙在哪 |
| 四个水平方向 | ≥1 个命中且距离在 0.5–40 m（这个场地南边是敞开的，允许逃逸） | 抓整体尺度/朝向错误 |
| `RAY_ENCLOSED=1` | 只放宽"上方必 miss / 5° 必打地面"两条（封闭场景本来就有顶和近墙） | **正下方的断言永不放宽** |

> **教训（值得写进任何 benchmark 的规范）**：**"0 命中"的探针会静默通过每一条性能断言。**
> 任何"每帧 N 条射线"的 benchmark，必须同时断言**命中率非零**且**命中距离几何合理**
> （至少一个"向下打地面 = 传感器高度"的标尺）。否则单位错 1000 倍也能得到漂亮的 µs/ray。

#### 9.1.3 修正后的有效性证据（sanity + 独立暴力求交交叉验证）

标称位姿 `(10.925, 2.525, 0.26)`、200 m、30k 窗口 0：

```
SCENE mesh=.../RMUC2026.stl submeshes=1 verts=137700 tris=45900 load_ms=16.4 bvh_ms=22.7 scale=0.001000 zoff=1.641344
WORLD bounds_m=(-14.575 -6.401 -0.200)-(14.575 9.649 1.530) size_m=(29.150 16.050 1.730)
SENSOR pose=(10.925 2.525 0.260) max_range=200.0 reps=5
SANITY down       dir=(+0.000 +0.000 -1.000) hit=1 dist=0.260 expect=hit~sensor_z PASS=YES
SANITY up         dir=(+0.000 +0.000 +1.000) hit=0 dist=-1.000 expect=miss(no ceiling) PASS=YES
SANITY graze-5deg dir=(+0.996 +0.000 -0.087) hit=1 dist=2.751 expect=hit~z/sin(5) PASS=YES
SANITY +x         hit=0  SANITY -x hit=1 dist=6.925  SANITY +y hit=0  SANITY -y hit=1 dist=3.808
SANITY overall=PASS fails=0
HITDIST threads=1 p01=2.17 p05=2.49 p50=3.74 p95=7.41 max=26.64 zero_lt_0.1m=0
```

- **正下方 = 0.260 m**，与传感器高度**完全一致** ⇒ 尺度、z 偏移、mesh 变换三者同时被钉住；
- 命中距离分布 `p05=2.49 / p50=3.74 / p95=7.41 / max=26.64 m` ⇒ 打的是 2–7 m 的地面/结构 + 26.6 m 的远角，
  **没有一条 < 0.1 m**（`zero_lt_0.1m=0`）⇒ 不是"传感器埋在实体里"的退化命中；
- 5° 俯角打 2.751 m（理论 2.983 m，差 7.8%，是场地上一处矮台阶/坡），在 ±30% 容差内；
- 水平 2/4 命中（+x / +y 打空，因为这个场地 0.26 m 高度上南/东边界是敞开的），符合目视几何。

**独立交叉验证**（防止"BVH 和我自己的射线构造一起错"）：用 Python + numpy 写一份**暴力
Möller–Trumbore**（45,900 个三角面 × 35 条射线，同一 STL、同一变换、同一 CSV 行），与 C++ 的
`ClosestRayResultCallback` 结果逐条比对：

```
   idx   C++ dist    py dist    delta
  6156     5.1178     5.1178  0.00000
  ...
 12599     6.8006     6.8006  0.00000
     0    -1.0000    -1.0000  0.00000   （未命中，两边一致）
mismatches: 0 of 35      （30 条命中 + 5 条未命中，最大 |Δ| = 1e-5 m）
```

#### 9.1.4 结果：4 个场景 × 1/2/4/8/16 线程（**有效数字**）

全部：`mid360.csv` 连续 30,000 行（第 0 帧）、`max_range=200 m`、z = 0.26 m、min-of-5。
`µs/ray` = 该线程数下总时间 ÷ 30,000；`frame_30k_ms` = 一帧 30k 条射线的墙钟时间；
`fits` 判据 = 预算 100 ms/帧（10 Hz）。

| 场景 | 位姿 (x,y,z) | **命中率** | 1 线程 µs/ray → 帧 ms | 2 线程 | 4 线程 | 8 线程 | 16 线程 |
|---|---|---|---|---|---|---|---|
| **A1 开阔（南边界）** | (4, −6, 0.26) | **9.82 %** | 0.484 → **14.52 ms** | 0.279 / 8.36 ms (1.74×) | 0.201 / 6.03 ms (2.41×) | 0.113 / 3.40 ms (4.27×) | 0.066 / 1.99 ms (7.30×) |
| **A2 开阔（西侧）** | (−10, 0, 0.26) | **18.08 %** | 0.471 → **14.14 ms** | 0.294 / 8.83 ms (1.60×) | 0.207 / 6.20 ms (2.28×) | 0.113 / 3.39 ms (4.17×) | 0.068 / 2.05 ms (6.91×) |
| **B 标称出生点** | (10.925, 2.525, 0.26) | **16.70 %** | 0.417 → **12.50 ms** | 0.261 / 7.84 ms (1.59×) | 0.185 / 5.56 ms (2.25×) | 0.107 / 3.21 ms (3.90×) | 0.063 / 1.89 ms (6.61×) |
| **C 结构化（命中重）** | (12, 8, 0.26) | **42.81 %** | 0.578 → **17.34 ms** | 0.338 / 10.13 ms (1.71×) | 0.265 / 7.95 ms (2.18×) | 0.152 / 4.56 ms (3.80×) | 0.089 / 2.68 ms (6.46×) |
| **D 封闭上限（不可达）** | (10, −2, 0.26) | **100.00 %** | 0.978 → **29.35 ms** | 0.628 / 18.84 ms (1.56×) | 0.452 / 13.57 ms (2.16×) | 0.246 / 7.39 ms (3.97×) | 0.140 / 4.21 ms (6.97×) |

> **D 的定位**：`(10,−2)` 的顶只有 0.14 m 高（`SANITY up hit=1 dist=0.140`），**真机（含雷达外壳）塞不进去**，
> 所以它是"全部命中"的**成本上限**，不是可用位姿；它的 sanity 按设计会 FAIL（`RAY_STRICT=0` 放过）。
> 本场地的**可用位姿**命中率上限就是 **C 的 42.8 %**（再高只能钻进 0.4 m 高的空腔）。

**线程伸缩（30k 短批 vs 10 帧摊销）**：

| 线程 | A1 30k | C 30k | A1 10 帧（300k 射线） | C 10 帧 |
|---|---|---|---|---|
| 1 | 1.00× | 1.00× | 1.00× (0.455 µs/ray) | 1.00× (0.593 µs/ray) |
| 2 | 1.74× | 1.71× | 1.89× | 1.91× |
| 4 | 2.41× | 2.18× | 2.73× | 2.98× |
| 8 | 4.27× | 3.80× | 4.77× | 4.27× |
| 16 | 7.30× | 6.46× | **8.36×** | **7.25×** |

- **4 线程 5–8 ms/帧、8 线程 3–4.6 ms/帧**，即 10 Hz 预算的 **12–30 倍余量**；连**单线程**都有 3.4–8× 余量。
- 伸缩**亚线性**是三个原因叠加：①30k 一帧只有 12–17 ms，16 线程的**线程创建/回收 ~0.8 ms** 占比可观
  （10 帧摊销后 16 线程从 6.6–7.0× 升到 7.3–8.4×）；②本机是 **i7-14700HX（8 P-core + 12 E-core，28 线程）**，
  超过 8 线程就落到 E-core；③每线程一个 `btDbvtBroadphase` + 共享 BVH 的**内存带宽**瓶颈。
  ⇒ 真做 R1 时应当用**常驻线程池**（不要每帧建线程），4–8 个工作线程就够。

**敏感性**（标称位姿、1 线程）：

| `max_range` | 10 m | 30 m | 70 m | 200 m |
|---|---|---|---|---|
| µs/ray | 0.391 | 0.405 | 0.433 | 0.427 |
| 命中率 | 16.48 % | 16.70 % | 16.70 % | 16.70 % |

⇒ **射程几乎不影响成本**（BVH 对逃逸射线剪枝很快），所以"为了省钱把 200 m 改小"没有意义；
反过来，**200 m 的配置不用改**。

| CSV 滚动窗口 | 第 0 帧 | 第 10 帧（start=300000） | 第 20 帧（start=600000） |
|---|---|---|---|
| 命中率 | 16.70 % | 17.05 % | 16.72 % |
| µs/ray（1 线程） | 0.424–0.462 | 0.426–0.431 | 0.437–0.456 |

⇒ **窗口之间命中率稳定在 16.7–17.1 %**，不是"第 0 帧特殊"。

#### 9.1.5 命中率不是成本的主因；"命中射线 vs 未命中射线"分账（1 线程）

| 场景 | 混合 µs/ray | **只打命中的射线** | **只打未命中的射线** | 命中/未命中 |
|---|---|---|---|---|
| A1（9.82 %） | 0.484 | **1.985** (n=2,945) | 0.326 (n=27,055) | 6.1× |
| A2（18.08 %） | 0.471 | **1.382** (n=5,423) | 0.281 (n=24,577) | 4.9× |
| B（16.70 %） | 0.417 | **1.392** (n=5,009) | 0.264 (n=24,991) | 5.3× |
| C（42.81 %） | 0.578 | **0.973** (n=12,844) | 0.369 (n=17,156) | 2.6× |
| D（100 %） | 0.978 | **1.008** (n=30,000) | —（0 条） | — |

三条结论，**和"命中越多越慢"的直觉不完全一样**：

1. **单条命中射线比未命中射线贵 2.6–6.1 倍**：`ClosestRayResultCallback` 只在**命中**时才做
   精确三角面求交（未命中只走 AABB 剪枝）；
2. 但**整帧成本并不由命中率单调决定**：A1 的命中率只有 9.82 %，成本（0.484）却**高于**命中率 18.08 % 的 A2（0.471）。
   原因是 A1 的命中是**长距离掠射地面**（`p95=12.48 m`，见 HITDIST），穿过的 BVH 节点更多、
   命中的三角面更多；A2 的命中集中在 1.9–4 m。**成本 ≈ Σ(节点访问 + 三角面测试)**，不是命中条数；
3. 所以**"命中重"和"未命中重"两个场景都要报**（本节 A1/A2 vs C vs D）：可用位姿区间
   **9.8 % → 42.8 % 命中**对应的成本是 **0.417–0.578 µs/ray（1 线程）**，只差 1.4 倍；
   真正的成本上限（100 % 命中、0.978 µs/ray）**比标称位姿贵 2.3 倍**。

#### 9.1.6 R1 的判定

**✅ 可行。** 在"真世界 + 真 30k 图案 + 真位姿"下：**1 线程 12.5–29.4 ms/帧**，
**4 线程 5.6–13.6 ms/帧**，**预算 100 ms 有 3.4–18× 余量**，且**完全不碰渲染器**
（⇒ 无 GPU 也能跑；连 `--headless-rendering`/EGL 都不需要，CI 里不会再出现
"渲染器起不来 ⇒ LiDAR 没数据"的耦合）。

**必须一起说的三条边界**：

1. 这里测的是**射线求交**，不含 30k 点/帧的**消息构造 + 序列化 + 传输**
   （480 KB LaserScan + 960 KB PointCloudPacked，见 §9.2）。R1 与 R0 共用同一条发布路径，
   所以 **R1 相对 R0 的增量就是这 0.5–13 ms**；
2. BVH 是**静态世界建一次**（22–24 ms）。**动态障碍**要 refit/rebuild，本节**没测**；
3. 只算**世界 mesh**，射线会穿过机器人自身（真系统必须处理 self-hit / 传感器安装件）。

### 9.2 R0：30k / 2880 的真实数字（headless 与 GUI）

三个运行都在**同一个盒子**上，世界是 `.tmp_gzsim2/w_30k.sdf` / `w_2880.sdf`
（规则网格 `gpu_ray`，1000×30 与 360×8），`-r -v 3`，各跑 ≈25 s。RTF 有两个口径：
**报**= `/stats` 的 `real_time_factor`（瞬时/窗口值，末次采样）；**均**= `sim_time / real_time`
（含世界加载与 spawn，所以偏低）。两者都列，别混用。

| 运行 | 射线 | 模式 | **RTF 报** | sim/real（均） | 迭代 | **CPU** | **RSS** | **scan Hz** | **points Hz** | `/clock` Hz |
|---|---|---|---|---|---|---|---|---|---|---|
| `run_r0_30k_headless` | 1000×30 = 30,000 | `-s --headless-rendering` | **0.9994–1.0** | 23.228 / 25.406 = 0.914 | 23,228 | **24.7 %**（单 ruby） | **375 MB** | **10.064** | 25.218 ⚠️ | **1000.01** |
| `run_r0_30k_gui` | 30,000 | `-r -v 3`（server+GUI，Xvfb 1280×800） | **0.77–0.88**（末次 0.8325） | 19.745 / 25.348 = 0.779 | 19,745 | **1321 %**（GUI 1295.8 + server 21.5 + Xvfb 4.0） | server 363 MB + **GUI 588 MB** = 1.09 GB | **8.542** | 20.910 ⚠️ | **841.53** |
| `run_r0_2880_headless` | 360×8 = 2,880 | `-s --headless-rendering` | **1.0** | 23.234 / 25.394 = 0.915 | 23,234 | **23.4 %** | **365 MB** | **10.000** | 28.168 ⚠️ | 999.83 |

载荷与元数据（**扫描与点云是同一份数据的两种编码**）：

| 话题 | 类型 | 字节/帧 | 结构 | `frame_id`（gz 侧） | 桥到 ROS 后 |
|---|---|---|---|---|---|
| `/poc/gpu_lidar` | `ignition.msgs.LaserScan` | **480,193**（480 KB） | `count=1000 vcount=30 ranges=30000` | `sensor_rig::link::gpu_lidar` | `sensor_rig/link/gpu_lidar` |
| `/poc/gpu_lidar/points` | `ignition.msgs.PointCloudPacked` | **960,143**（960 KB） | `width=1000 height=30 point_step=32 row_step=32000` | `frame_id=sensor_rig::link::gpu_lidar` | 同上 |
| `/livox/lidar`（P1-lite） | LaserScan | 480,221 | `count=1000 vcount=30 ranges=30000` | `p1_robot::base_link::livox_lidar` | `p1_robot/base_link/livox_lidar` |
| `/livox/lidar/points`（P1-lite） | PointCloudPacked | 960,148 | `width=1000 height=30 point_step=32` | 同上 | 同上 |

⇒ 10 Hz 下 **≈14 MB/s** 进 `ros_gz_bridge`（480 KB + 960 KB 每帧，两路都订阅时），
`ros_gz_bridge` 进程本身实测 **8.9 % CPU / 70 MB**（`p1_gui/cpu.txt`，第一枪）——**这不是瓶颈**。

**`frame_id` 约定**（迁移必改项，§6.7 的实测补充）：Fortress 默认是
`<model>::<link>::<sensor>`，**`ros_gz_bridge` 把 `::` 换成 `/`**（实测：
`frame_id: p1_robot/base_link/livox_lidar`）。所以 TF 树里的 frame 名必须从 Classic 的
`livox_frame`/`livox_lidar` 改成 `p1_robot/base_link/livox_lidar`（或在 bridge 侧重写）。

⚠️ **表里 points Hz 那三列不要当真**：`25.218 / 20.910 / 28.168` 都是**另一个 15 s 窗口**里
顺序测的（`run_r0.sh:55-57` 是先 points 后 scan，不是同时），而且今天复现出的
"每周期多发 2.2–2.6 条"现象会把点云速率抬高（见 §9.3.4）。**同一时刻**同时订阅两路的 P1-lite
实测是 **10.001 / 10.001 Hz**（`p1_headless/hz_gz_both.txt`），那才是"一帧一路"的真实关系。

### 9.3 "GUI 必看"与"10 Hz 点云"能不能同时成立

第一枪留下的疑问：P1-lite 的 GUI 跑出 **8.350 / 8.428 Hz**（不是 10 Hz），且 `dt_max=0.217 s`。
**是 RTF 掉了，还是 publisher/sim-clock 耦合错了？** 这决定了"GUI 必看"是不是与"10 Hz 点云"冲突。

#### 9.3.1 受控实验（同世界、同传感器、只改渲染模式）

`.tmp_gzsim2/run_rate.sh`：`p1_lite.sdf`（1000×30 `gpu_ray` + IMU + DiffDrive），
**server 与 GUI 分成两个进程启动**（`ign gazebo -s` / `ign gazebo -g`）以便**精确归属 CPU**；
每段 `timeout` 硬上限；每次跑用**独立 `GZ_PARTITION`**（这台机器是共享的，别的任务可能同时在跑 sim，
同一 partition 上的同名 topic 会互相污染）；`HOME` 每跑一个独立可写目录
（只读的 `$HOME/.ignition` 会让 GUI 起不来并 SIGKILL server）。

| 模式 | `/clock` Hz | `/stats` RTF | scan **墙钟 Hz** | scan **sim 时间 Hz** | server CPU | GUI CPU | server RSS | GUI RSS |
|---|---|---|---|---|---|---|---|---|
| **headless**（`-s --headless-rendering`，无 DISPLAY） | **1000.01** | **0.999** | 22.13 | 22.9 | **≈49 %** | — | 387 MB | — |
| **GUI 1280×800** | **833.64** | **0.819** | 21.61 | 28.4 | **≈51 %** | **1112–1130 %** | 392 MB | **610 MB** |
| **GUI 640×480** | **835.43** | **0.879** | 21.28 | 28.4 | **≈51 %** | **1214–1220 %** | 393 MB | **611 MB** |

（CPU/RSS 取 `cpu_samples.txt` 里**真正的 `ign gazebo` 进程**那一行；脚本用 `setsid` 起独立进程组，
样本里还会混进 `timeout` 包装进程（≈0 %/2 MB），**把两者平均会得到 24 %/195 MB 这种假数**，别用。
三个模式的 **server CPU（49→51 %）与 RSS（387→393 MB）基本不变** ⇒ **GUI 没有加重 server 的负担**，
它抢的是整机的核：**GUI 一个进程就吃 11–12 个核**。）

#### 9.3.2 机制：**sim 时钟被整体拖慢**（不是 publisher 的错）

证据链（每一环都是独立测量）：

1. **`/clock` 从 1000.01 Hz 掉到 833.6 Hz**（×0.834）——`/clock` 是"每个 1 ms 步进一条"，
   所以这直接就是**步进频率**掉了；`/stats` 的 RTF 0.999 → 0.819 独立复现同一结论。
2. **server 的 CPU 没变**（49 % → 51 %）而 **GUI 一个进程吃 1112–1220 %**（≈11–12 核）⇒ 抢 CPU 的是 GUI 的软件光栅化
   （llvmpipe，本机无 GPU，FSAA 8 被强制降到 0：`Ogre2RenderTarget.cc:574` 的警告）。
3. **传感器是按 sim 时间定频的**：gz-sensors 的 `Sensor::Update(_now)` 拿 `_now` 和
   `nextUpdateTime` 比（`sensors6_Sensor.cc:458-483`，`nextUpdateTime += 1/updateRate`），
   `GpuLidarSensor::PublishLidarScan(_now)` 用它当 stamp（`sensors6_GpuLidarSensor.cc:283`）。
   所以 **墙钟 Hz = sim 时间 Hz × RTF**。
4. **决定性的一步在"每周期恰好 1 条"的那次运行里**：第一枪 `p1_gui` 8.350 Hz / `p1_gui2` 8.428 Hz
   对 headless 10.001 Hz ⇒ 比值 **0.835 / 0.843**；而 `/clock` 的比值是 **0.834**（本轮）
   / **0.842**（R0 的 `841.53/1000.01`）。**三个独立量在 1 % 内一致** ⇒ **就是 RTF 掉，
   不是 publisher/时钟耦合错**。
5. `dt_max=0.217 s` 也对得上：`100 ms / 0.834 ≈ 120 ms` 的正常周期 + 一次丢帧（≈2 个周期）⇒ 0.217 s。
6. **今天的墙钟速率不能用来验证这条传递链**：今天每周期发 2.2–2.6 条（§9.3.4），所以墙钟
   `22.13 → 21.61` 几乎没掉，而 sim 时间速率 `22.9 → 28.4` 反而偏高（倍数不稳）。**倍数把
   RTF 的效应掩盖了。** 但 `/clock`（1000.01 → 833.64）和 `/stats`（0.999 → 0.819）已经把
   "整场仿真变慢"这件事直接测出来了，第一枪那组 1.0× 的数据则把"传感器跟着变慢"补上了。

**⇒ 谁在拖慢？** 不是"点云算不动"（server CPU 没变），而是 **GUI 的 llvmpipe 把整机内存带宽/CPU
抢到 gz-sim 的 1 ms 步进跑不满 1000 步/秒**。这同时解释了 R0 的 GUI：世界更重（有阴影 + 30k），
GUI 吃 **1296 %**、RTF 0.83、`/clock` 841.5 Hz、scan 8.542 Hz（比值 0.849）。

**⇒ 缩窗口救不回来**：640×480 只把 RTF 从 0.819 抬到 0.879（GUI CPU 反而从 1112 涨到 1216 %）⇒
GUI 的成本是**固定开销**（场景/合成器/llvmpipe 管线），不是像素数。想省只能**关 GUI** 或换有 GPU 的机器。

#### 9.3.3 对"GUI 必看 + 10 Hz 点云"的裁决

| 问法 | 答案 |
|---|---|
| GUI 能不能留 | **能**（独立进程、独立 gz-rendering 实例，§7.6 Q2 已证） |
| 开了 GUI 还有 10 Hz 点云吗 | **在 sim 时间里是 10 Hz**（SDF 的 `<update_rate>10</update_rate>` 没变；第一枪实测 headless `10.001 Hz` → GUI `8.35–8.43 Hz`，算法侧开 `use_sim_time` 看到的仍是 10 Hz 周期） |
| 那 8.35 Hz 是什么 | **墙钟频率**：10 Hz × RTF 0.82–0.88。整场仿真一起变慢，**不是点云单独变慢** |
| 代价是什么 | **墙钟吞吐 0.82–0.88×**：一次 40 s 的跑要 47–49 s；实时性/回归时长要留 ~20 % 余量；`dt_max` 会到 0.2 s（下游 `use_sim_time` 的 TF/odom 也一起变慢，**相对关系不变**） |
| 要 10 Hz 墙钟怎么办 | **关 GUI**（headless `/clock` 1000.0 Hz、RTF 0.999），或上 GPU |

#### 9.3.4 ⚠️ 必须标注的口径异常：**今天所有模式都多发 2.2–2.6 条/周期**

今天在**所有**模式下都测到"传感器一个周期发了 2–3 条消息"：

| `update_rate` | 1 | 5 | 10 | 20 |
|---|---|---|---|---|
| 实测墙钟 Hz | **2.47** | **11.20** | **22.13** | **46.21** |
| 倍数 | 2.47× | 2.24× | 2.21× | 2.31× |

而判据显示"周期本身是对的"：`dt_max ≈ 1/update_rate`（1 Hz → 1.0034 s；10 Hz → 0.1026 s），
`dt_min ≈ 0`（成对出现），**单一 publisher**（`ign topic -i` 只有一行），**`dup_stamps=0`**
（成对的消息 sim stamp 不同），**与射线数无关**（把 30k 降到 300 条，速率仍是 23.6 Hz），
**与是否 `--headless-rendering` 无关**（`-s` 与 `-s --headless-rendering` 都是 22.2 Hz）。

⇒ **判定**：`<update_rate>` 并没有被当成绝对频率执行（每周期 2–3 次 publish）。
第一枪在同一世界测到的是 **1.0×**（10.001 Hz，`dt_max=0.1018`），所以这个倍数**依赖环境/负载，不可复现**。
**因此**：

- **绝对 Hz 数字（本节的 22 Hz、第一枪的 10 Hz）都不能当作"传感器频率"引用**；
  可信的是 **比值**（GUI/headless）、**`/clock` Hz**、**`/stats` RTF**，以及**同时订阅两路时的 1:1 关系**；
- 这也正好解释了 §9.2 表里 `points` 那几个 20–28 Hz 的怪值（顺序探测 + 不同窗口）；
- 机制未查明（见 §9.6 #3）。**在下游用 ROS 侧 `ros2 topic hz` 做验收**，别信 gz 侧的绝对数。

### 9.4 R3：能不能把任意 CSV 图案塞进 OGRE2 的 `GpuRays`

**问题**：Fortress 的 `gpu_ray` 只能配规则网格（`<ray><scan>` 的水平/垂直 samples），
所以拿不到 MID-360 的逐射线图案。能不能**只换"方向表"**、保留"光栅化 + 一次 pass"的优势？

#### 9.4.1 结论先行

**技术上成立，而且比想象中轻——因为上游代码里"方向表"本来就已经是一张纹理了。**
但它是**对 `gz-rendering` 公共 API 的扩展**，落地必然是 **fork（2 个包，带 SDF 管道则 3 个）**，
并且**精度受立方图重采样的硬上限约束**。详见下面。

#### 9.4.2 源码里"方向表"到底在哪（file:line）

上游的实现是**两遍**：

| 环节 | 位置 | 作用 |
|---|---|---|
| 第 1 遍：**立方图** | `Ogre2GpuRays.cc:136-207`（`cubeCam[6]`、`kCubeCameraCount=6`）、`Ogre2GpuRays.cc:563-622`（按 FOV 算每个面要多少采样） | 6 个 90° 相机把**整个球面**渲染成 6 张深度图 |
| 深度 → 距离 | `media/materials/scripts/gpu_rays_1st_pass_fs.glsl:83-86`（`viewSpacePos = cameraDir * d; l = length(viewSpacePos)`） | 每个像素一个"沿该像素方向的射程" |
| **第 2 遍：方向表** | **`Ogre2GpuRays.cc:661` `CreateSampleTexture()`**，填充循环 **`Ogre2GpuRays.cc:722-745`** | 建一张 **RGBA32F 纹理**，**每个输出像素**打包 `(u, v, faceIdx)`：`R=u`、`G=v`、`B=cube face`（注释在 `Ogre2GpuRays.cc:678-682`） |
| 方向 → cube UV | **`Ogre2GpuRays.cc:633-660` `SampleCubemap()`** | 世界方向向量 → 面号 + uv |
| 第 2 遍的着色器 | `media/materials/scripts/gpu_rays_2nd_pass_fs.glsl:26`（`uniform sampler2D cubeUVTex`）、**`gpu_rays_2nd_pass_fs.glsl:66-95`** | `data = texture(cubeUVTex, uv)` → 取 `faceIdx` → 从对应面取 `range/retro` |

**关键**：`Ogre2GpuRays.cc:722-745` 那个双重循环就是"规则网格 → 方向 → cube UV"的**唯一**入口：

```cpp
double v = vmin;
for (unsigned int i = 0; i < h2nd; ++i) {          // 垂直行
  double h = min;
  for (unsigned int j = 0; j < w2nd; ++j) {        // 水平列
    math::Vector3d ray(0, 0, 1);
    math::Quaterniond pitch(math::Vector3d(1,0,0), -v);
    math::Quaterniond yaw  (math::Vector3d(0,1,0), -h);
    math::Vector3d dir = yaw * pitch * ray;
    math::Vector2d uv = this->SampleCubemap(dir, faceIdx);   // ← 方向表
    pDest[index++] = uv.X(); pDest[index++] = uv.Y();
    pDest[index++] = faceIdx; pDest[index++] = 1.0;
    h += hStep;
  }
  v += vStep;
}
```

⇒ **任意 CSV 图案 = 把 `(i,j)` 换成 CSV 的第 k 行**：`az = csv_az[k]`、`el = 90°−csv_zen[k]`，
`h2nd` 从 30 改成 1、`w2nd` 从 1000 改成 30000。**不需要改第 1 遍，也不需要改着色器。**

#### 9.4.3 要改哪些地方、多少行（估算）

| # | 包 / 文件 | 改动 | 估算 |
|---|---|---|---|
| 1 | **`gz-rendering` 公共 API** `include/gz/rendering/GpuRays.hh:121-156`（当前只有 `SetRayCount/SetVerticalRayCount/SetAngleMin/Max`） | 加 `SetRayDirections(const std::vector<math::Vector3d>&)` + `RayDirectionCount()`；**空 = 现状**（后端兼容） | **30–50 行** |
| 2 | `gz-rendering` `BaseGpuRays.{hh,cc}` + `Ogre2GpuRays.{hh,cc}` | 存方向表；`CreateSampleTexture()` 里"有方向表就按表填、没有就走原双重循环"；`UpdateFOV` 里 `w2nd = N, h2nd = 1`；`cubeFaceIdx` 照旧收集（决定要渲染哪几个面） | **60–100 行**（**复用** 661–790 的纹理上传代码） |
| 3 | **`gz-sensors`** `src/GpuLidarSensor.cc:186-240`（`CreateLidar()` 里配网格）+ `src/Lidar.cc:150-163`（`set_count/set_vertical_count`）、`Lidar.cc:233-280`（`PublishLidarScan` 的双层循环） | 读 CSV（`Time,Azimuth,Zenith`），算方向，调新 API；消息改成 `count=N, vertical_count=1`；点云 `width=N, height=1`；`FillPointCloudMsg` 走一维 | **120–200 行** |
| 4 | **`gz-sim`**（可选） | SDF `<sensor type="gpu_ray">` **没有** CSV 字段（`sdformat-9.7/sdf/Lidar.hh:104-260` 只有 `horizontal/vertical` 的 samples/min/max/resolution）⇒ 要么扩 schema + `Sensors` system（**fork `gz-sim6`**），要么**自己写一个 gz system 直接建传感器**（不动 gz-sim，但仍要 fork gz-sensors 才能定制 `GpuLidarSensor`） | **40–150 行** |
| | **合计** | **2 个包（必需）/ 3 个包（SDF 顺手）** | **≈250–400 行 + 打包/版本钉子** |

**注意**：给 `GpuRays` 基类**加虚函数会破坏 ABI**（vtable 布局变），所以"装一个 patch 进现成
`libignition-rendering6`"是不可行的——必须是**整包重编译的 fork**（colcon overlay）。
这对我们是可接受的（迁移本来就要重建工作区），但**不是"改个配置"**。

#### 9.4.4 精度上限（**这条决定 R3 值不值**）

第 1 遍的立方图分辨率是硬编码 clamp：

```cpp
// Ogre2GpuRays.cc:615-622
unsigned int samples1stPass = std::clamp(v, min1stPassSamples /*128*/, max1stPassSamples /*1024*/);
this->Set1stTextureSize(samples1stPass, samples1stPass);
```

⇒ **每个立方图面最多 1024²，即 90°/1024 ≈ 0.088°/texel**。任意方向是从这张图里
**最近邻重采样**出来的，于是：

- **角度量化**：每条射线的方向被吸到 0.088° 的网格上。MID-360 的 30k 图案铺在
  360°×59°（≈21,240 deg²）上，**平均点间距 ≈0.84°**（`sqrt(21240/30000)`），
  立方图 texel 比它**细约 10×** ⇒ **量化误差小于图案间距，可接受**；
- **边缘漏检/重影**：细杆、薄板、栏杆这类目标，命中与否取决于立方图 texel 是否覆盖它
  ⇒ **同一个世界、同一条射线，R3 与 Classic/R1 可能给出不同答案**（Classic 是精确三角面求交）；
  这直接影响"感知等价性"验收（§7.4 的判据 ⑤）；
- **无法保留逐点时间偏移**：CSV 第一列是 `Time`，Classic 插件虽然当前 `offset_time=0`
  （`livox_points_plugin.cpp:381`），但真机是有帧内运动畸变的；R3 和 R0 一样是"一帧一个位姿"。

⇒ **R3 的保真度是"方向对、距离是重采样的近似"**，介于 R0（方向都不对）和 R1（逐射线精确）之间。

#### 9.4.5 要不要 fork / 能不能上游

> **2026-10-18 补充**：R1 的「外面是否已经有人做了」已单独调研，见 [`livox_sensor_prior_art.md``livox_sensor_prior_art.md` —— **存在 MIT/BSD-3 的现成实现**（`ashduwihch/mid360_gazebo_harmonic` = rmagine/Embree CPU BVH + MID-360 CSV + `CustomMsg`；`uos/rmagine_gazebo_plugins` = BSD-3 且明示 Fortress 已测），**R1 可降级为「移植 + 适配」（S–M）**；关键新事实：Fortress **没有** `components/RaycastData`（那是 `gz-sim8+`），所以"让物理引擎批量求交"在 Fortress 不存在，BVH 必须自己持有。

| 问 | 答 |
|---|---|
| 要 fork 吗 | **要**：`gz-rendering`（必需）+ `gz-sensors`（必需）+ 可选 `gz-sim`；加虚函数 ⇒ 必须整包重编译 |
| 能上游吗 | **技术上可以**（内部结构已经支持"方向表"，PR 是"把表从规则网格换成外部数组"），但要**跨 3 个 repo + SDF schema 变更 + 一个新的 sensor 配置入口**，且会引入"重采样精度"的产品决策 ⇒ 现实中是**几个月级的多 PR**，赶不上我们的迁移窗口 |
| 风险 | **高**：①fork 的 ABI/版本钉子（Fortress 一旦打补丁就要 rebase）②重采样精度影响感知等价性验收 ③llvmpipe 下 30k 方向表的纹理带宽未实测（第 1 遍的分辨率不能随便调大） |
| 与 R1 比 | R1 **不动任何上游包**（只加一个自己的 system）、**逐射线精确**、**纯 CPU**；R3 省的是 CPU（光栅化），但把成本换成"fork + 精度损失" |

#### 9.4.6 R3 判定

**⚠️ 现实但重。** 如果本项目**长期押注 gz**、并且能接受"距离是 0.088° 立方图重采样"的近似，
R3 是唯一能同时拿到"任意图案 + 光栅化 O(1) 内存"的路线；
但**对"这一轮迁移"**，它的性价比低于 R1（见 §9.5）。**不建议把它排进 P2。**

> 顺带说明：第一枪下载的 **`igngz6_Sensors.cc`（1037 行）和 `igngz6_RenderUtil.cc`（3838 行）
> 都不在这条改动路径上**——前者是 gz-rendering 的 sensor 容器（无 `GpuRays`），后者是 OGRE2 场景
> 构建（材质/可视），全文 grep 不到 `GpuRays`。**`gzsim6_Sensors.cc` 那次下载失败（0 字节）也不影响结论**：
> 真正要改的是 `Ogre2GpuRays.cc` 的 `CreateSampleTexture()` 和 `GpuRays` 的公共 API。

### 9.5 决策表与推荐

#### 9.5.0 先回答 §7.6 Q1 留下的开放问题（"自研 R1 会不会又变成 O(rays)"）

第一枪的判断是：**"R1 本质上是把 ODE 的活搬到自己手里，很可能又变成 O(rays) 的 CPU 成本"**，
并说这是 P2 最重要的技术判断。**本节把它量化了**：

- **确实是 O(rays)**：单线程 0.417–0.978 µs/ray，30k 条 = **12.5–29.4 ms/帧**；
- **但这个"O(rays)"在无 GPU 的 28 线程机器上完全付得起**：4 线程 **5.6–13.6 ms**、8 线程 **3.2–7.4 ms**，
  10 Hz 预算（100 ms）有 **7–30× 余量**；
- **内存优势保住了**：R1 只需要存方向向量（30k×3×8 B ≈ 720 KB）+ 一份静态 BVH，
  **不需要"每射线一个碰撞体"**（Classic 的 2.74 GiB 根因）；
- **而且它不碰渲染器** ⇒ 顺手解掉"LiDAR 依赖软件 GL"这条刚性耦合（§7.5 的"没有省法"结论对 R1 不成立）。

⇒ **§7.6 Q1 的答案是分层的**：**"R0 已消除射线成本"成立；"R1 会把成本搬回来"也成立，
但搬回来的量级是 0.5–13 ms/帧，不是 Classic 的 2.74 GiB + 单线程世界更新。**

#### 9.5.1 四条的对照（保真度 × 吞吐 × 无 GPU × 工作量/风险）

| 方案 | **保真度**（能否复现 MID-360 图案 / 距离是否精确） | **吞吐**（30k@10 Hz，本机实测） | **无 GPU 可用** | **工作量 / 风险** | 结论 |
|---|---|---|---|---|---|
| **Classic 11（现状）** | **最高**：逐射线 ODE 求交，图案任意 | **最差**：RTF 0.32–0.46（整栈）、RSS 2.74 GiB（每射线一个 ODE 体） | ✅（但吃 444–609 % CPU 的 gzclient） | 0 / **EOL 风险** | 基准台，别动 |
| **R0：Fortress `gpu_ray` 规则网格** | **低**：方向只能是规则网格，**拿不到非重复图案**；距离精确（光栅化） | **最好**：RTF 0.999（headless）/ 0.82（GUI），RSS 375 MB，CPU **24.7 %**（R0 世界，生命周期均值） | ✅（llvmpipe/EGL，实测） | **0（已跑通）** / 低 | **近期可用的默认档** |
| **R1：自研 CPU BVH system** | **最高**：CSV 逐射线、三角面精确求交；还能加逐点时间戳/噪声 | **够用**：1 线程 12.5–29.4 ms/帧；4 线程 5.6–13.6；8 线程 3.2–7.4（预算 100 ms） | ✅✅ **纯 CPU，连渲染器都不用** | **中高** / 中（自维护 system + 动态世界 refit） | **P2 推荐** |
| **R3：fork `gz-rendering`+`gz-sensors` 喂方向表** | **中高**：方向精确，**距离是 0.088° 立方图重采样**（薄物可能漏/重） | **最好**（同 R0，O(1) 内存） | ✅ | **高**（2–3 个 fork，250–400 行 + ABI 钉子） / **高** | **长期选项，不进 P2** |

#### 9.5.2 推荐（**含"保真 vs 速度"这个必须由用户拍板的取舍**）

**我的推荐是 R1（自研 CPU 射线 system）作为 P2 的主线，R0 作为开发/CI 的默认档，R3 只留作长期议题。**
理由按重要性排序：

1. **保真度是这个仓库的核心资产**：本项目已经有一整套"感知保真度"审计
   （`docs/tilted_lidar_fidelity.md`、`docs/session_digest.md`、`small_point_lio` 的变换 bug 审计）。
   R0/R3 都会改变点云的空间分布（R0 连方向都不对，R3 的距离是重采样的），
   **等于把已经校准好的感知基准作废**；R1 是唯一能保持"同一条射线打到同一个三角面"的路线。
2. **R1 的成本已经被量出来，而且不贵**：最坏（100 % 命中）单线程 29.4 ms/帧，4 线程 13.6 ms；
   标称位姿单线程 12.5 ms、4 线程 5.6 ms。**10 Hz 预算（100 ms）下的余量：单线程最坏 3.4×、
   标称 8.0×；4 线程 7.4–17.9×；8 线程 13.5–31×**，而且这是在一台
   **没有 GPU** 的机器上、用**纯 C++/Bullet** 得到的。
3. **R1 是纯加法**：一个 `gz-sim` system 插件（编译成 `.so`，在 world 里 `<plugin>` 加载）
   ⇒ 不动 `bringup_sim.launch.py` 默认值、不动 xacro、不动 `rm_perception/**`、不动 nav2 params，
   与 §6.11"保持不动清单"完全兼容。**不 fork 任何上游包**。
4. **R3 的收益（省 CPU）在本机并不稀缺**：server 侧的 30k `gpu_ray` 只吃 **24.7 %**（R0 世界，
   生命周期均值；P1-lite 的渲染期样本 ≈49 %），
   而 GUI 一开就吃掉 1112–1296 %。**真正的瓶颈是 GUI 的软件光栅化（11–13 核），不是射线**。R3 省的那点 CPU
   解决不了"无 GPU 机器上 GUI 贵 12 倍"这个主要矛盾。
5. **R1 顺带解掉一个架构耦合**：Classic 和 R0 都是"LiDAR ⇒ 渲染器 ⇒ 软件 GL"，渲染器起不来就没点云；
   R1 把这条依赖**彻底切断**（CI/无头回归再也不会因为 EGL/llvmpipe 挂掉而丢数据）。

**必须由用户拍板的取舍（我不替你决定）**：

| 取舍 | 选 A | 选 B |
|---|---|---|
| **保真 vs 速度** | **A：要 MID-360 的逐射线图案** ⇒ 走 **R1**（多写一个 system，约 1–2k 行；每帧多 0.5–13 ms CPU；换来感知基准可迁移） | **B：只要"有个 30k 点的 3D LiDAR"** ⇒ 走 **R0**（零成本，今天就能跑；但点云分布与真机不同，所有感知阈值/标定要重做） |
| **GUI vs 10 Hz 墙钟** | **A：GUI 必看** ⇒ 接受 RTF 0.82–0.88（40 s 的跑变 48 s），算法侧 `use_sim_time` 看到的仍是 10 Hz | **B：要 10 Hz 墙钟** ⇒ 关 GUI（headless），调试另开一台/另起一个 GUI 连同一 server |
| **长期 vs 近期** | **A：长期押 gz** ⇒ 值得为 R3 排一个 fork 的预算（2–3 包） | **B：先把迁移做完** ⇒ R3 只写进 roadmap，P2 用 R1 |

**一句话**：**如果"点云必须和现在一模一样"——答案是 R1，而且它划算；
如果"点云长得不一样也行"——R0 已经躺在 `.tmp_gzsim2/` 里了，今天就能用。
R3 是"两个都要"的选项，但它把成本换成了 fork + 重采样精度，不该压在 P2 上。**

### 9.6 诚实清单（本节**新增**的未验证项，接 §8）

| # | 未验证 | 原因 / 现状 |
|---|---|---|
| 1 | **R1 接进完整栈后的 RTF** | 本节只测了**射线求交**（12.5–29.4 ms/帧 @1 线程）。不含：从 gz-sim 抽世界几何、消息构造（480 KB + 960 KB/帧）、传输、以及 SLAM/LIO/nav2 的负载。**必须等 P4 的 A/B**。 |
| 2 | **动态障碍下的 BVH 成本** | 探针把世界 BVH **建一次**（22–24 ms）。移动障碍需要 refit/rebuild，**未测**（Fortress 的 `btBvhTriangleMeshShape` 支持 refit，但代价未知）。 |
| 3 | **"每周期多发 2.2–2.6 条消息"的机制** | 已排除：多 publisher、重复 stamp、射线数、`--headless-rendering`、窗口大小。**未定位**到具体代码路径（缺 `gz-sim6` 的 `Sensors.cc`，且本沙箱**无外网**：`raw.githubusercontent.com` 解析到非公网 IP，无法下载）。**只有比值可用**。 |
| 4 | **R3 的原型与重采样精度** | **没有写任何代码**。0.088°/texel 的量化、薄物漏检率、llvmpipe 下方向表的带宽，全部是**推算**，未实测。 |
| 5 | **`gz::rendering::RayQuery`（第一枪的另一个尝试）** | 它的 `us_per_ray=0.348 / engine_ms=117.3` 与 CPU BVH 的 0-hit 数字**同源**（mm/m 单位错），**同样无效**；本轮**没有重测**（需要 GL 上下文 + 用米制 mesh 重跑）。**不要引用那两个数字。** |
| 6 | **有 GPU 机器上的对照** | 本机无 `/dev/dri`、NVIDIA 驱动未加载 ⇒ 所有数字都是"无 GPU"的绝对值。有 GPU 时 R0/R3 的成本会大幅下降，R1（纯 CPU）**不变**——这反而让 R1 的相对优势在有 GPU 的机器上变小。 |
| 7 | **`gz-sim6` 的 `Sensors.cc` 源码** | 上一枪下载失败（0 字节，`.tmp_gzsim2/gzsim6_Sensors.cc`）；本节对 SDF 管道的分析只到 **schema 层**（`sdformat-9.7/sdf/Lidar.hh`）。 |
| 8 | **16 线程以上的伸缩** | 只测到 16 线程（28 个逻辑核）。E-core/P-core 混合拓扑下的最优线程数未系统扫。 |
| 9 | **`guismall` 的"窗口真的变小了吗"** | Xvfb 屏幕设成 640×480，但 `ign gazebo -g` 的窗口未必跟着缩（Qt 会 clamp/裁切）；**GUI CPU 没变（1112→1216 %）说明"按像素省钱"这条路本来就不通**，但"窗口实际多大"未取证。 |

---

## 附录 C 第二枪复现命令（全部在 `.tmp_gzsim2/`，**不动任何功能代码**）

```bash
cd <repo>
D=.tmp_gzsim2
MESH=src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUC2026_world/meshes/RMUC2026.stl
CSV=src/simulation/rm_simulation/livox_laser_simulation_RO2/scan_mode/mid360.csv

# ---- 0) 编译（R1 探针需要 ignition-common4-graphics 与 ignition-utils1） ----
g++ -O2 -std=c++17 -Wall $D/cpu_ray_bvh.cc -o $D/cpu_ray_bvh \
    $(pkg-config --cflags --libs ignition-common4 ignition-common4-graphics ignition-msgs8 ignition-utils1) \
    $(pkg-config --cflags --libs bullet) -lpthread
g++ -O2 -std=c++17 -Wall $D/hz_probe2.cc -o $D/hz_probe \
    $(pkg-config --cflags --libs ignition-msgs8 ignition-transport11 ignition-utils1) -lpthread

# ---- 1) R1：先看 sanity（不 PASS 就不会给计时） ----
RAY_REPS=3 $D/cpu_ray_bvh "$MESH" "$CSV" 30000 200.0 1 2 4 8 16
#   RAY_SCALE / RAY_ZOFF / RAY_POSE="x,y,z" / RAY_START（CSV 滚动窗口）/ RAY_ENCLOSED=1
#   RAY_DUMP=path 可导出逐射线命中距离，用于和 Python 暴力求交交叉验证

# ---- 2) R1：全场景 × 全线程（结果写 $D/r1_final.log） ----
$D/run_r1.sh

# ---- 3) R0：30k / 2880 × headless / GUI（会写 run_r0_*/ 并做 `ign stats`） ----
$D/run_r0.sh                      # 第一枪的脚本，产物在 $D/run_r0_*
#   观察点：stats.txt 的 real_time_factor、cpu.txt、hz_scan.txt / hz_points.txt / hz_clock.txt

# ---- 4) GUI vs 10 Hz：同世界同传感器，headless / GUI 1280x800 / GUI 640x480 ----
$D/run_rate.sh headless headless
$D/run_rate.sh gui      gui 1280x800
$D/run_rate.sh guismall gui 640x480
#   产物：rate_<label>/{hz_both,hz_clock,hz_stats,pub,cpu_samples}.txt
#   要点：server 与 GUI 分进程启动（set sid 进程组）以便精确归属 CPU；
#         每次跑独立 GZ_PARTITION；每段硬 timeout；只 kill 自己的进程组

# ---- 5) 位姿扫描（找"命中重/未命中重"场景；用 30k 行，别用 4k，CSV 是时间排序的） ----
$D/sweep_poses.sh                 # 2 m 网格，写 $D/r1_pose_sweep.csv

# ---- 6) 收尾检查（只查自己的进程；本沙箱是独立 PID namespace，看不到也不影响别人的进程） ----
pgrep -a -f "ign gazebo" ; pgrep -a -f Xvfb ; echo "leftovers: (none) 期望为空"
#   每个脚本都 set sid 起自己的进程组，exit trap 里 `kill -9 -<pgid>` 只杀自己那一组
```

---

## 11. **最终状态**（2026-10-18）：P0–P2 落地为 opt-in 的 `sim_backend:=gz`

> 本节是上面 P0–P4 计划的**收口指针**，不重复内容。

**一句话**：`sim_backend:=gz` 已经是一个**可用的 opt-in 后端**（默认仍是 `classic`，
不传该参数时节点集合/命令行/时序逐字节不变）。详细交付物、契约对照表、实测数字、
"会坏的地方"与回退方法全部在 **`docs/fortress_sim_backend.md`（原文已删除，见 git 历史）**。

| 计划项 | 状态 | 落在哪 |
|---|---|---|
| **P0 可行性** | ✅ 完成（§1–§5 的 spike + §9/§10 的实测） | `livox_sensor_prior_art.md` §10 |
| **P1 机器人** | ⚠️ **部分**：模型能进 Fortress（从本仓 URDF 运行期派生 SDF，保留 **113 个真碰撞体**、`livox_frame` 的 roll −0.5236、IMU 传感器），但**底盘不动**（Fortress 无 `planar_move` 对应物 ⇒ 世界里的机器人是 `<static>`） | `src/simulation/rm_simulation/rm_livox_gz_sim/scripts/make_robot_gz_sdf.py` |
| **P2 传感器系统** | ✅ 完成：R1（自有 CSV 图案 + 30000 束 CPU 射线）在 Fortress 上跑通，**双发布 = 两个插件实例**（`/livox/lidar` CustomMsg + `/livox/lidar/pointcloud` PC2，逐点相同） | `tools/scripts/gzsim/`（取上游 + 我们的 3 个补丁）+ `rm_livox_gz_sim/` |
| **P3 桥接** | ✅ 完成，但**比计划小得多**：只有 `/clock` 需要 `ros_gz_bridge`；点云/IMU 是插件**进程内** rclcpp 直发，所以没有 `PointCloudPacked`、也没有 "frame_id 变成 gz scoped name" 那个高风险项 | `rm_livox_gz_sim/config/ros_gz_bridge.yaml` |
| **P4 A/B 验收** | ⚠️ **部分**：gz 侧全部实测（10.0000 Hz 仿真 / 0.97 RTF / 328–652 MB / 逐点一致 / 自机排除 A/B）；**Classic 侧没有同轮对照**；**LIO 端到端未通过**（`small_point_lio` 在 gz 档持续 `imu loop back`，不发 `odom→base_link`）；`/scan` 链路已打通（418 条有限波束、`frame_id=base_link`） | `fortress_sim_backend.md` §2.8 / §8 / §10 |

**与计划的一处口径修正**：计划里 P2 建议"新增独立包 `rm_livox_gz`、自写 `Mid360System`"
（B 方案，200–300 行）。实际走的是 A 方案（用上游 `ashduwihch/mid360_gazebo_harmonic`
+ 我们的 3 个补丁），因为 §10 已经证明它在 Fortress 上**零 API 障碍**（只差 2 个宏），
自写一遍的收益不足以抵消"再维护一份 300 行 C++"的成本。代价是**多一条构建期依赖链**
（三个上游 + Embree 构建期下载），因此把它做成了**一键脚本 + 补丁 + 独立前缀**，
不 vendor、不污染默认路径。


---


## 来源 2/7：`sim/worlds.md`


> 入口：`ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 ...`
> **2027 的两个新场地（RMUL2027 3V3 / RMUC2027，由 STEP 生成）见 `docs/worlds_2027.md`**；
> 本文（以及本文描述的 RMUC2026 资产）本轮**未改动**。
> 本文回答：这个场地是什么、怎么从 STL 变过来的、出生点怎么选的、地图/PCD 怎么来的、
> 实测跑通了什么、以及**哪些坑是真的**（台阶/通道 vs 我们 0.22 m 半径的车）。
> ⚠️ 上面这条"0.22 m 半径 / 雷达离地 0.226 m"只对**默认机器人模型**成立。换成别的车
> （`robot:=hzmirm`：0.6×0.6 m 车体、雷达离地 0.80 m）时：本文 §3 出生点的 0.30 m 净空判据、
> §5 的无头实测结论、§6「坑」里"台阶在 `/scan` 里看得见"这条**都要重估**
> —— 实测该模型在 RMUC2026 里 6.3 m 以内的地面根本不在视野里（`/segmentation` 地面 = 0 点），
> 见 `docs/robot_models.md` §3.2/§4/§5。

---

## 0. 一句话

把附件 `easystl.stl`（毫米单位、Z 朝上、29.15 x 16.05 m 的 RMUC 级别全场）按
**scale 0.001 + z 抬 1.6413436 m** 变成 Gazebo 世界 `RMUC2026`，并配套生成
2D 栅格图与先验 PCD，四种形态实测：**建图 ✅ / 2D+AMCL 导航 ✅ PASS / 3D+GICP 导航 ✅ PASS
（含一条"定位仍在缓慢收敛"的打折项）**。
**场地本身可跑**，但场地几何对我们这台车不友好：0.20/0.30 m 台阶上不去、两个半场之间
只有 0.45~0.50 m 宽的通道（车直径 0.44 m）⇒ 实际可通行区 ≈ **176 m²（占图上游离区 237.8 m² 的 74%）**，
且基本被限制在出生点所在的那半个场（见 §6）。

---

## 1. 资产清单与来源

| 文件 | 说明 |
|---|---|
| `src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world/meshes/RMUC2026.stl` | **实际被 Gazebo 加载的那份**（`GAZEBO_MODEL_PATH` 指向 `<share>/hzmi_rm_simulation/meshes`，见 §2.3）。2,295,084 B |
| `src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world/model.{config,sdf}` | `model://RMUC2026_world` 的独立入口（与 world 文件里内联的那份等价） |
| `src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUC2026_world/RMUC2026_world.world` | 世界文件（结构镜像 `RMUL2026_world.world`） |
| `src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUC2026_world/model.{config,sdf}` + `meshes/RMUC2026.stl` | 与上面对称的第二份目录树（镜像 RMUL2026 的既有结构；这份 `meshes/` 目前**不被任何路径引用**，只是照抄结构，占 2.2 MB） |
| `src/bringup/rm_nav_bringup/map/RMUC2026.{pgm,yaml}` | 2D 栅格图。⚠ **当前这一份不是本文 §4.1 的合成产物** —— 2026-10-06 起换成实跑建图的 `/map`，2026-10-07 又提升到 `RMUC2026_v3` 那次会话的图（`577x326`、`origin [-25.2,-9.09]`）。**当前默认对 / 换图 / 回滚见 `docs/map_assets.md`**；§4.1 描述的那份合成图现在在 `map/attic/RMUC2026.pgm.synth.bak` |
| `src/bringup/rm_nav_bringup/PCD/RMUC2026.pcd` | 先验点云。⚠ 同上：当前是实跑建图产物（2026-10-07 提升，**203,352 点**），§4.2 描述的那份合成点云现在在 `PCD/attic/RMUC2026.pcd.synth.bak` |
| `src/simulation/rm_simulation/hzmi_rm_simulation/launch/rm_simulation.launch.py` | +`WorldType.RMUC2026` 一项（出生点）与对应 GroupAction |
| `src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py` | `amcl_init_x/y` 字典 +`'RMUC2026': 0.0` |

**STL 出处（只读输入，未修改）**：
sha256 `cc723f3156999598127dfd34d1467767773708ecd46e78f6ad4c40de597d0846`
（原附件 `/home/weicheng/.dsh/attachments/v1/files/cc/cc723f31.../easystl.stl`，
工作副本 `.tmp_cache/stl_view/easystl.stl`）。仓库里两份拷贝的 sha256 与原件**逐个相同**（`sha256sum` 复核）。

---

## 2. 网格转换（mm → m、z 抬升、碰撞）

### 2.1 三处改动（相对 `RMUL2026_world.world`）

| 项 | RMUL2026 | RMUC2026 | 为什么 |
|---|---|---|---|
| `<scale>` | `0.001`（world 内联）/ `0.01`（model.sdf，未生效） | **`0.001 0.001 0.001`** | 本 STL 原生单位 = **毫米**（bbox 29150 x 16050 x 1730），不缩放就是 1000 倍大 |
| 姿态旋转 | `0 0 -2.6 -1.5705 0 0`（绕 X 转 -90°） | **`0 0 1.6413436 0 0 0`（无旋转）** | `RMUL2026.stl` 是 **Y 朝上**（bbox y 只有 520 mm）所以那份要转；本 STL 的 z 只有 1730 mm ⇒ **本来就是 Z 朝上**，转了反而躺倒 |
| 模型 pose | `8.5 4.5 2.65`（把场地挪到世界一角） | **`0 0 0 0 0 0`** | 让 **mesh 米制坐标 == world 坐标**，地图/PCD/出生点全部同一个系，少一层换算 |

z 抬升量的来历：底板（可行驶地面）**顶面**在 mesh 局部坐标 `z = -1.64134362793 m`
（不是栅格拟合值 —— STL 顶点里有 4412 个点精确落在 `-1641.34362793 mm` 这个平面上），
所以 `<visual>/<collision>` 的 pose 抬 `+1.6413436 m`，场地地面正好落在 `world z = 0`。
**不抬的话机器人会在 z=0 出生、地面在 -1.64 ⇒ 直接悬空 1.6 m 掉下去**（RMUL 那份 world 的
`z` 参数注释里记过同类事故：坠落中做重力初始化会让 LIO 地图倾斜）。

### 2.2 世界级设置：**一个都没改**

`gravity / magnetic_field / atmosphere / physics(ode, 0.001 s, RTF 1, 1000 Hz) / scene / audio / wind /
spherical_coordinates` 与 `RMUL2026_world.world` **逐字节同值**；那份 world 里没有 `<plugin>`，
本份也没有。唯一动过的是 5 盏灯的 pose：类型/颜色/衰减/方向全同，位置整体平移
`(-8.5,-2.88)`（RMUL2026 场地中心 `(8.5,4.5)` → RMUC2026 场地中心 `(0,1.62)`），
高度取 RMUL2026 world 的 `<state>` 段实际生效值 `z=7~14`（那份定义处写 `z=1`、运行时被 state 覆盖）。

### 2.3 `model://` 是怎么解析到的（踩坑记录）

`GAZEBO_MODEL_PATH` 由 `gzserver.launch.py → GazeboRosPaths.get_paths()` 在**启动时**扫描
所有包的 `package.xml` 里 `<export><gazebo_ros gazebo_model_path=...>` 拼出来（本仓库 =
`<share>/hzmi_rm_simulation/meshes`）。所以：
`model://RMUC2026_world/meshes/RMUC2026.stl` → `<share>/hzmi_rm_simulation/meshes/RMUC2026_world/meshes/RMUC2026.stl`。
⇒ 新资产加完后**必须 `colcon build --symlink-install --packages-select hzmi_rm_simulation rm_nav_bringup`**
（`install/` 是逐文件符号链接，不 build 新文件不会出现），然后 `source install/setup.bash`。

### 2.4 网格质量与碰撞：实测能用

已知（`.tmp_cache/stl_view/README.md`）：45,900 三角形 / 137,700 顶点、**非水密**、352 个独立 body、
绕向不一致、有大量重合/近重合薄片。
**做法**：`<collision>` 直接用同一份 raw mesh（和 RMUL2026 同款写法），不额外做简化凸包。
**实测量到的证据**：
* `spawn_entity` 报 `Successfully spawned entity [robot]`；
* 机器人静止时 `/odom_ground_truth`（Gazebo 平面底盘插件的真值）**z = 0.0600 m**，
  正好等于轮半径 0.06 ⇒ **站在地面上，既没沉下去也没悬空**；整场跑动中（2.6 m 直行）z 恒为 0.0600；
* 45,900 面静态 trimesh 没有产生任何 ODE/碰撞告警（`grep -E "\bODE\b|trimesh|collision"` 空）；
* LiDAR 打到台阶立面的点云与 STL 几何重合度 87.5%（§5.1）。
⇒ **不需要简化碰撞体**。若以后要省 CPU，可只把"可行驶地板 + 台阶立面"做成简化碰撞，但**没有证据表明需要**。

---

## 3. 出生点：怎么选的

推导（脚本 `.tmp_cache/rmuc2026/build_assets.py`，全部可复跑）：

1. 用 `.tmp_cache/stl_view/heightmap.npz`（0.02 m 顶视高度栅格）聚合到 **0.05 m** 栅格，
   取每格**最高**面高度（保守，保住薄墙）；
2. 相对底板 `z=-1.6413436` 抬升 > **0.15 m** 的格 = **障碍**（0.20/0.30 m 台阶、塔、柱、墙全落入），
   ≤ 0.15 m 的格 = **可行驶**；场地外沿（栅格无数据）也标成障碍 ⇒ 规划器不会往空处走；
3. 在**严格平台面**（抬升 ≤ 0.05 m）上做距离变换，要求 `clearance ≥ robot_radius(0.22) + margin(0.08) = 0.30 m`；
4. 取出生点所在连通域内**离障碍最远**的点：

   | | world (mesh 米制) | clearance | 该点 ±0.55 m 邻域高度起伏 |
   |---|---|---|---|
   | **选定** | **(+10.925, +2.525)** | **2.704 m** | ≤ 0.004 m（真平） |
   | 左半场对称点 | (-10.925, +0.775) | 2.706 m | ≤ 0.004 m |

   两半场镜像、数值等价（差 2 mm）；**取 +x 半场**只为与 RMUL2026 的出生点象限一致、便于叙述。
   实测复核：`/odom_ground_truth` 出生位姿 = **(10.9249, 2.5215, 0.06)**，与设定值差 5 mm 量级。

5. 出生 **z = 0.2**（与 RMUL2026 同口径）：轮半径 0.06 ⇒ 落地后 base_link 停在 0.06，留 0.14 m 余量，
   既不长时间悬空（避免 LIO 在坠落中做重力初始化），也不会插进地面。

launch 里的写法（`rm_simulation.launch.py::get_world_config`）：

```python
WorldType.RMUC2026: {
    'x': '10.925', 'y': '2.525', 'z': '0.2', 'yaw': '0.0',
    'world_path': 'RMUC2026_world/RMUC2026_world.world'
}
```

---

## 4. 地图与 PCD 是怎么产出的

> **⚠ 2026-10-07 现状（先读这一行）**：本章 §4.1/§4.2 记录的是 **2026-10-05 的"由 STL 合成"路线**，
> 它**已经不是当前默认资产** —— 那两份现在分别在 `map/attic/RMUC2026.pgm.synth.bak`（+`.yaml.synth.bak`）
> 与 `PCD/attic/RMUC2026.pcd.synth.bak`（`git show 280888e:…` 也有原始版本），可随时用 §4.4 的工具复跑。
> **当前默认对 = 实跑建图产物**（2026-10-07 提升到 `RMUC2026_v3` 那次会话的 2D + 3D）：
> 清单、实测对照、换图与回滚全在 **`docs/map_assets.md`**。本章保留原样，作为"合成路线怎么做的"的存档。

两条路线都试了：**当初资产用 (a) 从 STL 合成**，(b) 真实建图只作**交叉验证证据**（§5.1）；
**现在反过来** —— (b) 实跑建图是当前默认资产，(a) 退到 `attic/`（见上面的现状说明）。

### 4.1 `map/RMUC2026.pgm` + `.yaml`（路线 a；**现已在 `map/attic/`，历史存档**）

* 由 §3 的 0.05 m 栅格直接出图：`free → 254`、`occupied/外沿 → 0`，
  **阈值 0.15 m**（相对底板抬升）。取 0.05 m 而不用 0.03：0.03 m 时 free 面积 217.1 m²、
  0.15 m 时 237.8 m²，差的 20 m² 全是台阶根部的栅格过渡带；两种阈值下"能被 0.22 m 车走的部分"
  几乎一样（都被 inflation 吃掉），取 0.15 与"台阶算墙"的语义一致。
* 尺寸/原点：`resolution 0.05`、`600 x 341`、**`origin = [-25.925, -9.425, 0]`**
  = 网格左下角 `(-15.0, -6.9)` 减去出生点 `(10.925, 2.525)`。
* **map 系 = 出生点相对系**（与 RMUL2026 同口径）：出生点在 map 里就是 `(0,0)`，
  所以 `bringup_sim.launch.py` 的 `amcl_init_x/y` 给 `0.0`；`gicp`/`icp` 的
  `initial_pose: [0,0,0]` 也**不用改**（这就是选"出生点系"而不是"world 系"的原因：
  世界系的话 `initial_pose` 得按 world 注入，而 launch 并不注入它）。
* 面积：free **237.8 m²**、occupied 229.4 m²、场地外沿 44.4 m²（也标占用 = 围墙）。

### 4.2 `PCD/RMUC2026.pcd`（路线 a；**现已在 `PCD/attic/`，历史存档**）

* 几何：把 STL 采样成"**地面以上薄壳**"（镜像 `PCD/RMUL2026.pcd` 的性格 —— 那份 z 跨度 0.665 m、
  含地面切片 + 低矮障碍），高度带 **`[地面-0.08, 地面+0.60] m`**；
* 采样：按面积均匀采 3,000,000 点 → 留带内 2,036,997 点 → **按法向分两档做体素下采样**：
  * 近水平面（`|nz|>0.7`，地面/台阶顶面）：**0.25 m** ⇒ 11,124 点（只提供 z 约束，稀疏即可，对应 RMUL 的"地面切片"）
  * 近竖直面（台阶立面/塔/柱/墙）：**0.10 m** ⇒ 15,494 点（提供 x/y 约束，要密）
  * 合并 **26,618 点 / 0.85 MB**，字段与 RMUL2026.pcd 同构（8 x float32：x y z intensity + 4 个零法向/曲率）
* **坐标 = map 系**，其中**地面放在 `z = -0.06`**：这是"`initial_pose: [0,0,0]` 语义下地面该在的位置"
  —— 初值把 base_link 放在 map 原点，而车静止时 base_link 离地 0.06 m，所以地面 = `-0.06`，
  初值**精确正确**（对照：`RMUL2026.pcd` 的地面在 `-0.32`，因为那份存的是 LIO `camera_init` 系，
  与初值语义差 0.26 m，靠 GICP 的 `max_correspondence_distance=1.5 m` 吸收）。
* 体量对照：节点日志 `原始 26618 → 体素 0.100 m 后 21161`（RMUL2026 是同 leaf 下 12450）——
  场地面积是 RMUL2026 的 2.4 倍，按面积折算密度相当。**若 align 太慢**，回退阶梯：
  `voxel_leaf_size 0.10 → 0.15/0.20`（或把本 PCD 的水平面 0.25 → 0.35）。

### 4.3 现有 `map/RMUC.pgm` 能不能复用？**不能**（有证据）

`RMUC.pgm` 尺寸确实和本 STL 接近（28.85 x 15.05 m vs 29.15 x 16.05 m），但几何对不上：

* 用与 §5.1 同一把尺子（chamfer：`RMUC.pgm` 的占用格 → STL"可行驶地板轮廓"）在 **±4 m 平移**里搜：
  最好 **mean 0.525 m / 命中率 10.7%**，而全窗口命中率**中位数 7.8%**（≈随机基线），
  峰值显著性为负（最好点落在搜索窗边缘，不是峰）；
* 两个先验几何假设下更直接：`STL+(14.575, 6.401)`（左下角对齐）mean **0.732 m / 12.3%**、
  `STL+(14.425, 7.424)`（中心对齐）mean **0.718 m / 11.3%** —— **都在随机水平**；
* 语义上也对不上：`RMUC.pgm` 的 free 有 **366.8 m²**，本 STL 的可行驶地板只有 **237.8 m²**
  ⇒ RMUC2024 那张图是"几乎全平"的 2024 场地，与这张带 0.20/0.30 m 大台阶的 2026 场地
  **不是同一套布局**（与早先 FFT 配准"回到随机水平"的结论一致）。
⇒ 结论：**不复用**，2D 图与 PCD 都从本 STL 现生成（§4.1/§4.2）。

### 4.4 一条命令复跑：`tools/scripts/world/stl_to_world.py`

§3 / §4.1 / §4.2 的做法已整理成一个脚本（约定全部参数化，**换 STL 不用改代码**）：

```bash
# 复现已提交的 RMUC2026 资产（--map-grid 是当初手工取的整数边界，见下面的差异 ②）
python3 tools/scripts/world/stl_to_world.py \
    --stl .tmp_cache/stl_view/easystl.stl --world-name RMUC2026 \
    --spawn 10.925 2.525 --map-grid -15.0 -6.9 600 341

# 换一份 STL：出生点自动选（严格平台面上离障碍最远），其余全默认
python3 tools/scripts/world/stl_to_world.py --stl /path/new_field.stl --world-name RMUL2027

# 试跑/回归：写到临时树，完全不碰仓库里的资产
python3 tools/scripts/world/stl_to_world.py --stl ... --world-name RMUC2026_TEST --out-root /tmp/world_test
```

产出四样 + 两样附带：`meshes/<NAME>_world/`（`meshes/<NAME>.stl` 逐字节拷贝 + `model.sdf` +
`model.config`，这是 Gazebo 的 `model://` 解析根）、`world/<NAME>_world/`（`<NAME>_world.world`
+ 上面两份 + `meshes/`，镜像既有目录结构）、`map/<NAME>.pgm|yaml`、`PCD/<NAME>.pcd`，
外加**打印摘要**（bbox / 地面高度 / 网格 / free-occupied 面积 / 出生点 + clearance / 点数）与
**JSON manifest**（默认 `<out-root>/.tmp_cache/world/<NAME>.manifest.json`，含 STL sha256 与全部参数）。
**launch 文件不自动改**：摘要里会打印该加的那两行（格式同 §3），照抄即可。
`--help` 有全部开关；`--robot-radius / --clearance-margin / --band / --voxel-* / --map-pad /
--seed` 等都有与 RMUC2026 一致的默认值。

**回归证据**（`--world-name RMUC2026_TEST` 写进临时目录，比对完即删、**未提交**）：

| 检查项 | 工具输出 | 已提交资产 | 结论 |
|---|---|---|---|
| STL 拷贝 sha256 | `cc723f31…` | `cc723f31…` | 逐字节相同 |
| 场地 bbox | x[-14.575,14.575] y[-6.401,9.649] z[-1.841,-0.111] | 同 | 相同 |
| 自动求出的地面 | mesh z = **-1.6413436**（202.9 m² 的最大水平面） | 抬升 1.6413436 | 相同 |
| `map/<NAME>.pgm` | 600x341 @0.05，free **237.8** / occupied **229.4** / 外沿 **44.4** m² | 同 | **逐字节相同** |
| `.yaml` origin | `[-25.925, -9.425, 0]` | 同 | 除 `image:` 名外相同 |
| 出生点 | (10.925, 2.525) clearance **2.704 m** | 同（§3 表） | 相同 |
| PCD 点数 | **26,582**（0.25 m→11,146 ＋ 0.10 m→15,436） | **26,618**（11,124 ＋ 15,494） | −36 点（−0.14%），见差异 ① |
| PCD bbox（map 系） | x[-25.500,3.650] y[-8.926,7.124] z[-0.140,0.540] | 同 | 相同 |
| world / model.sdf | 去注释后 251 / 67 行 | 同 | 只有 `<model name>` 与 mesh URI（含世界名）不同 |

两处**已知的、刻意的**差异：

① **PCD 点数 ±0.1%**：已提交那份是 `trimesh` 默认（无种子）随机采样出来的；工具默认固定
`--seed 0` 以便逐次可复跑，于是"带内点数"（2,037,263 vs 2,036,997）与体素点数差几十个
（体素下采样本身确定，变的是哪些体素被采到）。bbox、两档 leaf、密度口径都一致。
② **地图网格默认值**：工具默认取"场地 bbox 外扩 `--map-pad 0.5 m` 再向外对齐整格" ⇒ **604x342**；
已提交那张是当初**手工取的整数边界 600x341**（30.00 x 17.05 m，x 比 bbox+0.5 少 0.075 m）。
两者的 free/occupied 面积完全一样（多出来的只是外沿一圈"占用"）；要逐字节复现旧图就显式给
`--map-grid -15.0 -6.9 600 341`（上面第一条命令已经这么写了）。

**另外：`.tmp_cache/rmuc2026/` 里现存的脚本与本次工具不一致，以工具为准**（资产两处都对得上）：

* `gen_assets.py` 把地图网格、出生点、地面高度（`-1.641343627929688`）全写成常量；工具全部从
  STL 自己算（地面 = 面积最大的水平面，本 STL 逐位等于那个常量）。
* `build_assets.py` 的**末尾版本**把出生点距离变换做在 `free`（阈值 0.15 m）上，会给出
  **(7.975, 2.025) / 3.200 m** —— 那**不是**已提交的出生点；能复现 §3 表与 launch 注释里那组数
  （对称两区各 ~71.4 m²、+x 点 2.704 m / −x 点 2.706 m）的是**严格平台面（抬升 ≤ 0.05 m）**版本，
  工具默认用的就是后者（`--plateau-threshold 0.05`）。即：**脚本漂移过，资产没漂**。

---

## 5. 无头实测（headless）

环境（与 `docs/smoke_test_runbook.md` / `.tmp_cache/five_way/run_one.sh` 同一套隔离）：
`HOME=/tmp/gzhome-*`（沙箱里 $HOME 只读，gzserver 建 `~/.gazebo` 会 SIGABRT）、
非默认 `ROS_DOMAIN_ID`、`GAZEBO_MASTER_URI=http://127.0.0.1:1140x`、`unset DISPLAY`、`nav_rviz:=False`、
**所有步骤在同一个 bash 调用内**（PID namespace 一结束进程就被清掉）。日志在 `.tmp_cache/rmuc2026/`。

### 5.1 `mode:=mapping lio:=fastlio mapper:=slam_toolbox`（建图）

| 检查项 | 结果 |
|---|---|
| Gazebo 加载世界 | ✅ 无 mesh/URI 报错；`gzserver` 正常起 |
| 机器人出生在地面上 | ✅ `/odom_ground_truth` = `(10.9249, 2.5215, 0.0600)`；整程 3 组 `tf2_echo odom base_link` z 稳定在 **-0.243**（不沉不飘） |
| 话题 | ✅ `/livox/lidar`、`/livox/lidar/pointcloud`、`/livox/imu`、`/scan`、`/odom`、`/odom_ground_truth`、`/map` 全部出现 |
| 频率（RTF≈0.5 时） | pointcloud 7.8~8.0 Hz、imu 77~79 Hz、scan 7.6~8.0 Hz、odom 7.7~7.9 Hz（标称 10/100，按实时率折算正常） |
| `/map` 会随运动增长 | ✅ 驱动前 `known 20.8 m²` → 驱动后 `58.5 m²`（另一次跑到 74.2 m²） |
| 驱动方式 | ⚠️ 见下 |
| LIO 是否发散 | ✅ **无发散**：LIO 估计位移 / Gazebo 真值位移 = **2.623 / 2.619 m = 1.002**（`/odom` 话题口径 2.510/2.619 = 0.958），逐帧曲线全程贴合（详见下面的"测量陷阱"） |
| 真实建图产物 vs STL 几何 | ✅ 把 `/map_save` 存下的 22,947 点真实点云平移到 world 系后：bbox `x[-14.64,14.45] y[-6.19,9.56]`（STL 是 `x[-14.575,14.575] y[-6.401,9.649]`）；0.05 m 栅格上 **87.5% 的"竖直结构"格落在 STL 障碍格 5 cm 之内**（mean chamfer 0.019 m）⇒ **毫米缩放、z 抬升、模型 pose 三件事都被实测证明是对的** |

**驱动方式（说清楚）**：`/cmd_vel_chassis`（Gazebo 平面底盘插件的执行话题）直接发 `Twist`，
`linear.x=0.4, angular.z=0`，持续 6 s ⇒ 真值前进 2.42~2.64 m（与 0.4 m/s 相符）。
**为什么不走 nav 侧**：`mode:=mapping` 不起 nav2，`/cmd_vel_nav` 没有发布者；
`/cmd_vel → /cmd_vel_chassis` 那条 fake_vel_transform 通路在 `spin_speed:=0.0` 下也可用，
但直接发执行话题最省事、且能排除中间环节。**注意插件没有命令超时**：
发布停止后车会**保持最后速度继续走**（实测停发后仍在缓慢前进），测完必须显式发零速或直接收栈。

**⚠️ 一个测量陷阱（写下来省得别人再踩）**：用 `rclpy.spin_once()` 每 0.2 s 采一次
`lookup_transform('odom','base_link')` 时，`/tf` 有 ~80 msg/s、采样循环只 ~5 Hz ⇒
tf2 Buffer 永远在处理积压 ⇒ "latest" 拿到的是十几秒前的旧值，看起来就像
**"LIO 滞后 13 s / 估计位移是真值的 2.6 倍"**。换成 `MultiThreadedExecutor`（或 `tf2_echo` CLI）
后同一场景立刻变成 **1.002**。**LIO 没问题，是测量工具的锅**。

### 5.2 `mode:=nav localization:=amcl nav:=rpp planner:=navfn`（2D 图 + AMCL）

命令见 §7。结果（`nav_smoke_regression.py --goal -6.0 3.5 --localization RMUC2026-amcl`）：

```
P0 回归：✅ PASS
  · 地图 0.050 m/格 600x341 origin=(-25.93, -9.43)；目标落格 (398, 258) = free
  · 结果 status=SUCCEEDED 用时=91.4s d_min=0.0m recoveries=6
  · 命令链 max|v|,|w|: nav=(0.50,0.75) smooth=(0.50,3.00) chassis=(0.50,3.00)
  · 真值: twist_max=3.000 pose_delta=8.715m
  · RTF=0.49 hz={pcloud 8.0, imu 79.3, scan 8.0, odom 7.7} tf_age=0.1
  · "Control loop missed its desired rate" 出现 1 次
```

⇒ **加载地图 ✅ / AMCL 定位稳定 ✅（map→odom 3 s 窗口漂移 0.0000 m）/ 规划-控制-底盘链路 ✅ / 到点 ✅**。

⚠️ **过程不干净（要如实说）**：91.4 s 里 `controller_server` 报了 **4 次 `Failed to make progress`**，
`behavior_server` 的一次 `Spin` 恢复还**超时失败**（`Exceeded time allowance before reaching the Spin goal
- Exiting Spin`），合计 `recoveries=6`；`pose_delta 8.715 m`（直线只有 7.45 m）说明它绕了路。
即：**"能到点"成立，"走得顺"不成立** —— 与 §6 的"台阶当墙 + 通道窄"完全一致。

### 5.3 `mode:=nav localization:=gicp`（PCD + 3D 重定位）

```
P0 回归：✅ PASS
  · 结果 status=SUCCEEDED 用时=94.1s d_min=0.0m recoveries=6
  · 命令链 max|v|,|w|: nav=(0.50,0.75) smooth=(0.50,3.00) chassis=(0.50,3.00)
  · 真值: twist_max=3.000 pose_delta=9.155m
  · RTF=0.78 hz={pcloud 8.0, imu 78.0, scan 8.0, odom 8.0}
  · ⚠️ 稳定性：settled=False 等待=30.0s 漂移 xy=0.053 m yaw=0.0129 rad（阈值 0.02 m / 0.01 rad）
```

GICP 侧健康指标（`nav_gicp.log` 的 `[status]` 行）：

```
地图点数（GICP target）：原始 26618 → 去 NaN 26618 → 体素 0.100 m 后 21161
地图包围盒：min=(-25.50,-8.93,-0.14) max=(3.65,7.12,0.54)
[status] 采纳 1082/1082 帧 | 最近 score=0.01788 m² | align=21.1 ms | source=3143 点 → target=21161 点
         map→odom x=-0.119 y=-0.029 z=0.225 yaw=-0.65°
```

⇒ **PCD 被正确加载 ✅ / 100% 帧被采纳 ✅ / fitness 0.018 m²（warn 阈值 0.05）✅ / 单帧 align 21 ms ✅（比 RMUL2026 的 327~389 ms 还快，完全不构成 CPU 压力）/ 到点 ✅**。
⇒ 与 §4.2 的设计一致：**`initial_pose: [0,0,0]` 直接就是对的**（收敛后的 `map→odom` 只有 -0.12 m / -0.65° 的修正量），
说明"map 系 = 出生点系 + 地面放 z=-0.06"这套约定是自洽的。
⚠️ 唯一打折项：`map→odom` 在 30 s 窗口里仍在缓慢收敛（0.053 m / 0.0129 rad，约 1.8 mm/s），
没达到回归脚本的 `settle` 判据 ⇒ **不要把这次结果当作"定位收敛后"的干净读数**；
按 GICP"每帧都在精配准"的性质，这是持续微调而不是发散（100% 帧采纳 + score 稳定在 0.018）。
`recoveries=6`、4 次 `Failed to make progress` 与 AMCL 那次**完全同型**。

---

## 6. 坑：这张场地对我们这台车意味着什么（都是量出来的）

| 事实 | 数值 | 对 0.22 m 半径车（直径 0.44 m）的含义 |
|---|---|---|
| 场地可行驶**地板**（0.02 m 栅格，严格平面） | **214.6 m²** | 地面本身 |
| 地图 free（阈值 0.15 m，含台阶根部过渡带） | 237.8 m² | 规划器看到的"白区" |
| clearance ≥ 0.22 m 的集合 | 178.6 m²（同一连通域 176.3 m²） | **真正能走的 ≈ 176 m²（占 free 的 74%）** |
| clearance ≥ 0.30 m | 162.8 m²，但**裂成两块 80.6 / 80.5 m²** | 0.30 m 余量下两个半场不再连通 |
| 两个半场之间最窄处 | **通道宽 0.45 m**（world (±10.58,+9.33) 与 (-14.18,+9.22) 一带）；0.50 m 处在 (-14.22,+5.5) | 余量 5 mm/边 ⇒ **实际过不去**；A* 给出的过场路线长 **41.7 m**（绕场边） |
| 台阶高度 | **0.20 m 与 0.30 m**（另有 1 mm 级装饰面），高度突变 60~90° | 轮半径 0.06 m ⇒ **物理上上不去**；因此 2D 图把它们画成障碍、PCD 里保留（LiDAR 确实能看到） |
| 隧道 / 设计坡道 | **没有**（`.tmp_cache/stl_view/` 的 ray-parity 结论：≥0.25 m 净空的 subsurface 空洞只有 3.7 m²、全是 ≤0.15 m 宽的缝） | 不要指望"钻洞"或"爬坡"路线 |
| 0.15 m 高处的自由空间 / 走廊中位宽 | 240.9 m² / **0.25 m**；28 处 < 0.70 m 的卡点 | 这些卡点大多在两个角部结构内部 ⇒ 里侧基本是死区 |

**结论**：场地能用（地图正确、能定位、能规划到点），但**它是一场"全场级"障碍赛**，
不是我们平时跑的 12 x 8 m 半场。想在这个 world 里做有意义的导航考核，
目标点应当选在**出生点所在的半场**（约 176 m² 可达），跨半场的目标请当作"已知不可达"。

---

## 7. 怎么自己跑（复制即可）

```bash
cd <repo>
source /opt/ros/humble/setup.bash && source install/setup.bash

# 0) 资产是新加的 ⇒ 先 build（install/ 是逐文件符号链接，不 build 看不到新文件）
colcon build --symlink-install --packages-select hzmi_rm_simulation rm_nav_bringup
source install/setup.bash

# 1) 建图（Gazebo + FAST-LIO + slam_toolbox；带 RViz 看）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=mapping lio:=fastlio mapper:=slam_toolbox nav_rviz:=False lio_rviz:=True

# 2) 导航 + AMCL（2D 栅格图路线，实测 PASS）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=nav lio:=fastlio localization:=amcl nav:=rpp planner:=navfn spin_speed:=0.0

# 3) 导航 + GICP（3D 先验点云路线）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=nav lio:=fastlio localization:=gicp nav:=rpp planner:=navfn spin_speed:=0.0

# 4) 无头跑一次 P0 回归（另开一个终端；栈已在跑）
python3 tools/scripts/regress/nav_smoke_regression.py --goal -6.0 3.5 --localization RMUC2026-amcl
```

`--show-args` 里 `world` 是自由字符串，新值不需要额外声明：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py --show-args | grep -A2 "'world'"
```

**回滚**：本 world 的全部改动都是**新增文件 + 两处 launch 加行**，没有改任何既有资产：

```bash
git rm -r src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUC2026_world \
           src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world \
           src/bringup/rm_nav_bringup/map/RMUC2026.pgm src/bringup/rm_nav_bringup/map/RMUC2026.yaml \
           src/bringup/rm_nav_bringup/PCD/RMUC2026.pcd docs/worlds.md
git checkout -- src/simulation/rm_simulation/hzmi_rm_simulation/launch/rm_simulation.launch.py \
                src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py
colcon build --symlink-install --packages-select hzmi_rm_simulation rm_nav_bringup
```
（等价于 `git revert <本次 commit>`；`world:=RMUC2026` 消失后其余 world 完全不受影响。）

---

## 8. 未验证 / 已知未做

* `localization:=beluga / small_gicp / icp / slam_toolbox / cartographer` 在 RMUC2026 下**都没跑**：
  `beluga` 缺 `.pgm` 之外的 apt 包依赖验证、`slam_toolbox`/`cartographer` 需要 `.posegraph`/`.pbstream`
  （本 world 没有建），`icp`/`small_gicp` 与 `gicp` 共用同一份 PCD（大概率同上，但**未实测**）。
* `mode:=slam_nav`、`nav:=dwb/teb/mppi`、`planner:=smac2d`、`global_obstacle/local_obstacle` 各槽位
  在 RMUC2026 下**未做 A/B**。
* 跨半场（41.7 m 绕行）的目标**没有实测**，只有几何上的"最窄 0.45 m"证据。
* 场地里的 **0.20/0.30 m 台阶能否被更小的车（半径 < 0.2 m）通过**、以及"能不能爬上去"，
  没有做参数化实测。
* STL 的**非水密/重复面**对 ODE 的长期影响（长时间接触、多个接触点）没有做压力测试，
  只跑了 3 次短程（< 3 min/次）。
* 世界文件里 5 盏灯的**实际照度**没有校验（只保证与 RMUL2026 同参数、位置平移）。
* `tools/scripts/world/stl_to_world.py` 的产物只做了**文件级回归**（pgm 逐字节、world/model 去注释逐行、
  PCD 点数与 bbox，见 §4.4），**没有**再起一次 Gazebo 验证 `--world-name RMUC2026_TEST` 能 spawn；
  `RMUC2026` 本身的无头实测见 §5。


---


## 来源 3/7：`sim/worlds_2027.md`


> 这是 `docs/worlds.md`（2026 系列，RMUC2026 为主）的续篇。本文只讲两件事：
> **两个 2027 的 STEP 怎么变成 Gazebo 能吃的 STL/world/model**，以及**做完之后实测到了什么**。
> 2026 的四个 world、地图/PCD、launch、`src/localization/rm_localization/**` **一个字节都没动**（边界与回滚见 §7/§8）。
>
> 打开方式（**还没接进 launch**，原因见 §6.3）：
>
> ```bash
> # install/ 是逐文件符号链接，新增资产必须重新 build（见 docs/worlds.md §2.3）
> colcon build --symlink-install --packages-select hzmi_rm_simulation && source install/setup.bash
> gzserver src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUL2027_world/RMUL2027_world.world
> gzserver src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUC2027_world/RMUC2027_world.world
> ```

---

## 0. 一句话

两个 2027 CAD（`RMUL2027_3V3_V1.0.0.stp` 1.99 MB / 1,274 个曲面；
`RMUC2027_V1.0.0.stp` **653 MB** / 214,794 个曲面 / 629 个实体）用 **gmsh 4.15.2 的 OCC 导入 + 2D 三角化**
转成毫米单位 STL，再按 2026 的老约定（`<scale>0.001` + 位姿让**可行驶地面落在 world z = 0**）
包成 `<NAME>_world/{meshes/<NAME>.stl, model.config, model.sdf, <NAME>_world.world}`。
**无头 gzserver 11.10.2 实测**：两个 world 都 **< 1.4 s** 加载、**0 条** mesh 解析错误；
RMUL2027 地面实测 **z = −0.000009 ~ +0.003 m**（0.20 m 平台 +0.200、0.42 m 环墙 +0.400 也对上）；
RMUC2027 低区地面实测 **z = −0.012 ~ +0.013 m**（与从 STL 算出的预测逐点差 ≤1 mm），中央高台/坡道 +0.19~+0.24。
**出生点没定**（这是本文唯一留给人的事，见 §6）。
**2027-10-28 追加**：两个场地四周各加了一圈 **1.5 m 高的静态围栏**（立柱 + 3 根横杆），
理由/边界/参数/实测见 **§9**（场地 mesh 与 §1–§6 的数字都不变）。
**2027-10-29 追加**：RMUC2027 中央塔在 world z **1.50–1.85** 有一段 0.35 m 真断口（网格里一个面都没有），
看着像悬空 ⇒ 加了 `RMUC2027_tower_support` 把它接回下段桁架（**不改场地 mesh**），见 **§9.7**。

⚠️ RMUC2027 是**两步**的：gmsh 926 万面 → 二分简化到 30 万面（653 MB 的 STEP 直接三角化出
**9,257,114 面 / 2.52 GB ASCII**，不可能直接进仓/进 Gazebo），过程与代价见 §2.3b。

---

## 1. 资产清单（本次新增，全部是"新增文件"）

| 文件 | 字节 | 说明 |
|---|---|---|
| `src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUL2027_world/RMUL2027_world.world` | 11,276 | 世界文件（结构镜像 `RMUL2026_world.world`，只改 mesh/位姿/灯/state） |
| `.../world/RMUL2027_world/model.config` + `model.sdf` | 413 / 2,739 | `model://RMUL2027_world` 的独立入口（与 world 内联那份数值一致） |
| `.../world/RMUL2027_world/meshes/RMUL2027.stl` | 1,726,834 | 200 mm 三角化；**竖直轴是 Y 且 +y 朝下**；mm 单位 |
| `src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUL2027_world/…` | 同上 4 个 | **`model://` 真正解析到的那份**（`GAZEBO_MODEL_PATH=<share>/hzmi_rm_simulation/meshes`，见 `docs/worlds.md` §2.3）；与 `world/` 那份逐字节相同 |
| `.../world/RMUC2027_world/RMUC2027_world.world` | 9,656 | 同构 |
| `.../world/RMUC2027_world/model.config` + `model.sdf` | 625 / 2,791 | 同构 |
| `.../world/RMUC2027_world/meshes/RMUC2027.stl` | **符号链接** | → `../../../meshes/RMUC2027_world/meshes/RMUC2027.stl`（见下） |
| `src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUC2027_world/meshes/RMUC2027.stl` | **18,933,384** | **2027-10-29 换新**：组委会 STL（378,666 面、米制）已烘成 mm + 世界系（地面 z=0、塔上段下移 0.959 m），见 §9.8。旧的是 300,000 面/15,000,084 B（STEP 简化产物） |
| `.../world/RMUC2027_fence/model.config` + `model.sdf` | 467 / 62,390 | 围栏（44 立柱 + 12 横杆，纯 SDF 盒子，无 mesh）；`<include>` 进 world。见 §9 |
| `src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUC2027_fence/…` | 同上 | `model://RMUC2027_fence` 的解析根 |
| `.../world/RMUL2027_fence/model.config` + `model.sdf` | 465 / 36,126 | 围栏（20 立柱 + 12 横杆）；同上 |
| `src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUL2027_fence/…` | 同上 | 同上 |
| `tools/scripts/world/perimeter_fence.py` | ~9 KB | 生成上面两份围栏的**唯一来源**（改参数重跑即可，不要手改生成的 model.sdf） |
| `.../world/RMUC2027_tower_support/model.config` + `model.sdf` | 518 / 5,810 | 中央塔断口支撑（2 方柱 + 2 横撑，纯 SDF 盒子）；`<include>` 进 RMUC2027 world。见 §9.7 |
| `src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUC2027_tower_support/…` | 同上 | `model://RMUC2027_tower_support` 的解析根 |

**为什么有两个目录树**：`world/` 是 `rm_simulation.launch.py` 里 `world_path` 的落点；
`meshes/` 才是 `model://` 的解析根（`package.xml` 的 `<export><gazebo_ros gazebo_model_path="${prefix}/meshes"/>`）。
2026 的 RMUL2026/RMUC2026 就是这个双份结构，本轮照抄；**实际被加载的是 `meshes/` 那份**。
RMUL2027 的 1.7 MB 直接存了两份（与 2026 一致）；**RMUC2027 的 15 MB 只存一份**，
`world/` 那边放一个相对符号链接把目录结构补齐（避免 2×15 MB 的纯冗余；Gazebo 不会去读那一份）。

---

## 2. 转换配方（可复跑，本文所有数字都是它跑出来的）

### 2.1 环境

`gmsh 4.15.2`（`third_party/.gmsh/venv/bin/python`，带 `trimesh 5.2.0`）+ Gazebo Classic `11.10.2`；
RMUC2027 的简化另需 `fast-simplification 0.2.0`（本轮装进同一个 venv：
`third_party/.gmsh/venv/bin/pip install fast-simplification`，venv 在 `third_party/` 下、不入仓）。
输入是只读附件：

* `/home/weicheng/.dsh/attachments/v1/files/7d/7d47c1b6…/RMUL2027_3V3_V1.0.0.stp`（1.99 MB, AP214, mm, Creo）
* `/home/weicheng/.dsh/attachments/v1/files/a5/a5ad3ede…/RMUC2027_V1.0.0.stp`（653,443,279 B, AP214, mm, Creo）

### 2.2 STEP → STL（gmsh，OCC 导入）

```python
import gmsh
gmsh.initialize(); gmsh.option.setNumber("General.Terminal", 0)
gmsh.merge(step)                                      # OCC 读 STEP
gmsh.option.setNumber("Mesh.MeshSizeMax", size_mm)    # 200 / 400（毫米！）
gmsh.option.setNumber("Mesh.MeshSizeMin", size_mm*0.5)
gmsh.option.setNumber("Mesh.Algorithm", 6)
gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
gmsh.model.mesh.generate(2)                           # 只要面网格（不做体网格）
gmsh.write(out_stl)                                   # ASCII STL
gmsh.finalize()
# ASCII → 二进制（约 1/3.6 体积）：trimesh.load(ascii, process=False).export(bin)
```

**关键**：`MeshSizeMax` 是**上限**，不是"目标"。gmsh 对**每个曲面**独立三角化，小曲面（螺栓、倒角、
薄壁、孔）仍按 `MeshSizeMin` 加密 ⇒ 三角数由**曲面个数**主导。RMUL2027 只有 1,274 个曲面 ⇒ 200 mm 就只有
34,535 面；RMUC2027 有 **214,794 个曲面** ⇒ 400 mm 仍然炸出 **926 万面**。

### 2.3 三个网格的实际参数与产物

| 场地 | STEP | 曲面数 | `MeshSizeMax/Min` | 三角数 | 二进制 STL | ASCII STL | merge / mesh / write | 备注 |
|---|---|---|---|---|---|---|---|---|
| **RMUL2027** | 1.99 MB | 1,274 | 200 / 100 mm | **34,535** | **1,726,834 B (1.65 MiB)** | 6,178,202 B | 0.6 s / 0.9 s / 0.2 s | 本文采用 |
| RMUL2027（粗一档） | 同上 | 1,274 | 400 / 200 mm | 13,912 | 695,684 B | 2,298,776 B | ≈1.5 s 总 | 只做对照，未入仓 |
| **RMUC2027** | 653 MB | 214,794 | 400 / 200 mm | 9,257,114 → **300,000** | **15,000,084 B (14.3 MiB)** | 2,516,939,574 B（2.52 GB） | 846.8 s / 519.3 s / 27.6 s | gmsh + 简化，见 §2.3b |

同一份输入重跑是**逐字节确定**的：RMUL2027 200 mm 两次独立跑的 ASCII STL md5 都是 `0b24f6c94dbcfd75b18613ad5edb0ddf`。

> 选 200 mm 而不是 400 mm（RMUL）：2026 的 `RMUL2026.stl` 只有 3,187 面（788 KB ASCII），本轮 34,535 面
> 已经比它细 10 倍；再粗到 400 mm 只省 1 MB，却把台阶/薄墙的细节丢掉了。

### 2.3b RMUC2027：653 MB STEP 的两步走（含真实耗时/内存）

| 步骤 | 时间 | 峰值 RSS | 结果 |
|---|---|---|---|
| `gmsh.merge(step)`（OCC 读 STEP） | **846.8 s（14.1 min）** | 6.3 GB | 408,702 点 / 622,820 曲线 / **214,794 曲面** / 629 实体 |
| `mesh.generate(2)` @ 400 mm | **519.3 s（8.7 min）** | 8.1 GB | 9,257,114 三角 |
| `gmsh.write()` ASCII | **27.6 s** | 8.1 GB | **2,516,939,574 B = 2.52 GB** |
| 剔除孤立小件（连通分量 bbox 对角 < 0.1 m） | 4 s | 0.6 GB | 1,755 → 618 个分量，833,889 → 812,595 面（只掉 2.5% 面积） |
| `fast_simplification`（quadric，`agg=7`）→ 300,000 面 | 4.8 s | 1.0 GB | **300,000 面 / 15,000,084 B**，面积 1,737.1 → **1,688.5 m²（−2.8%）**，bbox 不变 |
| 合计（不含简化） | **≈ 23.3 min** | 8.1 GB | —— |

为什么必须简化：2.52 GB ASCII / 926 万面的 STL 既进不了仓库（二进制也要 463 MB），
Gazebo 的 visual + ODE trimesh 也不可能拿它当碰撞体（对照：RMUC2026 只有 45,900 面 / 2.3 MB）。
**没有"再做细一点"的中间档**：纯 gmsh 的下限 ≈ 曲面数 × 1.5 ≈ 32 万面，和最终 30 万面是同一个量级，
所以这一步不是"偷工"，而是**已经到这份 CAD 的粗档底**。简化的代价（面积 −2.8%）在 §4.3 用落板实测核对过。

---

## 3. 坐标与位姿约定（把 2026 文档里含糊的一处钉死）

### 3.1 RMUL2027：mesh 的 +y 指向**地面以下**

`RMUL2027.stl` 的 bbox 是 `x ±6300、y −420…+100、z ±4300`（mm），和 **`RMUL2026.stl` 的 bbox 完全相同**
（12600 × 520 × 8600 mm），所以 2026 那套 `<scale>0.001` + 绕 X 转 −90° 直接可用。但**方向必须实测**：

| mesh 局部高度 | 水平面积 | 是什么 |
|---|---|---|
| `y = +100 mm` | **108.36 m²**（= 12.6 × 8.6，整个外框） | 100 mm 厚底板的**下表面**（从上方看不见） |
| `y = 0 mm` | **75.6 m²** | **可行驶地面**（顶面）；从上方打 10,836 条竖直射线，52.6% 首个命中在这里 |
| `y = −10 mm` | 9.0 m² | 中央 10 mm 厚的垫高/装饰层 |
| `y = −200 mm` | 15.9 m² | 0.20 m 高的平台 |
| `y = −420 mm` | 12.4 m² | 0.42 m 高的四周环墙顶面 |

⇒ mesh 不是"Y 朝上"，而是**竖直轴是 Y、+y 朝下**。用**精确** `Rx(−90°)`（`-1.5707963`，
不是 2026 那份的 `-1.5705`）时 `world z = −0.001·y`，于是：

* 地面 `y=0` ⇒ `world z = 0`，**pose 的 z 就是 0**（`model pose` 与 `visual/collision pose` 都是 0）；
* 环墙 `y=−420` ⇒ `+0.42 m`，平台 `y=−200` ⇒ `+0.20 m`（§4.1 落球逐条对上）；
* 底板下表面 `y=+100` ⇒ `z=−0.10 m`（在地下，不参与）。

> 2026 用的 `-1.5705` 让 `cos≠0`，全场多出 `≈0.0003 × 4300 mm ≈ 1.3 mm` 的倾斜；2027 这两份改用精确 −90°，
> 这一项为 0。**2026 的 world 没有改**（本文只新增）。

### 3.2 RMUC2027：Z 朝上，低区地面在 `z = −1700 mm` ⇒ pose z = `+1.700`

与 RMUC2026 同一套（`docs/worlds.md` §2.1）：mesh 本来就是 Z 朝上、**不需要旋转**，只把地面抬到 `z=0`。
抬升量的取法（**两处互相印证**）：

1. **面积加权水平面众数**（2026 用的算法；先在 RMUC2026 上复算得 `−1641.3440 mm`，与文档的 `−1.6413436 m`
   差 0.4 µm，算法可信）→ 但这个"最大单平面"在 RMUC2027 上**不适用**：最大的水平面是 `z=−1941.34` 的
   435.3 m²（底板**下表面**），其余水平面都被切碎（最大的"地面"平面只有 33 m²）。
2. **顶层高度图 + "局部平坦格"众数**（本轮新增的判据，能自动排除坡道/立面）：
   0.2 m 栅格、3×3 邻域起伏 < 20 mm 的格 = 局部平面，取这些格的层级众数 = **−1700 mm**
   （−1700 / −1690 / −1710 三档各 ~360 格，峰顶很平）。
   同一判据在 RMUC2026 上给出 `−1640 mm`、且最大连通平坦块 **156 m²**（与文档的地面吻合）——
   而 RMUC2027 最大连通平坦块只有 **8 m²**：**这张场地的地面本身就不是一整块平面**。

⇒ pose z = **+1.700 m**（**2027-10-29 起改为 pose z = 0**：新 STL 已把"地面=0"烘进网格，见 §9.8，本节其余结论不变）。**注意这张场地的地面确实不平**：顶层高度四分位是 `−1712 / −1484 mm`，
低区（−1.70 附近）之外还有中央高台/四条坡道（−1.50 档）与 0.42 m 环墙；实测分布见 §4.3。
"地面在 z=0"这句在这张图上**只对低区成立（±13 mm）**，比 RMUL2026/RMUC2026 的"一整块平面"弱一档。

### 3.3 场地摆在世界的哪里

| 场地 | model pose | world 里的场地外框 (m) | world 里的 z 范围 (m) | 说明 |
|---|---|---|---|---|
| RMUL2027 | `0 0 0 0 0 0` | `x [−6.300, 6.300]`、`y [−4.300, 4.300]` | `[−0.100, +0.420]` | 旋转后 `world x = mesh x`、`world y = mesh z`；中心 = **世界原点** |
| RMUC2027 | `0 0 0 0 0 0` | `x [−14.000, 14.000]`、`y [−6.377, 9.627]` | `[−0.241, +3.557]` | mesh 米制坐标 == world 坐标（与 RMUC2026 同口径）；中心 `(0, +1.625)` |

RMUL2026 是把场地挪到 `(8.5, 4.5)`；2027 这两份**放在原点/mesh 原坐标**，理由：没有既有地图/PCD/出生点要继承，
而且和 RMUC2026 的"mesh 米制坐标 == world 坐标"一致，后续量什么都少一步换算。

---

## 4. 实测（`gzserver 11.10.2`，headless，私有 `HOME` / `GAZEBO_MASTER_URI`）

量法：`gz model -p -m <model>` 拿模型位姿；**落体**拿"真实接触面"——
球（r=0.1 m）与薄板（0.3×0.3×0.02 m，μ=5，从预测面上方 0.03 m 松手）。
薄板是给 RMUC2027 用的：那张地面有起伏，球会顺坡滚出场地（第一轮 10 个球里有 4 个滚到 z=−30~−83 m），
板子不滚。接触面 z = 板心 z − 0.01（球则 −0.1）。

### 4.1 RMUL2027：`RMUL2027_world.world` 直接加载

| 项 | 实测 |
|---|---|
| gzserver 启动 → `gz model -p` 可查 | **1.10 s**（复跑 0.87 s；1.65 MiB 网格） |
| `gz model -p -m RMUL_2027` | `0 0 3e-06 0 -0 0`（`link` 的 −3 µm 底噪，等价 0） |
| 网格/材质解析错误 | **0 条**（日志里只有 headless 必然出现的 `RenderEngine` 警告） |
| ODE / trimesh 告警 | **0 条** |
| `model://` 解析 | 走 `GAZEBO_MODEL_PATH=<...>/meshes` → `RMUL2027_world/meshes/RMUL2027.stl` |
| `gz model -i` | `is_static: true`（与 2026 一致） |

落球（球心 z，地面 = z − 0.1）：

| 落点 (world x, y) | 预期 | 球心 z | ⇒ 地面 z | 判读 |
|---|---|---|---|---|
| `(−2.85, −1.55)` | 地面 0 | 0.100000 | **−0.000000** | ✅ |
| `(0, −2)` | 地面 0 | 0.099991 | **−0.000009** | ✅ |
| `(0, +2)` | 地面 0 | 0.100000 | **0.000000** | ✅ |
| `(5, 0)` | 地面 0 | 0.099991 | **−0.000009** | ✅ |
| `(−5, +2)` | 地面 0 | 0.103000 | **+0.003000** | 3 mm 装饰层（STL 同点同值） |
| `(−0.85, +0.05)` | 10 mm 垫高层 | 0.110000 | **+0.010000** | ✅ |
| `(2, 0)` | 0.20 m 平台 | 0.300000 | **+0.200000** | ✅ |
| `(6.2, 4.2)` | 0.42 m 环墙 | 0.500002（滚落后） | **+0.400002** | ✅（落在 0.40 m 那圈墙顶） |

⇒ **地面落在 world z = 0（误差 ≤3 mm，且那 3 mm 是几何本身就有的装饰层）**，
与 2026 的约定一致（`spawn_z` / 出生高度逻辑不需要改）。

### 4.2 bbox：Gazebo 11 没有 AABB 导出，用"STL bbox × scale + pose" + 落体交叉验证

`gz model -p` / `-i` 只给位姿与结构，没有 AABB。所以 bbox 这样给：`pose = 单位矩阵`、`scale = 0.001`
⇒ **world AABB = mesh bbox / 1000**，再用落体在场地四角/边中实测高度，确认世界坐标与几何对得上。

| RMUL2027 | x (m) | y (m) | z (m) |
|---|---|---|---|
| CAD bbox（mm/1000） | `[−6.300, +6.300]` | `[−4.300, +4.300]` | `[−0.420, +0.100]` |
| world AABB（旋转后，Gazebo 里看到的） | `[−6.300, +6.300]` | `[−4.300, +4.300]` | `[−0.100, +0.420]` |

`z` 的上下界互换是旋转的必然结果（mesh 的 `y` 轴映射到 world 的 `z` 且符号翻转）。

### 4.3 RMUC2027：`RMUC2027_world.world` 直接加载

| 项 | 实测 |
|---|---|
| gzserver 启动 → `gz model -p` 可查 | **0.91 s**（带探针那份 0.95 s / 1.38 s；14.3 MiB / 30 万面） |
| `gz model -p -m RMUC_2027` | `0 0 0 0 -0 0` |
| 网格/材质解析错误 | **0 条** |
| ODE / trimesh 告警 | **0 条**（30 万面的静态 trimesh 没有触发任何告警） |
| `gz model -i` | `is_static: true` |

落板（板心 z，接触面 = z − 0.01；"预测"= 从 STL 顶层高度图算的同一层）：

| 落点 (x, y) | 预测接触 z | 实测板心 z | ⇒ 实测接触 z | 与预测差 |
|---|---|---|---|---|
| `(−12.9, +3.32)` | +0.013 | 0.023331 | **+0.013331** | +0.000 |
| `(+12.9, −0.68)` | −0.002 | 0.008375 | **−0.001625** | +0.000 |
| `(+12.7, +4.32)` | −0.012 | −0.002029 | **−0.012029** | −0.000 |
| `(−10.5, −1.08)` | −0.012 | −0.002092 | **−0.012092** | −0.000 |
| `(−7.9, +3.72)` | +0.004 | 0.013687 | **+0.003687** | −0.000 |
| `(+11.3, +2.33)`（高台/坡道侧） | +0.195 | 0.219467 | **+0.209467** | +0.014 |
| `(−11.3, +2.32)`（高台/坡道侧） | +0.188 | 0.254835 | **+0.244835** | +0.057 |

⇒ **低区地面实测 z ∈ [−0.012, +0.013] m**（5 个点，与 STL 预测逐点差 ≤ 1 mm）；
高台/坡道区实测 **+0.19 ~ +0.24 m**（那一区本身有坡度，板子会顺着落到坡上，差 1~6 cm 是坡面不是误差）。
**RMUC2027 的"地面在 z=0"只对低区成立**，这与 §3.2 的结论一致。

| RMUC2027 | x (m) | y (m) | z (m) |
|---|---|---|---|
| CAD bbox（mm/1000） | `[−13.99999, +14.00002]` | `[−6.37722, +9.62717]` | `[−1.94134, +1.85734]` |
| world AABB（+1.700 之后） | `[−14.000, +14.000]` | `[−6.377, +9.627]` | `[−0.241, +3.557]` |

---

## 5. 网格质量与碰撞

* **RMUL2027** 200 mm：34,535 三角形、103,605 顶点、总表面积 **375.70 m²**、`is_watertight = False`
  （gmsh 的 STL 不合并顶点，每个三角形独立）。
* **RMUC2027** 简化后：300,000 三角形、900,000 顶点、总表面积 **1,688.49 m²**（原始 1,737.14，−2.8%）、
  `is_watertight = False`。
* `<visual>` 与 `<collision>` 用**同一份 STL、同一个 `<pose>`/`<scale>`**，不做凸包/简化碰撞体
  —— 与 2026 同款做法（`docs/worlds.md` §2.4 已论证过这种 raw mesh 直接当碰撞可用）。
  RMUL2027 的 3.5 万面 < RMUC2026 的 4.6 万面；RMUC2027 的 30 万面是 2026 的 6.5 倍，
  但静态 trimesh 只在加载时建一次 BVH，实测加载 0.91 s、无告警。
* **RMUL2026 vs RMUL2027 几何不一样**（虽然是同一块 12.6 × 8.6 的场地）：两份 STL 各做 0.1 m 顶层高度图
  逐格相减，**1,026/10,836 格（9.5%）差 > 5 mm、1,018 格差 > 5 cm、最大差 0.40 m**，平均 |差| 19.6 mm。
  ⇒ **2026 的地图/PCD 不能当 2027 的先验**（也是本轮不碰 `map/`、`PCD/` 的原因之一）。
* RMUC2027 的几何里**有东西高到 world z = +3.56 m**（mesh z = +1857）：`z > +1000 mm` 的面有 28,825 个、
  面积 12.9 m²，x 覆盖 ±12.9 m —— 形状像**灯架/桁架/立柱**那类东西（RMUC2026 的 mesh 最高只到 world +1.53 m）。
  它不影响物理（静态），但**会挡雷达视线**，用之前要知道。

---

## 6. 出生点：**没有**（唯一留给人决定的事）

### 6.1 场地中心/外框（直接可用的数）

| 场地 | 世界外框 (m) | 中心 (m) | 地面 | 最高点 |
|---|---|---|---|---|
| RMUL2027 | `x [−6.30, 6.30]`、`y [−4.30, 4.30]` | `(0, 0)` | 主地面 `z = 0` | `+0.42`（四周环墙） |
| RMUC2027 | `x [−14.00, 14.00]`、`y [−6.377, 9.627]` | `(0, +1.625)` | 低区地面 `z = 0`（±13 mm，见 §4.3） | `+3.56`（灯架类） |

### 6.2 如果只想先跑起来：两个**候选**（不是结论）

判据与 2026 一致：把"相对地面抬升 > 0.15 m"的格当障碍，在可行驶面上算到最近障碍的欧氏距离，取最远点。

| 场地 | 候选 (x, y) | clearance | 该点地面高度 | ±0.55 m 起伏 |
|---|---|---|---|---|
| RMUL2027 | **`(−0.85, +0.05)`** | **4.20 m** | +0.010 m（10 mm 中央垫高层） | 1 mm |
| RMUC2027 | **`(+5.90, +2.72)`** | **6.80 m** | +0.030 m | —— |

次优（对称点）：RMUL2027 `(−2.85, −1.55)` / `(3.15, 1.55)`（2.7 m）；
RMUC2027 `(−5.70, +2.72)` / `(+5.90, +0.52)` / `(−5.90, +0.52)`（6.8 m）。

⚠️ **这两个点没验证过能不能出生**（没起机器人、没跑 LIO、没跑导航）：本轮只做了"场地本身"的无头验证。
真要用，按 2026 的口径给 `z=0.2`（轮半径 0.06 ⇒ 落地 base_link 0.06，留 0.14 m 余量；
`docs/worlds.md` §3 解释过为什么不写 1.16）。

### 6.3 为什么没接进 launch

`gzserver <world>` 能直接开（§0），但 `ros2 launch … world:=RMUL2027` **现在会失败**：
`rm_simulation.launch.py::get_world_config()` 是 `world_configs.get(WorldType[...])`，
没登记的 `WorldType` 直接抛 `KeyError`，而 `WorldType` 里也没有 2027 这两个值。
按本轮的任务边界（**不动 launch**），这一步留空；接的时候只需两步：
`WorldType` 加两个值 → `world_configs` 加两条（`x/y/z/yaw` + `world_path`）。
`rm_nav_bringup/launch/bringup_sim.launch.py` 的 `amcl_init_x/y` 字典同理（2026 就是这么加的）。

---

## 7. 未验证 / 已知未做

* **出生点没实测**（§6.2 只是几何候选）；机器人相关的一切（`spawn_z`、LIO 重力初始化、台阶/坡道能不能上、
  `/scan` 里看不看得见地面、map/PCD 建图与重定位）**全部没跑**。
* **没接 launch**（§6.3）；`world:=RMUL2027` / `world:=RMUC2027` 目前不可用，只能 `gzserver` 直接开。
* **没做 2D 栅格图 / 先验 PCD**：按任务边界不碰 `map/`、`PCD/`（§5 也已证明 2026 的几何对不上）。
* **没在 gzclient（GUI）里看过**：headless 环境无 X（`RenderEngine`: `Unable to create X window`），
  只验证了加载与物理；材质/光照/视觉观感未验证。
* **AABB 不是 Gazebo 导出的**（§4.2 说明了原因与替代验证）。
* **RMUC2027 的简化只做了一次面积核对 + 7 个落板点**：没有逐区域对比简化前后的高度图，
  也没有做"机器人长时间多接触点"的碰撞压力测试。薄壁/细杆被简化吃掉的可能性不能排除
  （面积 −2.8% 主要来自这类小面）。
* **RMUC2027 的地面不是单一平面**（§3.2/§4.3）：如果后续要求"全场地面 z=0"，
  需要按半场/区域分别校准，不能只用一个 pose 的 z。
* 2026 的四个 world、`map/`、`PCD/`、`launch/`、`src/localization/rm_localization/**` **未改动**。

---

## 8. 回滚

本轮全是新增文件（+ 本文 + `docs/worlds.md` 顶部一行指引），最大的是两份 STL（§1）。回滚：

```bash
git rm -r src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUL2027_world \
         src/simulation/rm_simulation/hzmi_rm_simulation/world/RMUC2027_world \
         src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUL2027_world \
         src/simulation/rm_simulation/hzmi_rm_simulation/meshes/RMUC2027_world \
         docs/worlds_2027.md
git checkout -- docs/worlds.md
```

---

## 9. 围栏：两个场地的四周各加一圈静态围栏（2027-10-28 追加）

> 本节是上面 §0–§8 的**增量**：§1–§6 说的"场地"现在都多了一圈围栏（`<include>` 进 world，
> 场地 mesh 一个字节没改）。加它的理由、量出来的边界、参数、实测都在这里。

### 9.1 为什么：STEP 场地是"秃"的，而真实赛场有一圈边界结构

| 证据 | 数字 |
|---|---|
| `RMUL2027.stl` 几何最高点 | **+0.42 m**（就是场地自带的边界矮墙），再没有更高的东西 |
| `RMUC2027.stl` | 最高的东西集中在**中央**（龙门/灯塔，world z 到 +3.56 m），四周开阔、**没有任何东西把场地围住** |
| 真实赛场点云 `PCD/RMUL.pcd`（1,589,841 点）按 z 分层 | **z 1.0–1.6 m：场地边界上一圈"环"**（10×10 顶视栅格，边界格 2.4k–7.4k 点、内部 ~0 点）；z 1.6–3.0 m 同一圈继续（墙到 ~3 m）；z 3–15 m 另一个边界环 + 顶棚（到 ~14 m）|

⇒ 真实场景是"**一个被墙/围栏/房间包住的场地**"，STEP 只导出了地面与地面上的道具。
对 3D 全局重定位（GICP / KISS / Scan Context）来说，这等于**把最有辨识度的竖直面全丢了**：
场内只剩地面和中央那几个结构，任何位置看过去都长得差不多。围栏把这层结构补回来。

### 9.2 边界：从 STL 量出来的，不是猜的

| 场地 | 量法 | 结论（围栏中心线） |
|---|---|---|
| **RMUC2027** | 沿 x 扫到 ±14.000 还有地面（x=14000 处 z=−1684，x=14100 处**没有几何**）；沿 y 扫出台面在 **−5.85 / +9.15** 结束，再往外是 **0.45 m 宽、低 0.145 m 的外侧低台**（一直铺到 bbox 的 −6.377/+9.627） | **28.00 × 15.00 m**，`x ∈ [−14.00, 14.00]`、`y ∈ [−5.85, 9.15]`（正好是标准 RMUC 场地尺寸）。**不用 bbox**：bbox 在 y 方向多出来的 0.45 m 是低台，不是可行驶面 |
| **RMUL2027** | 可行驶台面（z=0）到 `|x|=5.95~6.00`、`|y|=3.95~4.00` 为止；再往外是场地自带 **0.42 m 高、0.30 m 宽**的边界矮墙，外表面正好是 bbox 的 ±6.30 / ±4.30 | **12.60 × 8.60 m**（= bbox = 矮墙外表面），`x ∈ [−6.30, 6.30]`、`y ∈ [−4.30, 4.30]`；比可行驶台面外扩 0.30 m |

围栏中心线就放在上表这两条线上（立柱中线），所以立柱只有 **0.03 m（半个柱宽）** 落在可行驶边界里，
远小于车半径 0.22 m —— 可行驶区实际不变，出生点/路径不受影响（§9.4 有实测）。

### 9.3 参数（两个场地同一套，由工具生成，不手写）

`tools/scripts/world/perimeter_fence.py`：

```bash
third_party/.gmsh/venv/bin/python tools/scripts/world/perimeter_fence.py \
    --name RMUC2027_fence --xmin -14 --xmax 14 --ymin -5.85 --ymax 9.15
third_party/.gmsh/venv/bin/python tools/scripts/world/perimeter_fence.py \
    --name RMUL2027_fence --xmin -6.30 --xmax 6.30 --ymin -4.30 --ymax 4.30
```

| 项 | 值 |
|---|---|
| 立柱 | `0.06 × 0.06 m`，顶面 `z = +1.50`，**向下埋 0.10 m**（盒子 1.60 m，露出 1.50 m；地面起伏 ≤3 cm 时不会悬空） |
| 立柱间距 | 每条边按 ≈2.0 m 均分：RMUC **2.000 m**（x 向边，14 段）/ **1.875 m**（y 向边，8 段）；RMUL **2.100 m**（6 段）/ **2.150 m**（4 段） |
| 横杆 | 3 根 `0.05 × 0.05 m`，中心 `z = 0.30 / 0.70 / 1.10`，每条边通长（与立柱重叠，静态无妨） |
| 盒子数 | RMUC **44 立柱 + 12 横杆 = 56**；RMUL **20 + 12 = 32**；每个盒子 1 个 `<collision>` + 1 个 `<visual>` |
| 物理 | `<static>true</static>`、单 link、`mu = mu2 = 1`、restitution 0；**场地 mesh 未改** |
| 门 | **没有门，连续一圈**（理由见 §9.5） |
| 文件 | `world/<NAME>/model.{config,sdf}` + `meshes/<NAME>/model.{config,sdf}`（后者是 `model://` 的解析根）；world 里只加一个 `<include><uri>model://<NAME></uri></include>` |

> RMUL2027 的最下面那根横杆（z=0.30）**被 0.42 m 高的矮墙挡住**，视觉上只看得到 0.70 / 1.10 两根
> 和立柱露出的 0.42–1.50 m 段 —— 这正是"围栏架在矮墙顶上"的样子（不是 bug）。

### 9.4 实测（headless gzserver 11.10.2）

| 项 | RMUC2027_fence | RMUL2027_fence |
|---|---|---|
| 加载（world 整体） | **0.76 ~ 1.02 s** | **0.87 ~ 0.96 s** |
| `gz model -p -m <fence>` | `0 0 0 0 -0 0` | `0 0 0 0 -0 0` |
| `gz model -i` 数到的 collision / visual | **56 / 56** | **32 / 32** |
| mesh/ODE 错误与告警 | **0 条** | **0 条** |
| 解析 AABB（从 `model.sdf` 逐盒算） | `x[−14.030, 14.030] y[−5.880, 9.180] z[−0.100, 1.500]` → **28.06 × 15.06 × 1.60 m** | `x[−6.330, 6.330] y[−4.330, 4.330] z[−0.100, 1.500]` → **12.66 × 8.66 × 1.60 m** |
| 立柱高度（落板：0.045 板从 1.503 m 松手） | 4 个角柱 + 2 个边中柱 **全部板心 1.505** ⇒ 接触面 **1.500** | 同左，**板心 1.505** ⇒ **1.500** |
| 立柱位置 | 板心落在 `(∓14.000, −5.850)` / `(∓14.000, +9.150)` / `(0, −5.850)` / `(14.000, 1.650)` | `(∓6.300, ∓4.300)` / `(0, +4.300)` / `(6.300, 0)` |
| 横杆高度（0.04 板从 1.116 m 松手） | `(14,2.6)` / `(0,9.15)` / `(14,8.9)` 全部 **板心 1.130** ⇒ 接触面 **1.125**（= 1.10 + 0.025） | `(6.30,1.075)` / `(−2.15,−4.30)` 同样 **1.130** ⇒ **1.125** |
| 不侵占可行驶区（内侧落板） | `(13.5,0)` 底 **+0.0161**、`(13.7,3.0)` 底 **+0.0226**（都落在场地地面，没有被围栏抬高） | `(5.85,0)` 底 **0.000**、`(5.80,−3.5)` 底 **+0.003** |
| 出生候选不受影响 | `(5.90,2.72)` 0.2 m 方块底 **+0.0299**（= §6.2 记的地面 +0.030），距围栏 **6.40 m** | `(−0.85,0.05)` 0.2 m 方块底 **+0.0100**（= 10 mm 垫高层），距围栏 **4.22 m** |
| 机器人高度射线（纯几何：围栏盒子 vs 360° 水平射线，7 层高度） | z=**0.70 / 1.10**（两根横杆）：**360/360 命中**；其余高度只有穿过立柱的 **12/360**；距离 **6.40–21.59 m** | z=**0.70 / 1.10**：**360/360 命中**；其余 **15/360**；距离 **4.22–8.31 m** |

**射线只有 100% 在两根横杆的高度上**，不是缺陷：围栏是**格栅**（2 m 一根柱 + 3 根 5 cm 横杆），
一条水平射线从两根横杆之间穿过去是几何必然；真实 3D 雷达是"一圈一圈扫"，会在横杆/立柱上留下点
（`docs/tilted_lidar_fidelity.md` 那套倾斜视场更是如此）。所以围栏给的是**水平细线 + 竖直立柱**两类特征，
而不是一整面墙。

**一个真 bug（已修，留档）**：生成器最初把"沿 y 方向的两根横杆"按 `y=0` 居中（默认矩形居中于原点）。
RMUC2027 的矩形是 `y ∈ [−5.85, 9.15]`（中心 **1.65**）⇒ 那两根横杆整体偏了 1.65 m：
−y 侧伸出场地 1.65 m、+y 侧短 1.65 m。落板打 `(14, 8.9)` 立刻暴露（修复前那里是空的、板子直接掉到地面）。
修好后 `(14, 8.9)` 落在 **1.130**。RMUL2027 因为矩形居中于原点（`±4.30`），从一开始就没受影响。

### 9.5 为什么不留门

仿真里机器人是**从场内出生**的，不需要开进来；连续一圈对配准最有利（任何朝向都能打到边界）。
真要开门：把落在缺口区间内的立柱跳过、并把该边的横杆拆成两段即可（工具故意没做参数化，免得出现
"半开的门"这种更不真实的中间态）。

### 9.6 没验证的（围栏部分）

* **没接机器人、没跑真实 LiDAR**：9.4 的"射线命中"是**纯几何**检查（围栏盒子 vs 射线），
  不是 Mid360 插件真跑出来的点云；视场遮挡、量程（Mid360 40 m 足够）、反射率都没测。
* **碰撞行为没测**：机器人以多快速度撞上去会不会被推、`mu=1` 合不合适、会不会卡在立柱/横杆之间，
  都没跑过；围栏只有 collision 盒子，没有"可翻越/可撞倒"的动力学（`static`）。
* ⚠️ **围栏不是"物理上关住"车的东西**：最下面一根横杆在 z=0.30（RMUC）/ 0.30 但被 0.42 m 矮墙挡住（RMUL）。
  默认仿真车的底盘碰撞盒是 **0.2 × 0.3 × 0.1 m**（`urdf/simulation_waking_robot.xacro`），顶面约 0.2 m ⇒
  **RMUC2027 里它可以从 0.275 m 那根横杆下面开出去**（只剩每 2 m 一根的立柱挡着），然后掉下 ±x 的场地
  边沿或 ±y 的 0.145 m 低台。要真"关住"就给生成器加一根低横杆（一行参数）：
  `--rails 0.10,0.30,0.70,1.10`。本轮按需求保持 0.3/0.7/1.1，**没有加**。
* `gz model -l`（列出全场模型）在 Gazebo 11 里**不存在**；本轮用 `gz model -p/-i -m <fence>` + 落板 +
  解析 AABB 代替（§9.4）。
* 没把围栏做进 `map/`、`PCD/`（按任务边界不碰），所以现在**围栏只影响 Gazebo 与雷达仿真**，
  不影响 2D 栅格图与先验点云。
* 没在 gzclient（GUI）里看外观（headless 无 X）。
* 围栏的**视觉材质**是 `Gazebo/Grey`（与场地同色），没有做真实赛场的颜色/标志。

### 9.7 中央塔"断口"的支撑（2027-10-29 追加，**已于同日废弃 → 见 §9.8**）

> ⚠️ **本节方案已废弃**：用户要求"不要多一层，要塔自己贴地"，所以独立支撑模型 `RMUC2027_tower_support`
> 已从 world 删除、模型目录也删了；改成**换用新 STL + 把塔上段整体下移**（§9.8）。本节保留作历史/取证。

现象：RMUC2027 中央那套结构看着**悬空**（视觉上像"塔浮在 2.6~3.5 m"）。实测下来不是错觉：
网格里有一段真的没有面。**没有改场地 mesh、没有改围栏、没有改出生点**，加了一个 `RMUC2027_tower_support`。

#### 9.7.1 量出来的断口（全部从 `RMUC2027.stl` 上测）

塔柱体取 `|x| < 1.0 m`、`1.25 < y < 2.05 m`，按 **50 mm** 逐层统计"这一层有没有面"：

| world z 区间 | 面数 | 判读 |
|---|---|---|
| 0.00–0.20 | 1 / 0 / 0 / 0 | 空 —— 但这段是**中央高地表面以下**：塔脚站在 0.2 m 高地上（见下） |
| 0.20–1.50 | 21 … 10621 | 连续（塔的下段桁架：立柱 0.2–0.8、斜撑 0.9–1.5） |
| **1.50–1.85** | **0 / 0 / 0 / 0 / 0 / 0 / 0** | **连续 7 个 50 mm 层完全没有面 ⇒ 真断口，宽 0.35 m** |
| 1.85–2.05 | 30 / 133 / 152 / 213 | 连续（上段桁架） |
| 2.05–3.56 | 是 | 连续（灯架/桁架本体） |

关键补充：
* **不是"细杆被网格吃掉"**：柱体内 1.50–1.85 内**一个三角形都没有**（不只是顶点稀疏），
  且上下两段在空间上没有任何跨越该带的几何。垂直射线在塔心 `(0, 1.62)` 的命中序列是
  `2.85 / 2.58 / 2.57 → 0.22 → 0.05`，中间**直接跳到地面**。
* **塔本身是落地的**：塔脚正下方的地面在 `(0,1.62)` 命中 **0.22 m**（中央 0.2 m 高地顶面）、
  `0.05 m`（场地地面）；下段立柱从 **0.20–0.25** 起。所以"悬空"的**只有 1.50–1.85 这一段**。
* **上下段是外张的斜撑、有 0.15 m 横向错位**（这也是为什么断口看着更明显）：

  | | 下段顶点 (world) | 上段根点 (world) | 横向错位 |
  |---|---|---|---|
  | 左斜撑 | `(−0.36, 1.57, 1.50)` | `(−0.51, 1.56, 1.86)` | 0.15 m |
  | 右斜撑 | `(+0.36, 1.68, 1.50)` | `(+0.50, 1.68, 1.86)` | 0.14 m |

* 顺带量到的小空洞：`0.80–0.90 m` 也有一处 ~0.10 m 的空带（两段桁架接头处，网格没收干净），
  但上下都在 0.1 m 内、视觉上不构成"悬空"，**没处理**（列在这里备查）。

#### 9.7.2 支撑几何（4 个盒子，全在塔的投影里）

| 件 | 尺寸 (m) | 中心 (x, y, z) | 作用 |
|---|---|---|---|
| 左方柱 | `0.16 × 0.12 × 0.52` | `(−0.435, 1.565, 1.660)` | z **1.40–1.92**：下端埋进下段斜撑 0.10 m、上端顶进上段斜撑 0.06 m；x 覆盖 `−0.515…−0.355`，把下段 `−0.36` 与上段 `−0.51` 两个端点都包住 |
| 右方柱 | `0.16 × 0.12 × 0.52` | `(+0.435, 1.675, 1.660)` | 同上（覆盖 `+0.355…+0.515`） |
| 横撑 A | `1.00 × 0.06 × 0.06` | `(0, 1.620, 1.620)` | 把左右方柱连成 H 形框架 |
| 横撑 B | `1.00 × 0.06 × 0.06` | `(0, 1.620, 1.800)` | 同上（第二道） |

* 材质/物理：`Gazebo/Grey`、`static`、单 link、`mu = mu2 = 1`、restitution 0（与场地/围栏一致）。
* **没有做斜撑**：断口只有 0.35 m，这个尺度上斜撑既看不出来也不真实。
* **为什么不做"从地面顶上去的柱子"**：塔在 1.50 以下本来就是连续的（下段桁架 0.2→1.50，塔脚踩在
  0.22 m 高地上），再加一根到地面的柱子只会**在可行驶面上多两根障碍**，不会更真实。
* 支撑在塔的投影内：塔 `1.86–2.10` 的投影是 `x[−0.67,+0.66] y[1.22,2.07]`，
  支撑是 `x[−0.52,+0.52] y[1.50,1.74]` ⇒ **没有比原来的塔多占可行驶面积**；
  支撑的最低点是 z=1.40，机器人（0.2 m 高）从下面过。
* 文件：`world/RMUC2027_tower_support/model.{config,sdf}` + `meshes/RMUC2027_tower_support/…`，
  world 里一个 `<include><uri>model://RMUC2027_tower_support</uri></include>`。

#### 9.7.3 实测

| 项 | 实测 |
|---|---|
| world 加载（含场地 + 围栏 + 支撑） | **0.97 s**，0 条错误/ODE 告警 |
| `gz model -p -m RMUC2027_tower_support` | `0 0 0 0 -0 0`；`gz model -i` 数到 **4 collision / 4 visual** |
| 围栏仍在 | `gz model -p -m RMUC2027_fence` = `0 0 0 0 -0 0` |
| 解析 AABB | `x[−0.515, 0.515] y[1.505, 1.735] z[1.400, 1.920]` = **1.03 × 0.23 × 0.52 m** |
| 方柱顶（0.06 立方体从 2.10 m 松手，落点 `(−0.36,1.57)`） | 停在 **1.95** ⇒ 接触面 **1.92** = 方柱顶 ✓（正好埋进上段斜撑） |
| 横撑顶（0.04 板从 1.803 m 松手，落点 `(0,1.62)`） | 板心 **1.83499** ⇒ 接触面 **1.825** = 1.80 + 0.025 ✓ |
| 竖直射线（2.0 m 平面上 11×6 = 66 条网格射线） | **30 条命中支撑**，命中高度只有 **1.92**（方柱）/ **1.83**（横撑）两档 ✓ |
| 塔下可行驶面 | 0.2 m 方块从 `(0,1.62,0.5)` 落下停在 **0.3198** ⇒ 接触面 **0.2198** = 中央高地顶面，**没有碰到支撑**（支撑最低 1.40）✓ |
| 出生候选 | `(5.90,2.72)` 底 **+0.0299** 不变（离塔 5.5 m）✓ |

### 9.8 换用组委会新 STL + 把塔上段整体下移（2027-10-29，取代 §9.7）

#### 9.8.1 为什么换
旧的 `RMUC2027.stl` 是从 653 MB STEP 三角化（926 万面）+ 简化到 30 万面来的，塔在 world z **1.50–1.85** 有
0.35 m 断口（§9.7）。用户随后给了一份**组委会的原始 STL**：同一块场地（底板下表面同样是 **435.3 m²** 的那个平面），
但塔的断口/高度不一样。用户明确"**不要多一层支撑，要它自己贴地/把高度降下来**" ⇒ 换新 STL + **把塔上段整体下移**，
不再加任何新几何（§9.7 的 `RMUC2027_tower_support` 已删除）。

#### 9.8.2 新 STL 与对齐（数字）
| 项 | 值 |
|---|---|
| 文件 | `rmuc_2027 (1).stl`（只读附件），**18,933,384 B** |
| 规模 | **378,666 三角 / 1,135,998 顶点**，表面积 **1,629.8 m²**，非水密 |
| 原始坐标系 | **米制、Z 朝上**，bbox `x[0.149,28.149] y[0.064,16.067] z[0.019,3.818]` |
| 对齐方法 | 0.1 m 顶层高度图与旧网格做**平移 + 4 种镜像**搜索（按 `mean|Δz|`） |
| 结果 | **identity（不镜像）+ 平移 (−14149.1, −6441.0, −1960.7) mm** ⇒ `mean|Δz| = 22.6 mm`、p90 = 24.8 mm；镜像/翻转明显更差（128 mm）；180° 旋转 22.8 mm（**这张场地近似中心对称**，用 identity） |
| 烘进 STL 的平移 | **(−14149.1, −6441.0, −260.7) mm**（x/y 用上值；z 再多抬 1700 mm，让**可行驶地面落在 z = 0**，从而 world pose 变成 (0,0,0)） |

对齐后的独立复核（竖直射线，mm）：可行驶地面到 **x = ±13.98**（±14.00 处无几何）⇒ 场地边 **±14000**；
y 台面到 **−5.87 / +9.12**，再往外是 **0.45 m 宽、−0.138…−0.151 m 的低台**（一直铺到 bbox 的 −6.377/+9.627）——
和旧网格（−0.145 低台、台面 ±14.00 / −5.85…+9.15）**同一套结构**，与围栏（x ±14.00、y −5.85/+9.15）差 **≤30 mm**。

#### 9.8.3 塔的下移（不是加支撑）
新网格里塔是**两组东西**：

| | 分量 | 面数 | z 范围 (world m) | 处理 |
|---|---|---|---|---|
| 下段 | 37 | 14,654 | 0.295 … 2.921（含**两根落地斜撑**） | **不动**（它本来就落地，站在 0.22 m 中央高地上） |
| 上段（灯架本体） | **787** | **117,976** | **1.859 … 3.558** | **整体下移 0.959 m** ⇒ 0.900 … 2.598 |

* 下移量怎么定：上段底 **1.859 m** 对齐到下段"台面顶面" **0.900 m**（实测：塔柱体 0.85–0.90 层有 356 面、
  0.90–0.95 层只有 1 面 ⇒ 台面在 0.90）⇒ **dz = −0.959 m**。
* 结果：塔柱体从 **0.20 m（塔脚）到 2.921 m（斜撑顶）连续**，50 mm 逐层只剩两处空层：
  **0.10–0.20 m**（塔脚与中央高地表面之间的空气）和 **2.95 m 以上**（斜撑顶之上）——**0.9–2.9 区间一个空层都没有**。
* 网格最高点：**3.558 → 2.921 m**（斜撑顶）；上段盒体顶 **2.598 m**。
* **没有加任何三角形**（378,666 面数不变），只是把上段 787 个分量的顶点整体平移 959 mm。

#### 9.8.4 world / 资产变更
* `meshes/RMUC2027_world/meshes/RMUC2027.stl`（`world/` 那侧仍是相对符号链接）换成新网格，**18,933,384 B**（旧 15,000,084）。
* `RMUC2027_world.world`：`visual`/`collision` 的 pose 由 `0 0 1.700` → **`0 0 0`**（地面已在 mesh 的 z=0）；
  `<scale>0.001` 不变（网仍是毫米）；**`<include>` 围栏保留**，塔支撑的 include 删除。
* `RMUC2027_tower_support`（world/ 与 meshes/ 两个目录）+ 它的 include **全部删除**。
* model.sdf（独立入口）/ model.config 同步（pose 0、描述改成新 STL）。

#### 9.8.5 实测
| 项 | 值 |
|---|---|
| world 加载 | **0.83 s**，0 条 mesh 解析错误 / 0 条 ODE 告警 |
| `gz model -p -m RMUC_2027` | `0 0 0 0 -0 0` |
| 场地 bbox（m） | `x[−14.000, +14.000]`、`y[−6.377, +9.626]`、`z[−0.241, +2.921]`（旧：z 到 +3.558） |
| 出生候选 `(5.90, 2.72)` 地面 | **+0.0304 m**（0.2 m 方块落底）⇒ 旧值 **+0.030**，差 **0.4 mm** ⇒ **`_HZMI_SPAWN_GROUND_Z` 仍是 0.030，不用改** |
| 其它地面点 | `(12,0)` +0.0167、`(13.9,0)` +0.0164、`(−5,3)` +0.023、`(0,−2)` +0.264（那一带是 0.26 m 台面）、`(0,1.62)` +0.179（塔下高地） |
| 围栏 | 仍在（`gz model -p -m RMUC2027_fence` = `0 0 0`），与新网格可行驶边差 ≤30 mm |
| 塔 | 柱体 0.20–2.921 连续（见 §9.8.3 的逐层表）；塔心 (0,1.62) 竖直射线命中 1.90/1.61/0.21/0.06（旧网格同点是 2.57→0.22 直接跳空） |

#### 9.8.6 诚实声明与影响
* **这份 STL 的塔本来就是"上段偏高 + 与下段脱开"的**（CAD 缺陷）。我们没有补几何，而是把**上段整体下移
  0.959 m** 让它落回下段台面 ⇒ **塔的绝对高度比 CAD 原始导出低 0.96 m**（3.56 → 2.60 m 盒体顶，
  网格最高点 2.92 m）。这是**人为补偿**，不是 CAD 原样。
* 场地的**其它几何一律没动**（只平移了塔上段的 787 个分量 + 整体刚体平移对齐）。
* ⚠️ `map/`、`PCD/`、`DB` 都是按**旧网格**建的：新网格与旧网格的平均几何差 **22.6 mm**、塔高度还变了
  ⇒ **需要重建地图/先验**（本轮按边界没碰，用户决定）。
* 没验证：GUI 外观（headless 无 X）、机器人/LiDAR 实跑、新网格的碰撞压力。


---


## 来源 4/7：`sim/round_summary_gz_frames.md`


> **状态**：四条线**全部完成**，本文没有"进行中"的条目。
>
> **写作口径**：**只汇总、不重算**——每个数字后面都挂着原文档的 `§` 或 commit。原文与本文
> 若有冲突，以原文的实测记录为准；已经发现的 **4 处口径冲突**逐条写在 **§4.0**，并说明取了哪一版、为什么。
>
> **证据分级**（全文逐条标注）：
> - **【实测】** 本轮真跑出来的（日志 / 探针 / bag 重放 / 命令行输出）；
> - **【推断】** 读源码、几何或文档推出来的，本轮**没有**数据；
> - **【未验证】** 没做，**不要当结论用**（汇总在 §4）。
>
> **覆盖提交**：
> `fe0dc31`（LIO 外参守卫）· `a137bf2`（机器卫生 runbook + 14 个脚本的收尾作用域）·
> `9152d90`（gz 文档收尾；gz 的**代码与主文档**实际随 `057294c` 一并入库）·
> `da5c2fe`（`model.config` 资产修正）。
>
> **主要来源**：`docs/tilted_lidar_fidelity.md` §O · `docs/machine_hygiene_runbook.md` ·
> `docs/fortress_sim_backend.md` · `docs/gazebo_sim_migration_plan.md` §11 ·
> `docs/cod_vs_ours_frame_architecture.md`。

---

## 0. 一句话结论

**这轮把四件事各自收口，且没有一件是靠"应该没问题"收的。**

| 线 | 一句话 | 现在能不能用 |
|---|---|---|
| **L1 外参守卫** | `small_point_lio` 的两个 latent 变换 bug（共轭 / 双重变换）从"文档里的设计"变成**开机可见 + 每帧可查**：外参有旋转会 WARN/ERROR，等式被破坏会限速 ERROR；**只诊断、不停机、不改任何数值**【实测】 | ✅ 能用；默认行为逐位不变 |
| **L2 机器卫生** | 把"swap 快满了"这个传闻**证伪**（真的没满），把真风险定位到 **oomd 整组杀**；并把 14 个脚本的收尾从"全机 pkill"改成"只杀自己这次起的"【实测】 | ✅ 能用；剩 `start_sentinel.sh` 一处 |
| **L3 Fortress 后端** | 多了一条 opt-in 的 `sim_backend:=gz`（**Classic 仍是默认、默认路径逐字节不变**），传感器/话题/帧/QoS 契约对齐【实测】 | ⚠️ **半可用**：世界/点云/`/scan` 能跑，**LIO 在 gz 侧不出 odom** |
| **L4 COD 帧对账** | 更正了一个**前提性错误**（COD 用的就是我们那份 `small_point_lio`），判决为 **(b) 约定分叉 + 6 条可避免耦合（其中 4 条明确该修）**，给出 C/B/A 三选项【实测源码 + 推断】 | ✅ 结论可用；选项 C 未实施（等拍板） |

**现在能不能用**（按用途分）：

| 用途 | 能不能 | 依据 |
|---|---|---|
| Classic 默认路径（跑仿真、LIO、nav2、建图） | ✅ **能**，且与改动前**逐字节相同** | launch 增量 `+2/−0`（`fortress_sim_backend.md` §9）；LIO 默认档重放 **0 差异**（`tilted_lidar_fidelity.md` §O.4a） |
| gz 后端做**传感器/感知/点云**验收 | ✅ 能（10.0000 Hz、RTF 0.97、`/scan` 通、自机排除生效） | `fortress_sim_backend.md` §5/§6/§8 |
| gz 后端做**定位/建图闭环** | ❌ **不能**：`small_point_lio` 持续 `imu loop back`、不发 `odom→base_link` | `fortress_sim_backend.md` §8.1 |
| gz 后端跑**移动**闭环（nav2 控制） | ❌ **不能**：底盘在 SDF 里是 `<static>`，`/cmd_vel_chassis` 无人消费 | `fortress_sim_backend.md` §10 第 1 条 |
| 真机 | — 本轮**没碰**（无 MID-360、无实车） | `tilted_lidar_fidelity.md` §O.5 第 1 条 |

**还差什么**（全部展开在 §4）：gz 侧 LIO（首要）· gz 底盘运动 · 同轮 Classic A/B · 真机 `R_bl` 来源 ·
COD 的 `urdf/` 本地副本 · `robot11_mount:=urdf` 去留（等拍板）。

---

## 1. 四条线 + 状态表

| 线 | 做了什么 | commit | 关键实测数字 | 剩余缺口 | 状态 |
|---|---|---|---|---|---|
| **L1 LIO 前提断言** | 新增独立 TU `extrinsic_guard.cpp`：启动期外参 readout（#1）+ 每帧 `T_ob·T_bl⁻¹≈T_ol` 断言（#2）；单测第 6 条 | `fe0dc31` | 成本 **24.4 ns/帧**（+~5 ns 拷贝）；默认档重放 **30/30、31/31、31/31 全 0 差异**（第二袋 33/34/34）；斜装档启动 ERROR `rpy=[30.000000,-0.000000,0.000000]°`、`\|t\|=0.204443 m`；注入共轭 ⇒ `0.207297637 m / 30.000000°`、计数 **1→15→29**（间隔 5.0–5.2 s）；反向对照 **0 违例**；假俯仰 **0.082°**（修前 4.890°）；最大平面角 **0.046/0.367/0.420°**（bug② 预测 29.977°） | `1.05°` 绝对角未复现（世界/出生点已变，§4 U7） | ✅ 完成 |
| **L2 机器卫生** | swap/zram/oomd 体检 runbook（逐条标 已执行/需 root/未执行）+ 新增 `scoped_cleanup.sh` + **14 个脚本**收尾作用域化 | `a137bf2` | swap **194 / 3901 MiB**（测试前 `pswpout` 仅 **652 页 ≈2.5 MB**）；`vm.swappiness=60`；zram **不可用**；oomd `kill`+`Limit=50%`（曾杀 **25 个进程**）；历史全局 OOM `async_slam_tool` **11.85 GB**；decoy **10 个** PASS + **3 真脚本 ×5 decoy** PASS；`pkill -9 -x gz` **18 行 → 0** | `start_sentinel.sh` 仍是全机 `pkill -f ros` 等 4 行 | ✅ 完成 |
| **L3 Fortress 生产化** | `fetch_and_patch_mid360.sh`（pin 上游、幂等、不 vendor）+ 新包 `rm_livox_gz_sim` + `sim_backend:=classic\|gz`（默认 classic） | `9152d90`（内容随 `057294c`） | 仿真 **10.0000 Hz** / 墙钟 **9.72 Hz** / RTF **0.97**；中位 **8018** 点；`offset_time` 最大 **99,996,667 ns** 单调；求交 **0.325 ms/实例 ⇒ ≈0.65 ms/帧**；双实例 BVH **建一次**、**313,721 点 atol=0**；RSS **328/441/652 MB**；首帧 **3.00 s**；`/scan` **418 束 2.75–8.09 m** `frame_id=base_link`；`--strip-conditions` **+2/−0** | gz 侧 LIO 未通过；底盘 `<static>`；无同轮 A/B | ✅ 完成（opt-in，功能有边界） |
| **L4 COD 帧架构对账** | 只读源码取证，出对账文档：H1/H2/H3 + 可避免耦合 O1–O6 + 三选项 | `057294c` | COD `obstacle_layer` 命中 **0/0**、`/scan` 命中 **0**；`small_point_lio` 与上游 `688d75c` **sha256 逐字节相同**；我们与 COD 的 vendored launch **唯一差异** = `:47` 多一个节点；`lio_tf_adapter` 被 **3 处** `!= 'small_point_lio'` 排除 | COD 本地无 `urdf/` ⇒ `T_bl≈I` 是推断 | ✅ 完成 |

### 1.1 L1：守卫具体落了什么（`fe0dc31`）

**文件与插入点**（全部有 `file:line`，见 `tilted_lidar_fidelity.md` §O.1）：

| 项 | 位置 | 内容 |
|---|---|---|
| 阈值 | `src/localization/rm_localization/small_point_lio/src/extrinsic_guard.cpp:36-52` | WARN `1e-3 rad`（≈0.057°）、ERROR `0.05 rad`（≈2.9°）、旋转容差 `1e-6 rad`、平移容差 `1e-4 m` |
| #1 启动 readout | 同上 `:66-109` | `rpy(T_bl)`（度）+ `\|t\|`，**每次启动恰好 1 条**，级别按 `max\|rpy\|` 取 INFO/WARN/ERROR |
| #2 每帧断言 | 同上 `:131-176`（违例分支起 `:157`，计数 `:158`） | `T_ob·T_bl⁻¹` vs `T_ol`；**只计数 + 限速 ERROR（1 条/5 s，墙钟 `steady_clock`）** |
| #2 取数点 | `small_point_lio_node.cpp:193-194` | 14 个 double 的 POD 快照（odom 回调末，**不引入任何新的浮点表达式**） |
| #2 检查点 | `small_point_lio_node.cpp:208-209` | 点云回调首、`get_subscription_count()` 分支**之外** ⇒ 有没有订阅者都查 |
| 构建 | `src/localization/rm_localization/small_point_lio/CMakeLists.txt:74-78` | 显式 `list(APPEND SOURCE src/extrinsic_guard.cpp)`（GLOB 在 configure 时求值，不显式列会在**已有 build/** 的工作区里 undefined symbol） |
| 单测第 6 条 | `tools/scripts/tiltmount/tf2_compose_order_test.cpp:155-201` | ⑥a/⑥b/⑥c |

**为什么必须独立 TU（本轮最反直觉的一处，实测逼出来的）**【实测】：

| 守卫放法（同一输入流 bag 重放，30 条 `/cloud_registered` / 31 条 `/odom` / 31 条 TF） | `/cloud_registered` | `/odom` + `odom→base_link` |
|---|---|---|
| 内联在 odom 回调里 | 30/30 逐位相同 | **31/31 有差，最大 1.11e-16（1 ulp）** |
| 同 TU、但在点云回调里调用（odom 回调只留 14 个 double 拷贝） | 逐位相同 | **31/31 有差，最大 1.11e-16** |
| **独立 TU `extrinsic_guard.cpp` + 同一取数点** | **逐位相同** | **逐位相同（0 差异）** |
| 语义中性对照：只加一行整数自增 | 逐位相同 | **0 差异** |
| 语义中性对照：只加 14 个 double 成员拷贝 | 逐位相同 | **0 差异** |

读法：不是"任何改动都会差"，而是**这段浮点代码在 odom 回调所在 TU 里就会让 GCC 的 IPA/内联预算变**
（本包 `-ffast-math -march=native`，`tf2::Transform::operator*` 的 FMA 收缩随之变）⇒ 1 ulp。
反过来，"守卫只诊断、不动数值"是**可逐位验证**的，不是承诺。

**平移为什么用"双合法值白名单"而不是严格断言**：本仓按 §J.5 **有意**保留旧平移 ⇒ 等式在平移上
**永远**不成立（与合成真值差 `|t_pub−t_composed| = 0.204267 m`）。所以判据是
`min(|t_pub−t_composed|, |t_pub−t_legacy|) <= 1e-4 m`：两条合法值都由本仓代码显式算出，
**谁写了第三套平移公式**才会触发。

**成本**：独立微基准 2e6 次、非恒定输入、`-O3 -march=native -ffast-math` ⇒ **24.4 ns/次**（两次 24.4/24.7）；
加 odom 回调里 14 个 double 的拷贝（~5 ns），量级 **<0.1 µs/帧**，相对点云回调（几十 µs 级）可忽略。
**无新 TF 查询、无新订阅/定时器/线程**（§N.5 明确反对"在点云回调里再查一次 TF"——那正是 bug ② 的形状）。

**三段回归**【实测】：

| 段 | 判据 | 实测 |
|---|---|---|
| (a) 默认档 `plugin`（`R_bl=I`） | 同一输入流重放，改动前 vs 改动后 | 袋1：`/cloud_registered` 30/30、`/odom` 31/31、`/tf` 31/31 **全 0 差异**；袋2（最终二进制）：33/34/34 **全 0 差异**；差异只有多出来的那条 INFO 守卫日志 |
| (b) 斜装档 `robot11_mount:=urdf`（`T_bl=Rx(30°)`） | 守卫**真的会喊** | 启动 ERROR `rpy=[30.000000, -0.000000, 0.000000]° (=0.523599 rad)、\|t\|=0.204443 m`；故障注入（把合成改回共轭）⇒ 3 条限速 ERROR，平移 `0.207297637 m`、旋转 `30.000000°`，计数 **1→15→29**，间隔 5.0–5.2 s；**反向对照**（正确代码 + 同一 30° 外参）⇒ **0 条违例** ⇒ 守卫不是"见 30° 就喊"，是"等式被破坏才喊" |
| (c) 两个老 bug 仍修好 | 同一次 `gq_urdf` live 跑的 `probe.json` | 假俯仰 `-0.082°`（**<0.5°**，修前 4.890°）；`/cloud_registered` 最大平面角 `0.046/0.367/0.420°`，而 bug ② 假设应为 **29.977°**；Kabsch(raw→published) 旋转 **0.087°**、平移 `[-0.0069, 0.0031, 0.0504] m` |

**⚠️ 一处诚实偏离**：§J 那轮的"地面倾角 ≈1.054°"本轮**没有复现**。原因已查明：本轮最大平面到传感器
的高度 `d = 0.103 m`，而 §J 那轮是 `d = 0.26 m`（真地面）；本轮出生朝向真值 yaw **0.01°**、§J 是 **10.12°**
⇒ **两次跑的"最大平面"不是同一块面、出生姿态也不同**（世界/出生点在 10-09 之后被改过；本轮未动
`src/simulation/rm_simulation/**`）。所以本轮**不主张"绝对角 = 1.05°"**，只主张**判决性签名**：bug ② 的 30° 签名不存在。

### 1.2 L2：机器卫生与收尾作用域（`a137bf2`）

**结论反转（第一件事先把传闻证伪）**【实测】：

| 之前的说法 | 本轮实测 | 判决 |
|---|---|---|
| "`SwapFree ≈ 1 MiB`，swap 快满了" | swap **4095 MiB 总 / 194 MiB 已用 / 3901 MiB 空闲**；`MemAvailable` **16.8 GiB** | **没耗尽** |
| 同上 | 本次开机（10-16 16:06 起）在测试**之前** `pswpout` 只有 **652 页 ≈ 2.5 MB**；若这次开机真被换满 4 GB，`pswpout` 会是 **~100 万页** 量级 | ⇒ 那个快照属于 **上一次开机**（换页统计随重启清零），**已被取代** |

**swap 里装的是谁**（cgroup `memory.swap.current` 口径，合计 ≈194 MB，与 `free` 吻合）：
firefox **102.35 MB** · Chrome **34.98 MB** · gnome-software **21.27 MB** · packagekit **9.65 MB** ·
VS Code 1.99 MB …；顶层 `user.slice` **181.5 MB**、`system.slice` **9.9 MB** ⇒ **是桌面应用，不是 gazebo**。

**系统面**：`vm.swappiness = 60`（默认）；zram **不可用**（`/dev/zram*` 不存在、模块未加载、无 `zram-generator`；
工具与模块都在）；`sudo` 在沙箱里被 `no-new-privs` 挡死 ⇒ **所有需 root 的命令一条都没执行**。

**真风险不是"swap 满"，是"oomd 整组杀"**【实测 + 历史日志】：

| 日期 | 谁杀的 | 判据 | 被杀单位 |
|---|---|---|---|
| 10-01 21:44 | **systemd-oomd（用户态）** | user slice PSI **72.74% > 50%** 持续 **20 s** 且有 reclaim | 整个 `vte-spawn-*.scope`（**25 个进程**，含终端里所有 job） |
| 10-06 11:30 | **内核 OOM killer**（`global_oom`，调用者是 oomd） | 全机真的分配不出页（`async_slam_tool` 单进程 anon-rss **11.85 GB**） | 单进程 `async_slam_tool` |

"50%" 的来源是发行版 drop-in `/usr/lib/systemd/system/user@.service.d/10-oomd-user-service-defaults.conf`
（`ManagedOOMMemoryPressure=kill` + `ManagedOOMMemoryPressureLimit=50%`，`systemctl show` 里显示成
`2147483648`），**不是** `/etc/systemd/oomd.conf` 的 60%（那份只有注释）。

**脚本收尾的作用域改造**【实测】：

* 新增 `tools/scripts/lib/scoped_cleanup.sh`（205 行）。三条判据，**任一**成立才算"自己的"：
  ① `setsid` 进程组（`PGID=$!`，`kill -INT -- "-$PGID"`）；
  ② `/proc/<pid>/environ` 里 `GAZEBO_MASTER_URI` 与本 run **逐字相等**；
  ③ PID 是 `$$` / `$LPID` / `$LAUNCH_PID` 的**后代**。
  名字/命令行**只用来挑候选**，绝不单独作为杀的理由；自带自伤保护（`$$` 及所有祖先永不杀）。
* **14 个脚本**接进去；`run_robot_model_probe.sh` / `robot11_gui_bench.sh` 原来没有 `setsid`，这次补上。
* 删掉一类"看着像保护、其实永远匹配不到"的写法：`pkill -f "gzserver.*$PORT"`（gzserver 的 cmdline 里
  **没有端口**，端口在 `GAZEBO_MASTER_URI` 环境变量里）⇒ 那是**空操作**。
* 证据：10 个合成 decoy 的判定矩阵 **PASS**（别人的 gzserver/gzclient/launch/slam 全活、本 URI 的全死、
  进程组兜底生效、自伤保护生效）；3 个**真脚本** × 5 decoy **PASS**（`run_nav_smoke_regress`、
  `run_remap_lowterrain`、`run_robot_model_probe` 连跑 3 次）；跑前残留清理也只清本 URI 的。
* 最终 `grep -rn 'pkill -9 -x gz' tools/` = **0**（改前 **18 行**）。

**剩余风险（明确留着）**：`tools/scripts/control/start_sentinel.sh:165/166/169/172` 仍是
`pkill -f gzserver` / `gzclient` / **`pkill -f ros`** / `rviz2` —— 其中 `pkill -f ros` 会杀掉机器上
**所有** ROS 进程（包括别人的）。

### 1.3 L3：Fortress 生产化（`9152d90`，内容随 `057294c`）

**交付物**：

* `tools/scripts/gzsim/fetch_and_patch_mid360.sh`：取三个上游并打补丁，pin 死在
  **rmagine `2.4.3`** / **rmagine_gazebo_plugins `c6dc04e`** / **mid360_gazebo_harmonic `497149a`**，
  幂等，编到独立前缀（默认 `$REPO/.tmp_gzlivox2/gzsim_ws`，`GZSIM_PREFIX` 可覆盖），
  **不 vendor 第三方代码**、不装进 `/opt/ros`、不碰系统库。
* 新包 `src/simulation/rm_simulation/rm_livox_gz_sim/`（launch / worlds / models / scripts / config）。
* `bringup_sim.launch.py` 的 `sim_backend:=classic|gz`（**默认 `classic`**）。

**默认路径零改动的证据**【实测】（`fortress_sim_backend.md` §9，工具 `launch_increment_diff.py` 比较**整棵
Action 树**的每个字段 + condition + 嵌套子 Action）：

| 口径 | 结果 |
|---|---|
| Action 数量 | 旧 **61** → 新 **63** |
| 变化点 | **3 处**：① 新增 `DeclareLaunchArgument name=sim_backend`；② 新增 `IncludeLaunchDescription`（gz，`condition=…=='gz'`）；③ 原 `start_rm_simulation` **内容一字未改**，只多了 `condition=…=='classic'`（默认恒真） |
| `--strip-conditions` 后 | **+2 新增 / 0 删除 / 顺序一致** ⇒ **纯增量** |
| `ros2 launch … --show-args` | 默认值全不变，只多一个 `'sim_backend': … (default: 'classic')` 与 `Valid choices are: ['classic', 'gz']` |

**三个缺口**（这是 L3 全部"不能"的来源）：

1. **gz 侧 LIO 未通过**【实测】：`small_point_lio` 持续打印 `imu loop back`（`preprocess.cpp:67` 的
   `imu_msg.timestamp < last_timestamp_imu`），**没有发出 `odom→base_link`**，`view_frames` 里
   **没有 `odom`/`map` 帧**，`/cloud_registered` 空。**发布端已被排除**：rclpy 直收 `/livox/imu`
   是严格 `+10 ms` 单调（381 条，`min=max=10,000,000 ns`，0 条负差），且 `Publisher count: 1`。
2. **底盘 `<static>`**【实测】：Fortress 没有 `planar_move` 的对应物 ⇒ `/cmd_vel_chassis` 不驱动底盘，
   nav2 规划/控制链路仍会跑但车不动。
3. **无同轮 Classic A/B**【未验证】：Classic 列填的是既有文档基线（见 §3 与 §4.0 冲突 1）。

### 1.4 L4：COD 帧架构对账（`057294c`）

**必须显式写下的更正**：**COD 用的就是我们 vendored 的同一份 `small_point_lio`**（同一上游
`Yancey2023/small_point_lio@688d75c`，`sha256` 逐字节相同，`tilted_lidar_fidelity.md` §N.1 已三条独立取证）。
"COD 换了 LIO 所以不受影响"**是错的**。COD 不炸的真原因是**换了 `base_link` 的定义**。

| 事实 | 证据 | 性质 |
|---|---|---|
| COD 的 `base_link ≡ livox_frame`（传感器壳体系，原点在雷达、z 随雷达斜 30°） | `small_point_lio_node.cpp:75` 的共轭只在 `T_bl=I` 时成立；`small_point_lio.launch.py:24-45` 声明单位 extrinsic 而 `:47` **只 return 节点**（死代码）；`cod_bringup` 全仓 3 处 `static_transform_publisher` **全是 `map→odom`**；`filter_node.cpp:56`（marker 发 `base_link`）vs `:81-119`/`:117`（CropBox 作用在输入点云系、`header` 原样保留）；`multiplenav_launch.py:70`（`target_frame: base_link` 而输入是 `/livox/lidar`） | 四条独立证据 ⇒ **推断（强旁证）**，因为本地 checkout **没有 `urdf/`** |
| COD 节点里是**共轭**（相似变换）`T_bl⁻¹·T_ol·T_bl` | `:75` | 【实测源码】 |
| COD 对"已配准的点"**又乘了一次外参** | `:159`（`transformed_point = R*point + T`），而 `small_point_lio.cpp:98` 已经是 `odom` 系 | 【实测源码】 |
| 两者在 `T_bl=I` 时**同时退化成恒等** ⇒ 在 COD 里**不可观测** | 上面的旋转/平移分量分析 | 【推断】 |
| ⚠️ **我们与 COD 的唯一差异**：我们的 vendored launch `:47` **把那条单位 TF 加进了返回值** | `src/localization/rm_localization/small_point_lio/launch/small_point_lio.launch.py:47`，blame **`98baa55`** | 【实测源码】⇒ **潜伏陷阱**（当前 bringup 不走这个 launch，所以还没咬人） |
| COD **完全没有 `obstacle_layer`** | 两份 params `grep -c obstacle_layer` = **0 / 0**；`/scan` 命中 **0**；两张图都是 `["static_layer","stvl_voxel_layer","inflation_layer"]`，源 `/livox/lidar_filtered`，`min/max_obstacle_height 0.15/1.0` | 【实测源码】 |
| COD 的 `/scan` 只喂 `slam_toolbox`（多模式），`map→odom` 是**静态** TF（没有定位节点） | `multiplenav_launch.py:65-94`；`singlenav_launch.py:67-89`；`localization_launch.py` 的 `lifecycle_nodes=['map_server']` | 【实测源码】⇒ nav2 的"每束 z 由扫描原点伪造"这条代码路径**从未被执行** |
| 本仓 `lio_tf_adapter` 是**唯一写对合成**的实现，却在 `lio:=small_point_lio` 时被**排除** | `lio_tf_adapter_node.cpp:66-72`（`q_lio*q_off`、`p+R·t`）；`bringup_sim.launch.py:2028-2033`、`:2051-2055`、`:2076-2080` 三处 `!= 'small_point_lio'`；`:2025` 注释自认"该换算的写法有瑕疵" | 【实测源码】 |

**判决**：**(b) 约定分叉 + 可避免耦合**，不是 (c) 真缺陷。可避免的 **6 条**（O1–O6）vs 固有的 **6 条**（I1–I6）；
其中 **4 条明确该修**（O1 高度阈值住在 4 个 frame 里 / O2 adapter 被排除 / O3 三档并存 + `urdf` 账是错的 /
O4 我们启用了上游那条单位 TF），**2 条半修**（O5 `local_obstacle:=scan` 默认值 / O6 roll-pitch 双来源）。

**三选项排序**：

| 排 | 选项 | 内容 | 成本 | 风险 | 建议 |
|---|---|---|---|---|---|
| 1 | **C** | `robot11_mount` 三档收敛成 `plugin`（默认）+ `sensor`（保真），`urdf` 从 `choices` 移除只留文档反例；删 vendored 的单位 TF；修 `cod_nav_comparison.md:27/:43` | **S（≤1 人日）** | 低 | **先做**（等拍板，见 §8） |
| 2 | **B** | `lio_tf_adapter` 成为**所有** LIO 槽的 `odom→base_link` 唯一权威（给 `small_point_lio` 加 `publish_tf:=false`）+ 高度语义单一所有者 | **M（2–3 人日）** | 中 | 主体 |
| 3 | A | 全面采用 COD 约定（`base_link` 搬到雷达） | L（≥1 周 + 全部基线重跑） | 高 | **不做** |

---

## 2. ⭐ 与之前的区别（重点）

> 这张表 = `fortress_sim_backend.md` §2 的 11 条（去重后 14 行）**+** 本轮其它三条线带来的 3 类变化。
> "之前" 一律指 **Classic 11 / 本轮之前**；"现在" 一律指 **gz 后端 / 本轮之后**。

### 2.1 总表

| 组 | # | 维度 | 之前（Classic / 本轮之前） | 现在（gz / 本轮之后） | 为什么重要 / 影响 |
|---|---|---|---|---|---|
| gz | 1 | **传感器实现** | URDF `<sensor type="ray">` + `libros2_livox.so` | 模型级 `<plugin filename="libmid360_system.so">`（gz **system** 插件） | 传感器从"URDF 里的一条 ray 传感器"变成"挂在 model 上的系统插件"；SDF 里 `livox_frame` 是一个真 `<link>` |
| gz | 2 | **求交后端** | **ODE 逐射线**打 `<collision>` | **Embree BVH**（rmagine `O1DnSimulatorEmbree`，建一次、逐帧复用）；Fortress 无 `components/RaycastData` ⇒ physics 后端编译期关闭 | 从"ODE 逐射线"变成"CPU 批量 BVH"⇒ **求交第一次可以计时**（0.325 ms/实例，§3）；也是 RSS 降一个量级的来源 |
| gz | 3 | **点云是否过桥** | **不过桥**：`libros2_livox.so` 在 gzserver 进程内直接发 ROS 话题 | **也不**过桥：插件在 gz 进程内用 rclcpp 直发；`ros_gz_bridge` **只桥 `/clock`** | ⇒ 没有 `PointCloudPacked` 中转、**`frame_id` 不会变成 gz 的 `model::link::sensor` 名字**（迁移计划 §6.7 点名的最高风险项**没有发生**）、少一次序列化 |
| gz | 4 | **字段布局** | `x,y,z`（f32）+ **`rgb`**（f32），**`point_step=32`** | `x,y,z`(f32, 偏移 0/4/8) + `intensity`(f32, 12) + `tag`(u8, 16) + `line`(u8, 17) + `timestamp`(f64, 18)，**`point_step=26`**，**无 `rgb`**，`is_dense=true` | 本仓**没有**消费者硬依赖 rgb：grep 只 2 处命中（Classic 插件生产端 + `point_lio` 一个**没人用**的 typedef）；linefit/p2l/patchwork/gicp/探针全按**字段名**取 xyz ⇒ **节点不坏**。唯一实际影响是**外部工具**：RViz `Color Transformer=RGB8` 会全黑（改 `Intensity`） |
| gz | 5 | **`frame_id`** | `/livox/lidar*`=`livox_frame`；`/livox/imu`=`imu_link` | **同**（插件 `<cloud_frame>` / `<imu_frame_id>` 钉住） | 不钉 `imu_frame_id` 会退回 `livox_frame`，与 Classic 不一致 |
| gz | 6 | **`offset_time`（逐点时间）** | **恒 0**（帧内无运动模型） | `i·1e9/point_rate`，**单调**、最大 **99,996,667 ns = 99.9967 ms**（`i=29999`） | 去畸变第一次有真的时间维；也是"两实例同 stamp 同点数"的前提 |
| gz | 7 | **QoS** | 点云 `SensorDataQoS`（best_effort + keep_last 5）；IMU reliable | **同**（上游原值 `KeepLast(20).reliable()` 被补丁 **0003** 改掉）；IMU **保持 reliable** | 详见 **§2.2**：IMU 必须 reliable；全仓 9 个点云订阅者**全是** best_effort ⇒ 换写者不会"发了个寂寞" |
| gz | 8 | **双发布** | **一个插件一次** publish 两个消息 | **两个插件实例**（同 link、同姿态），各发一路；IMU 只由 #1 发（`<publish_imu>`） | 两实例**共用一棵 BVH**（`1 supported visuals` 只 1 次）、**313,721 点 atol=0 逐点相同**；代价只有**求交 2×**（≈0.65 ms/帧），无内存/一致性代价 |
| gz | 9 | **30° 记在哪** | `plugin` 档：插件键 **`<tilt_rpy>`**（帧是平的、数据是斜的） | SDF **`<link><pose>`** 的 roll **`-0.523598775598293`**，**且要求 URDF 关节也斜**（`robot11_mount:=sensor`）⇒ 帧与数据自洽 | gz 后端**只实现 sensor 语义**："平帧 + 斜数据"是自相矛盾的组合，没有移植价值；代价是 `plugin` 档在 gz 侧**不存在**（§2.9 第 2 条） |
| gz | 10 | **自机排除机制** | `livox_frame` **故意不给 collision** ⇒ "不打自己"靠**几何缺失** | 插件把所属 model 登记进 `MapRegistry` ⇒ 整车 **113 个碰撞体**不进 BVH；`<ignore_self>true\|false</ignore_self>` 可 A/B | A/B 判决：`true` ⇒ 中位 **8040.5** 点、`r_min 0.4292 m`、`r<0.30 m` 的点 **0**；`false` ⇒ **0 点（全盲）**，因为传感器被自己的碰撞体**整个包住** ⇒ 这条是**必需**，不是优化 |
| gz | 11 | **世界资产要求** | Classic 容忍空 `<sdf>`、SDF 版本宽松 | SDF **≤ 1.9**（`sdformat12` 上限；写 1.10 ⇒ `Unable to convert from SDF version 1.10 to 1.9`，**一帧点云都不出**）；`model.config` 的 `<sdf>` **不能为空**（否则 `Failure to detect an sdf tag`） | 两条都是"静默失败/整世界起不来"级；本仓两份 `world/model.config` 已在 **`da5c2fe`** 修好（2 行）；`ign sdf -k` 可预检 |
| gz | 12 | **`HOME` 与插件路径** | 无要求 | gz 要**可写** `$HOME`（`~/.ignition`）；本机 `$HOME` 只读 ⇒ **GUI abort 并 SIGKILL server**；`GZ_SIM_SYSTEM_PLUGIN_PATH` **和** `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` **都**要设 | launch 自动把 gz 子进程的 `HOME`/`XDG_*` 重定向到运行期临时目录（`gz_home`）；裸 shell 里手起 `ign gazebo` 要自己重定向 |
| gz | 13 | **启动方式 / 参数** | `bringup_sim.launch.py …` 一条路 | 同名 launch 多一个 **`sim_backend:=classic\|gz`**，**默认 `classic`**；另有 `gz_world` / `gz_qos` / `gz_ignore_self` / `gz_home` / `gz_csv` / `gz_runtime_dir` | 3 处变化、净 **+2** 个 Action（61→63），`--strip-conditions` = **+2/−0**；不传该参数时节点集合/命令行/时序**逐字节不变** |
| gz | 14 | **底盘 / 真值里程计** | `libgazebo_ros_planar_move.so` 订阅 `/cmd_vel_chassis`，并发 `/odom_ground_truth` | **都没有**：机器人在世界文件里是 **`<static>true</static>`**；`/odom_ground_truth` **不存在** | ⇒ gz 只能做"静止的感知/定位/建图"验收，**不能**跑 nav2 移动闭环；依赖真值里程计的调试脚本在 gz 档拿不到数据（契约上不要求，`odom→base_link` 归 LIO） |
| 其它 | 15 | **新增断言**（L1） | 两个 latent bug 只能靠**离线文档**看 | 启动 readout（1 条/次）+ 每帧 `T_ob·T_bl⁻¹≈T_ol` 断言（**只计数 + 限速 ERROR 1/5 s**） | 成本 **24.4 ns/帧**；默认档**逐位 0 差异**；斜装档会喊；**不抛、不中止、不改发布、无 latched 话题** ⇒ 不改任何契约 |
| 其它 | 16 | **收尾作用域**（L2） | 全机 `pkill -9 -x gzserver/gzclient` + `pkill -9 -f 'ros2 launch'` + 空操作 `pkill -f "gzserver.*$PORT"` | `scoped_cleanup.sh`：**进程组 / `/proc/environ` 的 `GAZEBO_MASTER_URI` / 血缘**，**14 个脚本**接入 | 名字只用来挑候选、绝不单独作为杀的理由；同机有兄弟任务时不再误杀；`pkill -9 -x gz` **18 行 → 0** |
| 其它 | 17 | **资产修正**（`da5c2fe`） | 两份 `world/model.config` 是 `<sdf version="1.7"></sdf>`（**标签内容为空**） | `<sdf version="1.7">model.sdf</sdf>`（**2 个文件、2 行**） | Classic 侥幸容忍；sdformat12 不容忍 ⇒ 没有这 2 行，gz 世界**根本进不去** |

### 2.2 QoS：本轮唯一一条"结论与直觉相反"的

| 话题 | Classic | gz（默认） | 为什么 |
|---|---|---|---|
| `/livox/lidar`、`/livox/lidar/pointcloud` | `SensorDataQoS`（**best_effort** + keep_last 5） | **同**（`<qos_reliability>best_effort</qos_reliability>`） | 与 Classic **逐字对齐** |
| `/livox/imu` | 默认 **reliable** | **reliable 保留** | **FAST-LIO 用默认 depth-10（reliable）订阅 IMU**；best-effort 写者 + reliable 读者 **QoS 不兼容 ⇒ 一条都收不到**。IMU ~250 B @100 Hz，且发布发生在 gz transport 回调线程（不在 `PostUpdate`）⇒ 不构成背压源 |

**"全仓订阅者都是 best_effort"是这次敢改的前提**【实测 grep，`fortress_sim_backend.md` §4.2】：
`FAST_LIO`、`small_point_lio`、`point_lio`（`/livox/lidar`）、`linefit_ground_segmentation`、
`pointcloud_to_laserscan`、`patchwork_ground_segmentation`、`gicp_registration`、
`cloud_accumulator`、`tools/scripts/**` 探针 —— **9 个订阅者全是 `SensorDataQoS`/`BEST_EFFORT`**。
⇒ 换 best-effort 写者**不会**造成"收不到"。`reliable` 仍可一键复现（`gz_qos:=reliable`），**只用于对照实验**。

**诚实结论（这一条被任务书要求标出来）**：慢消费者实验里 **best_effort 9.711 Hz vs reliable 9.689 Hz**，
把订阅者 `SIGSTOP` 后 **best_effort 9.740 Hz vs reliable 9.660 Hz** —— **reliable 在本机也没有被堵**。
⇒ 这条改动的收益是**契约对齐 + 按定义永不阻塞的保证**，**不是本机测出来的性能提升**；
Classic 那次"写者阻塞"的机理牵涉"写者与读者在同一进程/同一执行器"，gz 插件是"PostUpdate 里 publish、
DDS 线程发"，**结构上就不同**。⚠️ Classic 那次"停更且不自恢复"的完整链条，本轮**没有**在 gz 侧复现，
也**没有**在 Classic 侧复跑对照（§4 U6）。

### 2.3 这 17 条可以分成三类（读法）

| 类别 | 条目 | 含义 |
|---|---|---|
| **物理/上游必然**（改不了，只能适应） | 1、2、4、6、11 | 传感器换了引擎、字段由插件决定、sdformat12 的版本上限 …… 这些不是"我们选错了" |
| **我们的选择**（有理由，但要能回退） | 3、5、8、9、12、13、14 | 不过桥、钉 `frame_id`、双实例、SDF `<pose>` 承担 30°、HOME 重定向、`sim_backend` 默认 classic、底盘静态 |
| **修 bug / 补护栏** | 7、10、15、16、17 | QoS 补丁 0003、实体级自机排除（Classic 靠几何缺失，不够用）、外参守卫、收尾作用域、`model.config` |

### 2.4 刻意**没有**变的东西（别误读）

| 契约 | 状态 |
|---|---|
| 话题名与类型（`/livox/lidar` CustomMsg、`/livox/lidar/pointcloud` PC2、`/livox/imu` Imu、`/scan`、`/cloud_registered`、`/Odometry`） | **一致**（下游节点一行未改） |
| TF 链（`map→odom→base_link→livox_frame/imu_link`）与单发布者约定 | **一致**（gz 侧不发任何 odom TF） |
| 谁发 `odom→base_link`（LIO）/ 谁发 `map→odom`（按 `mode`/`localization`） | **一致** |
| LIO 配置（`extrinsic_R=I`、`frame_id` 名字） | gz 档**不需要改** |
| Classic 的 world / xacro / config / 感知 / nav2 文件 | **一字未动**（`fortress_sim_backend.md` §2.11） |

---

## 3. 实测数字总表

> **口径警告（必读）**：**Classic 列是"既有文档里的历史基线"，不是本轮同机同时刻新测的** ⇒
> 没有做同轮 A/B，"变化"列**只能看量级**。Classic 的数字来源是 `livox_sensor_prior_art.md`
> §1/§10.3 与 `tilted_lidar_fidelity.md` §J（详见 §4.0 冲突 1）。

### 3.1 gz vs Classic 并排

| 指标 | **Classic 11**（⚠️ 历史基线，非同轮） | **gz / Fortress**（本轮实测） | 变化 / 口径 |
|---|---|---|---|
| RTF（`/clock` 仿真时间 / 墙钟） | **0.32–0.46**（`robot11` 全栈，迁移计划 §1/§9） | **0.97**（台架）/ **0.97**（`mode:=mapping` 整栈） | **≈ +2.4×**（按 0.40 中值算） |
| server RSS | **3.2 GiB**（prior_art §10.3 对照行） | **328 MB**（台架，整棵进程树）/ **441 MB**（含 robot11 + TF 发布者）/ **652 MB**（mapping 整栈） | **−80% … −90%** |
| 逐帧求交耗时 | 无（ODE 逐射线，未插桩） | **0.325 ms / 30000 rays**（单实例中位；min 0.307 / max 0.358，28 次采样）；双实例合计 **≈0.65 ms/帧** | 绝对量 <1 ms |
| 每帧点数（中位） | ≈ **11,568**（`tilted_lidar_fidelity.md` §J 的 `plugin` 档） | **8018.5**（台架，7854–8167）/ **8040.5**（robot11，sensor 档） | 差异来自**命中率与 `max_range`**，不是丢点（30000 束 26.7–27.2% 命中） |
| 启动 → 首帧点云 | 未记录 | **3.00 s**（launch → 日志 `Loaded 800000 scan directions`） | — |
| 仿真频率 | 10 Hz 名义（未记录实测回归） | **10.0000 Hz**（`CustomMsg` 戳线性回归，120 帧，误差 <1e-12） | — |
| 墙钟频率 | — | **9.72 Hz**（台架 headless）/ **9.48 Hz**（整栈） | 台架 RTF ≈0.97；整栈只记了 9.48 Hz，未单列 RTF |
| `offset_time` | **恒 0** | 最大 **99,996,667 ns**，**全帧单调** | 新增时间维 |
| `point_step` / `rgb` | **32** / **有** | **26** / **无** | 无本仓消费者硬依赖 |
| 双发布 | 1 次 publish 2 消息 | 2 实例；**119/119 帧同 stamp 同点数**、**313,721 点 atol=0** | 细表 §3.2 |
| 自机排除 | 靠几何缺失 | A/B：**8040.5 点 vs 0 点**，`r_min 0.4292 m` | 细表 §3.2 |
| `/scan` | 链路存在（历史） | **418 条有限波束、2.75–8.09 m**、`frame_id=base_link` | linefit → p2l 在 gz 档**打通** |
| LIO（`small_point_lio`） | 可用（§J/§N 有全栈数字） | ❌ **未通过**：`imu loop back`、无 `odom→base_link` | 见 §4 U1 |
| QoS 慢消费者 | 历史上有"写者阻塞"（Classic 那次） | best_effort **9.711 Hz** / reliable **9.689 Hz**（**都没被堵**） | 见 §2.2 / §4 U6 |

### 3.2 三个专项实验（原始数字）

**(a) 双实例：BVH 共享 + 逐点一致性**【实测，台架世界 `rmuc2026_gz_bench.sdf`，headless，120 帧】

| 判据 | 实测 | 结论 |
|---|---|---|
| BVH 是否只建一份 | 日志 `Embree map bootstrapped with 1 supported visuals` **只出现 1 次**（不是 2 次）；两实例 `<map_key>default</map_key>` 相同 ⇒ 同一 `EmbreeMapPtr` | ✅ 共用一棵 |
| 逐帧求交 | 单实例 min 0.307 / median **0.325** / max 0.358 ms @30000 rays；双实例合计 ≈0.65 ms | ≈2×，符合预期 |
| 内存 | 328 MB（只有传感器）/ 441 MB（含 robot11） | 没有"BVH 两份"的台阶 |
| `frame_id` / 时间戳 / 点数 | 两路都是 `livox_frame`；**119/119 同 stamp**；`n_delta_max = 0` | ✅ |
| 逐点坐标 | **313,721 个点 xyz 完全相同（atol=0，比特级）** | ✅ 两个实例算的是同一次求交结果 |

**(b) 自机排除 A/B**【实测，整栈世界 + robot11 的 113 个真碰撞体，同一位姿只切一个开关】

| 量 | `ignore_self:=true`（生产档） | `ignore_self:=false`（对照档） |
|---|---|---|
| 建树日志 | `… with 1 supported visuals` | `… with 14 supported visuals` |
| 每帧点数（中位） | **8040.5** | **0**（`n_max` 也是 0） |
| 最近点 `r_min` | **0.4292 m**（> 0.6×0.6 底盘半对角 0.42 m） | —（没有点） |
| `r<0.12 m` / `r<0.30 m` 的点 | **0 / 0** | — |
| RTF | 0.9686 | 0.9985 |

⇒ 对照档是 **0 点而不是"看见自己"**：传感器被自己的碰撞体整个包住，每条射线都从几何体内部出发 ⇒
单面几何判不到有效交点 ⇒ **全盲**。这比"看见自己"更糟，也更能说明这条排除是**必需**的。
（`r_min=0.429` 只说明"没有近场回波"，`ignore_self` 的 A/B 才是"看不见车体"的证据。）

**(c) QoS 慢消费者矩阵**【实测，2 s/帧的慢订阅者 + 快探针 + `/clock`】

| 场景 | 写者 QoS | 快探针墙钟频率 | `/clock` 相邻最大仿真跳变 |
|---|---|---|---|
| 基线（无慢消费者） | best_effort | 9.743 Hz | 18 ms |
| **慢消费者在时** | **best_effort** | **9.711 Hz** | 18 ms |
| 慢消费者退场后 | best_effort | 9.705 Hz | 21 ms |
| 基线（无慢消费者） | reliable | 9.703 Hz | 26 ms |
| **慢消费者在时** | **reliable** | **9.689 Hz** | 32 ms |
| 慢消费者退场后 | reliable | 9.730 Hz | 22 ms |
| **`SIGSTOP`（不 ack）** | **best_effort** | **9.740 Hz** | — |
| **`SIGSTOP`（不 ack）** | reliable | 9.660 Hz | — |

⇒ **两种 QoS 在本机都没被堵**。收益是契约与保证，不是实测提速（§2.2）。

### 3.3 怎么读这三张表（三条限制）

1. **没有同轮 A/B**：Classic 列是历史基线，机器上同时还有别的 agent 在跑 ⇒ 单个数字有 ±3% 抖动。
2. **"永不阻塞"没有在本机复现**：所以不要把 9.711 vs 9.689 当成"best_effort 更快"的证据。
3. **gz 侧的端到端只覆盖 `mode:=mapping`**：`mode:=slam_nav` / `mode:=nav` 未验证；且因为 LIO 未通过，
   `/cloud_registered` 一路没有数据（§4 U1/U5）。

---

## 4. 缺口与未验证（四份文档去重后 **24 条**）

### 4.0 先说 4 处口径冲突（本文取了哪一版、为什么）

| # | 冲突 | 本文取 | 为什么 |
|---|---|---|---|
| 1 | `fortress_sim_backend.md` §2.8 **抬头**写"Classic 的数字是本机同轮实测"，同节**表头/表尾**与 **§10 第 8 条**都说"Classic 是既有基线、**没有**同轮 A/B" | **取后者**（历史基线） | §10-8 是"未验证清单"里的自述，且**指名了来源**（`livox_sensor_prior_art.md` §1/§10.3），还给了理由（本轮时间全花在 gz 侧）。抬头那句应是笔误 |
| 2 | 本轮任务摘要说 gz 新增"**+3 Actions**"，`fortress_sim_backend.md` §9 说"**61→63**（净 +2）" | **取"3 处变化 / 净 +2"** | 两者不矛盾但口径不同：3 处 = 1 个新增 `DeclareLaunchArgument` + 2 个 `IncludeLaunchDescription`，其中 classic 那条是**原 Action 加了 condition**（不是新增）⇒ 净 +2。`--strip-conditions` 的 +2/−0 是同一件事的另一种数法 |
| 3 | `cod_vs_ours_frame_architecture.md` §0-5 说"放大成 6 个，其中 **4 个**是我们自己造的"，§7.2 小结说"**6 个**可避免耦合，**4 明确该修 + 2 半修**" | **取 §7.2** | §7.2 有 O1–O6 逐条证据与"该不该修"的裁决；§0 那句是摘要，措辞把 O5/O6 归到了"半修"之外 |
| 4 | `machine_hygiene_runbook.md` §6 说"已清掉 **9 个**脚本里的这类写法"，§8.2 逐文件表是 **14 个** | **取 14** | 9 = 最初点名的清单；实际改的是 14 个（§8.6 另说"18 行里 11 行在点名的 9 个脚本，另 7 行在同族 4 个脚本里，也一并改了"） |

### 4.1 高风险：阻断闭环 / 使结论不可用（7 条）

| ID | 缺口 | 证据 / 现状 | 影响 | 下一步 |
|---|---|---|---|---|
| **U1** | **gz 侧 LIO 未通过** | `small_point_lio` 持续 `imu loop back`（`preprocess.cpp:67`）、无 `odom→base_link`、`view_frames` 无 odom/map 边、`/cloud_registered` 空；发布端已排除（`+10 ms` 严格单调、`Publisher count: 1`） | gz 侧**不能**做定位/建图闭环；`map→odom→base_link` 的**单发布者契约没有被真正考到** | 先 `lio:=fastlio` 复跑（FAST-LIO 的 IMU 订阅是默认 reliable，与我们的写者兼容）；再给 IMU 回调加"前 20 条戳"临时日志 |
| **U2** | **gz 底盘是 `<static>`** | Fortress 无 `planar_move` 对应物；`/cmd_vel_chassis` 无消费者 | gz 侧**不能**跑 nav2 移动闭环 | 写一个 gz 侧的 `cmd_vel` → 位姿插件（未做） |
| **U3** | **没有同轮 Classic A/B** | Classic 列来自既有文档（§4.0 冲突 1） | §3 的"变化"列**只能看量级**，不能当精密对照 | 同机同刻跑一次 Classic 同参数（fortress §10 第 8 条） |
| **U4** | **gz 只支持 1 场地 + 1 机器人** | `world:=` 只 `RMUC2026`；`robot:=` 只 `robot11` | 其它场地/`hzmirm` 在 gz 档**直接报错**（明确报错，不静默） | 加 `worlds/<name>.sdf`（SDF ≤1.9）并登记 `WORLD_MAP`；其它机器人要从 URDF 派生 |
| **U5** | **gz 未验证 `mode:=slam_nav` / `mode:=nav`** | 端到端只跑了 `mode:=mapping` | nav2 在 gz 档的状态未知 | 等 U1 修好后再跑 |
| **U6** | **QoS "写者被堵"未复现** | best_effort 与 reliable **都没被堵**（§3.2c） | 这条改动的收益只能表述为"契约 + 按定义永不阻塞的保证"，**不能**表述为"实测提速" | 保留 `gz_qos:=reliable` 一键开关，将来复现 |
| **U7** | **§J 的 `1.05°` 未复现** | 本轮最大平面 `d = 0.103 m`、出生 yaw **0.01°**；§J 那轮 `d = 0.26 m`、yaw **10.12°** ⇒ 不是同一块面/同一姿态（世界在 10-09 后被改过；本轮未动 `src/simulation/rm_simulation/**`） | **不主张"绝对角 = 1.05°"**，只主张判决性签名（bug ② 的 30° 签名不存在：0.046° vs 29.977°） | 回到 §J 那个世界/出生点版本才能逐数对齐 |

### 4.2 中风险：结论必须标"推断"（8 条）

| ID | 缺口 | 证据 / 现状 | 影响 |
|---|---|---|---|
| **U8** | COD 本地 checkout **没有 `urdf/`** | `third_party/cod_nav_2026/` 只有 `src/`（190 文件） | "`T_bl ≈ I`"是**推断**（强旁证：若它不是单位，COD 的 `odom→base_link` 会明显错，nav2 不可能正常工作） |
| **U9** | COD 的 `odom` 是否真随雷达斜 30° | 由 `eskf.h:30`（`R_W_B(0)=I`）+ `small_point_lio.cpp:54-64`（`gravity` 取 body 系测量的和）**推出**的，**没有在 COD 数据上实测** | COD 文档 §4.5 的三条后果（STVL 斜板、p2l 斜板、footprint 差 `cos30°`）都是**推断**；若 `gravity` 被后续观测拉回真垂直，结论要改 |
| **U10** | COD 若缺 `base_link→livox_frame`，`:65` 的 `lookupTransform` 会抛异常 ⇒ `:68` 直接 `return` | 需要实跑才知道他们的图里到底有没有这条边 | 若真缺失：COD 实际是"没有任何 `odom→base_link`"⇒ 推翻 COD 文档 §2.1(c) 的表述顺序，但**不推翻结论** |
| **U11** | COD 选项 B1a 需改 `src/localization/rm_localization/small_point_lio/**`（跨 agent 权限） | 本轮只给方案与验收，**未落地** | 选项 B 的成本里含一次跨 agent 协调 |
| **U12** | 无真机、无 MID-360 | 所有数字来自本仓仿真 + bag 重放 | 真机上 `R_bl` 由谁提供（URDF 关节 vs 用户 `static_transform_publisher`）**未核实** |
| **U13** | 微基准不是 `perf` 级剖析 | 24.4 ns 是纯算术热缓存；"14 个 double 拷贝 + 成员读取"的额外开销按量级估 `<0.1 µs/帧` | 成本结论是"量级可忽略"，不是"精确 24.4+5 ns" |
| **U14** | gz 侧**动态障碍 refit** 未走到 | 世界全静态、机器人在 gz 侧也是静态；`delta_trans/delta_rot` 分支未被触发 | 动态场景下 BVH 的增量更新**未验证** |
| **U15** | gz **GUI** 无成体系数字 + 无 CPU 剖析 | 整树 RSS 910 MB、点云探针没采到数据；CPU 只有 0.5 s 采样的累计 tick | GUI 路径与 CPU 峰值都不能引用 |

### 4.3 低风险：覆盖率与流程（9 条）

| ID | 缺口 | 说明 |
|---|---|---|
| **U16** | 一致性断言只在"**有点云输出**的帧"上跑 | 某帧 `pointcloud_odom_frame` 为空则该帧不做断言（`/odom` 仍照发）⇒ **覆盖率的缺口**，不是数值风险 |
| **U17** | 没有 latched `UInt64` 违例计数话题（**有意**） | 加话题会改话题契约（§J.3 的"每话题恰好 1 个发布者"会多一条）；计数写进同一条限速 ERROR |
| **U18** | 故障注入只在 `.tmp_guard/` 的临时副本里做过 | 仓库源码里没有共轭写法、也没有任何注入开关 |
| **U19** | `R_bl` **时变**（云台/机械臂）未验证 | 守卫按"固定外参"设计；`T_bl` 每帧重查但 `#1` 只打一次；时变时 `#2` 会喊（这是对的） |
| **U20** | **沙箱限制**：root 命令 **0 条执行**；无宿主机 PID 视图 | `sudo` 被 `no-new-privs` 挡死 ⇒ swapfile/zram/swappiness/oomd 调参**全未执行**（命令原样给出）；"每进程 VmSwap top15"原理上取不到（只有 4 个 PID），改用 cgroup `memory.swap.current` |
| **U21** | 14 个改造脚本里 **11 个**只做了 `bash -n` + 合成判定 | 没有逐个跑完整流程（动辄 5–10 分钟）；3 个真跑过（`run_nav_smoke_regress` / `run_remap_lowterrain` / `run_robot_model_probe`） |
| **U22** | `start_sentinel.sh` 仍是全机 pkill | `:165` gzserver / `:166` gzclient / **`:169` `pkill -f ros`** / `:172` rviz2 —— 一键启动人工工具，会杀掉别人的 ROS |
| **U23** | 资产/规模覆盖不足 | `ign sdf -k` **未扫全仓** SDF（只做了 2 个世界）；多实例 **>2**、多传感器同世界**未测**；Classic 的 `meshes/obstacles/*` **没有**进 gz 世界 |
| **U24** | COD 侧三条外围未知 | `cpp_lidar_filter` 的裁剪盒到底按哪个系标（源码自相矛盾）；`map→odom` 静态 TF 的 `yaw=-0.5` 从哪来（无注释）；是否另有**仓库外**启动项提供 `base_link→livox_frame`/`urdf` |

**计数**：**24 条**（高 7 / 中 8 / 低 9）。另有 4 处**口径冲突**（§4.0，不是缺口，是"两份材料说法不同"）。

---

## 5. 回退表

| 线 | 回退动作（按"最小 → 彻底"） | 影响 | 注意 |
|---|---|---|---|
| **L1 外参守卫** | ① **只想静音**：不动代码，日志过滤（`grep -v 外参守卫`）② **彻底**：`git revert fe0dc31`；或手工删 `src/extrinsic_guard.cpp` + `small_point_lio_node.cpp` 的 2 处调用（`:193-194` 快照、`:208-209` 调用）+ `small_point_lio_node.hpp:44-63` 成员 + `CMakeLists.txt:74-78` 的 `APPEND` + 单测 ⑥（`tf2_compose_order_test.cpp:155-201`） | 只丢日志/计数；**发布数值逐位不变**（§O.4(a) 已逐位证明） | 别只删 `.cpp` 不删调用点（链接期 undefined symbol） |
| **L2 收尾作用域** | ① **单脚本回退**：把该脚本的 `sc_*` 调用换回原来的 `pkill`（每处改动都留了注释）② **彻底**：`git revert a137bf2`（同时删 `tools/scripts/lib/scoped_cleanup.sh` 与 runbook） | 回退即恢复"全机 pkill"⇒ 会误杀同机别人的栈 | **不要**只删 `scoped_cleanup.sh` 却留着 `source`（14 个脚本全部报错） |
| **L3 gz 后端** | ① **最省事**：**不传 `sim_backend`** ⇒ Classic（默认路径零改动）② **代码**：删 `src/simulation/rm_simulation/rm_livox_gz_sim/` + 回退 `bringup_sim.launch.py` 的 3 个 Action（`declare_sim_backend_cmd` `:1223`、classic 的 `condition` `:1586`、gz 的 `IncludeLaunchDescription` `:1591-1606`）③ **构建前缀**：`rm -rf .tmp_gzlivox2/gzsim_ws`（没装进 `/opt/ros`、不碰系统库；换一份用 `GZSIM_PREFIX` 指） | ①②③ 之后回到 **100% 原状** | **不建议** `git revert 057294c`：gz 的代码内容随它入库，整提交 revert 会**连 COD 文档一起撤掉** |
| **L4 资产修正** | `git revert da5c2fe`（两份 `world/model.config` 的 `<sdf>` 回到空） | Classic 不受影响；**gz 世界再也起不来**（`Failure to detect an sdf tag`） | 这 2 行是 gz 的**硬前提**，不是可选优化 |
| **L5 文档** | 删 `docs/round_summary_gz_frames.md` + 撤 `docs/README.md` 里那一行 | 无 | 纯文档 |

---

## 6. 用法速查

### 6.1 gz 后端（两步）

```bash
# ① 一次性：取上游 + 打补丁 + 编译（源码不进本仓库，编到独立前缀）
tools/scripts/gzsim/fetch_and_patch_mid360.sh

# ② 整栈：与 Classic 同一条 launch，只多一个参数
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz \
    world:=RMUC2026 mode:=mapping lio:=small_point_lio \
    robot:=robot11 robot11_mount:=sensor gui:=False spin_speed:=0.0
```

**两个硬要求**（缺了会明确报错，不静默跑错）：

| 要求 | 为什么 |
|---|---|
| `robot:=robot11` | 只有这个槽位在 gz 侧有 Fortress 化的模型生成路径（从同一份 URDF 派生 SDF） |
| `robot11_mount:=sensor` | 倾角必须在 **URDF** 里，`robot_state_publisher` 发的 `base_link→livox_frame` 才与物理一致（gz 的 SDF `<link><pose>` 也真的带 roll −0.5236） |

> `robot11_mount:=urdf` 在 gz 侧**也能跑**（`fortress_sim_backend.md` §2.7），但那一档"账是错的"
> （COD 文档 O3），只适合做反例 ⇒ 生产用 `sensor`。

**台架（只看传感器，不开感知/LIO）**：

```bash
ros2 launch rm_livox_gz_sim gz_sim.launch.py robot:=none gui:=False
# 带 GUI（无 GPU 机器要 Xvfb；本仓不替你起）
xvfb-run -s "-screen 0 1280x800x24" ros2 launch rm_livox_gz_sim gz_sim.launch.py robot:=none gui:=True
```

**Classic 原命令不变**：去掉 `sim_backend`（或不写）就是 Classic 默认路径。

### 6.2 其它常用

```bash
# 看守卫有没有喊（启动 readout / 每帧断言）
ros2 launch … 2>&1 | grep -E '外参守卫|位姿一致性断言失败'

# 单测第 6 条（不进 colcon）
g++ -O0 -o /tmp/tf2order tools/scripts/tiltmount/tf2_compose_order_test.cpp \
  -I/opt/ros/humble/include/tf2 -L/opt/ros/humble/lib -ltf2 -Wl,-rpath,/opt/ros/humble/lib && /tmp/tf2order

# 脚本收尾排障：看每个 own/skip 的原因
SC_SCOPE_VERBOSE=1 bash tools/scripts/…

# 默认路径零改动的自检
python3 tools/scripts/gzsim/launch_increment_diff.py --old /tmp/old.py \
  --new src/bringup/rm_nav_bringup/launch/bringup_sim.launch.py
```

> ⚠️ 本机 `$HOME` 只读，`ros2 launch --show-args` 会在 `~/.ros/log` 上 OSError（**与本次改动无关**）。
> 跑这类命令前把 `HOME`/`ROS_LOG_DIR` 指到可写目录。

---

## 7. 文档索引（这轮新增/更新的）

| 文档 | 变化 | 行数 | 看它解决什么问题 |
|---|---|---|---|
| `tilted_lidar_fidelity.md`（原文已删除，见 git 历史）| 追加 **§O**（+157 行） | 2987 | **守卫为什么这么写**：阈值、插入点、独立 TU 的 A/B、24.4 ns 成本、单测 ⑥、三段回归、诚实清单 |
| [`machine_hygiene_runbook.md``machine_hygiene_runbook.md` | **新增** | 701 | **swap 到底满没满**（没满）、该不该动、oomd 为什么杀整组、哪些命令需 root（一条没跑）、脚本收尾为什么只杀自己 |
| `fortress_sim_backend.md`（原文已删除，见 git 历史）| **新增** | 547 | **gz 后端怎么用**、与 Classic 的 11 条差异（§2）、契约逐条实测（§5）、自机排除（§6）、端到端（§8）、会坏的地方（§2.9）、回退（§2.11） |
| [`gazebo_sim_migration_plan.md``gazebo_sim_migration_plan.md` | 追加 **§11**（+25 行） | 1862 | **P0–P4 计划的最终状态**：哪项完成、哪项部分完成（P1 底盘、P4 A/B、LIO） |
| [`cod_vs_ours_frame_architecture.md``cod_vs_ours_frame_architecture.md` | **新增** | 607 | **COD 为什么没事**（`base_link≡livox_frame`）、我们哪 4 条耦合是自己造的（O1–O4）、C/B/A 三选项与验收命令 |
| [`round_summary_gz_frames.md``round_summary_gz_frames.md` | **新增（本文）** | 512 | **本轮四条线的合并入口**：一页看懂做了什么、与之前的区别、数字、缺口、回退、待办 |

---

## 8. 待办

| # | 待办 | 谁定 / 成本 | 依据 |
|---|---|---|---|
| **1** | **`robot11_mount` 的 `urdf` 去留**：选项 C 建议从 `choices`（`bringup_sim.launch.py:172`、`:1303`）移除，只留 `plugin`（默认）+ `sensor`（保真），`urdf` 降级为**文档里的反例** | **等你拍板**（≤1 人日，含验收 4 条） | COD 文档 §8 选项 C / O3 |
| **2** | **`start_sentinel.sh` 的全机 pkill**：`:169` 的 **`pkill -f ros`** 会杀掉整机所有 ROS 进程（还有 `:165` gzserver、`:166` gzclient、`:172` rviz2） | 低（把它也改成 `setsid` + 本 URI 核对） | `machine_hygiene_runbook.md` §8.7 第 1 条 |
| **3** | **`cod_nav_comparison.md:27/:43` 更正**：`:27`「全局 costmap 只吃 `/scan`」对 Gitee 2026 **不成立**（两张图都不吃 `/scan`）；`:43`「带在雷达面**之上** 0.01~1.0」实际是 **`0.15 / 1.0`** | 低（2 行） | COD 文档 §3.1 |
| **4** | **gz 侧 LIO 验证**（最高优先）：先 `lio:=fastlio` 复跑，确认 gz 后端本身能不能喂出 odom；再决定要不要给 `small_point_lio` 的 IMU 回调加临时日志 | 中（需跑仿真） | `fortress_sim_backend.md` §8.1 / §10 第 5 条 |
| **5** | **`fortress_sim_backend.md` §10 的 item 8 / item 9**：**8 = Classic vs gz 的同轮对照性能**（现在"变化"列只能看量级）；**9 = gz GUI（Xvfb）的成体系数字**（现在只有整树 RSS 910 MB，点云探针没采到数据） | 8 需要同机同刻跑两遍；9 需要 GUI 环境 | 同上 §10 第 8/9 条 |
| **6** | 其它（成本更高，先记）：COD 的 `urdf/` 取证（U8）· `R_bl` 时变（U19）· 全仓 SDF `ign sdf -k` 预检（U23）· 选项 B（adapter 唯一权威，M 级，需跨 agent） | 高 | §4 |

---

## 附：一句话记住这轮

**我们查出的 6 个"斜装雷达问题"里，2 个是上游 bug（已修 + 已加守卫）、4 个是我们自己的默认值/接线；
COD 之所以看起来"没事"，是因为它把 30° 藏进了 `base_link` 的定义里 —— 不是它更对，是它更看不见。**
（`cod_vs_ours_frame_architecture.md` §6.2）


---


## 来源 5/7：`research/round_summary_hybrid_default.md`


> 范围：`d36dbc1..95ae404`（24 个提交，30 文件 +5935/−209）。
> 一句话：**把 `odom` 的"朝向 + 原点"两件事一次修干净，并顺路挖出四个静默失效的真实缺陷；最后 `hybrid` 成为 `lio:=small_point_lio` 的默认重力对齐档。**
> 标注约定：**【实测】** = 本仓跑出来的数；**【推断】** = 未实测；**【引用】** = 外部资料。

---

## 一、这轮定下的五件事（结论 + 判据）

| # | 结论 | 关键证据 |
|---|---|---|
| 1 | **`odom` 的 z=0 = 初始化时刻的地面**（`gravity_align_z_origin:=ground`，仅在重力对齐档生效；`h` 由 URDF 解析 + 运行时地面互检） | init 时 URDF 地面 0 / `base_link` +0.1025 / `livox` +0.2595（精确）；`h_meas` Δ ≤ 2 cm ⇒ INFO |
| 2 | **安装参数真的从 URDF 读**（`gravity_align_rpy` / adapter `xyz,rpy` / `h` 先验），旧表降为交叉校验 | 6 组 parse-vs-table **Δ=0**；scratch 把 30°→20° ⇒ 派生值变 **−20°** 且 **WARN**；默认路径 **执行集 0 改 0 删** |
| 3 | **Livox 插件 `cloud_frame=sensor` 档的坐标 bug 修好**（点云曾被整体平移 `(0.6, 191.9, 70.5) mm`） | 拿网格已知墙面当基准：y=−8.5 残差 **−0.1919 → 0.0000 m**；`h_meas` **0.189 → 0.2603/0.2597（两世界都 INFO）** |
| 4 | **nav2 的高度带会在四个地方静默丢数据**（局部半平面 / 全局整层清空 / gicp 门自锁 / `map→odom` 反向注入倾角），现已有三处修复 + 一条只读守卫 | 见第二节 |
| 5 | ⭐ **`hybrid` 成为 nav 默认**（`gravity_align_mode:=auto`：`lio==small_point_lio` ⇒ `hybrid`，否则 `off`） | 默认 `fastlio` 路径**运行时逐字节不变**；`off`+SPL **逐字节回退**；显式 `hybrid`+`fastlio` 仍**逐字报错** |

### 为什么是 `auto` 而不是直接把默认写成 `hybrid`

launch 的默认 `lio` 是 **`fastlio`**，而 E1/E1b 的补丁只存在于 `small_point_lio`；`_check_gravity_align_contract` 对"非 off + 非 SPL"是**硬报错**。
⇒ 默认直接写 `hybrid` 会让**所有默认跑法在 t=0 报错**。`auto` 让两种默认都成立。

---

## 二、四类"静默失效"及其修复

| # | 位置 | 症状（都**无日志**） | 修复 |
|---|---|---|---|
| 1 | **局部** `obstacle_layer`（`odom` 帧） | 扫描面倾斜 ⇒ 高度带只收一条带：**in-band 333/776**、kept-y **[0.112, 3.568]**（单侧）⇒ **半平面盲区**（局部图 53.2% 面积永不 lethal） | `hybrid` 使盘面水平 ⇒ **776/776**、kept-y 双侧 |
| 2 | **全局** `obstacle_layer`（`map` 帧） | `map` 里扫描面 z ≈ **−0.1786** ⇒ **0/774** 在带内 ⇒ 全局图恒等于 static+inflation（`3713` = pgm occupied 格数，53 帧 min=max） | `457701c`：opt-in 覆盖层把**两处** `min_obstacle_height` 放到 **−1.0**；盒子试验：全局 **0 格 → 14/121 格**变化，删除后**逐格回退** |
| 3 | **gicp 首帧门** | `dt` 取"与上次**采纳**帧的戳差"，从未采纳 ⇒ `dt=0` ⇒ 门限卡下限 0.1 m ⇒ **采纳 0/25** ⇒ 6 s 后判失效、停发 `map→odom` | `9bb8f2f`：`dt` 改"上一帧收到" + 首帧免创新门 + 连续 3 拒**自愈重锚** + 失效态探针 ⇒ **采纳 100%**、创新 p50 **0.134 → 0.0097**、`TFDrop 42 → 0`、外推 `271 → 3` |
| 4 | **`map→odom` 的倾角** | `off` 档 `map→odom` 带 **29.75° roll**，把 30° 安装倾角**反向抵消** ⇒ `map` 帧反而是平的 | **不是缺陷而是"活定位器"的正常结果**；它解释了为什么**全局半不吃倾斜病的亏**，也**推翻了 §12.4 的"加倍成 60°"推断** |

**只读守卫 `band_guard`**（`65a0a85`，opt-in）：把 1、2 两种失效**当场喊出来**（`in-band / kept-y / z / frame / band` + 检查建议）；同栈 A/B 证明它**不改变运行结果**（local lethal 266→265、global 3898→3897）。

---

## 三、提交清单（24 个）

### A. 帧契约与 LIO（6）
| commit | 内容 |
|---|---|
| `1b5db4b` | **E1b**：`odom` 的 z 原点 = 地面（`gravity_align_z_origin` / `gravity_align_h` / `h_meas` 互检 / 独立 TU `gravity_align_ground.cpp`） |
| `4987dc4` | 契约 §4.7（z 原点规范）+ measurements §10（验收 A/B） |
| `f9bf4a9` | **安装参数从 URDF/xacro 解析**（+ 弧度守卫 `_assert_radians`；旧表降为交叉校验） |
| `023d2e8` | 契约 §13.5 + measurements §11（解析对照 + 30°→20° 决定性测试） |
| `6473a29` | ⭐ **`gravity_align_mode` 默认 `auto`** + 唯一解析点 + 契约 §4.10 + measurements §18 |
| `95ae404` | §18.4 守卫样本数订正 |

### B. 物理/世界（2）
| commit | 内容 |
|---|---|
| `72c7c07` | **修 Livox 插件 sensor 档**：不再叠加父 link 系 `offset.Pos()`（`parent` 档逐字节不变） |
| `ec5028f` | **RMUL2026 地面校准**到 world z=0（Δz = −0.049662） |

### C. 定位/代价图（4）
| commit | 内容 |
|---|---|
| `d475605` / `bcb652b` / `dafcb4e` | gicp `~/gate` 逐帧观测量 + 创新分量（dx/dy/dz）+ 探针 |
| `9bb8f2f` | ⭐ **修 gicp 首帧门死锁**（4 个新参数，默认=修好；逐字节回退四开关） |
| `555eb96` | measurements §14（复现/成因/修法/验收/全局半）+ §9.10 订正 |
| `457701c` | ⭐ **修全局 `obstacle_layer` 被 z 闸静默清空**（opt-in 覆盖层） |
| `bd18eaa` | §15 抬头措辞订正 |

### D. 工具/守卫/地图（4）
| commit | 内容 |
|---|---|
| `acae43f` / `49a4328` | 打包脚本：项目根少一级、`tar`/`mv` 目标、分发包 setup 根目录自适应（真跑通过：398 MB / 2187 条目） |
| `b1434b8` | **地图重建**（新名字 + sidecar，默认未提升）+ measurements §16 |
| `65a0a85` / `d881147` | **band_guard**（opt-in 只读守卫）+ measurements §17 |
| `9835957` | measurements §12（`slam_nav` 与 `nav+global:=scan` 验收） |

### E. 文档（1）
| commit | 内容 |
|---|---|
| `38a2d94` / `c616d69` / `c0b3803` | measurements §13（世界校准 + 插件 bug、作废值换算、日期口径） |

---

## 四、被**作废/修正**的旧数字（引用前必看）

| 旧说法 | 现在 | 原因 |
|---|---|---|
| 「机器人陷进地面 **6.9 cm** / 可见地面 0.1183 / odom 里 +0.069」 | ❌ **作废**：真实接触地面 0.049662，**陷地 = 0（2 µm）** | 0.1183 = 真实接触地面 + **插件 0.07053** |
| `h_meas` **0.1894 / 0.1906**（ERROR） | 修插件后 **0.2603 / 0.2597（INFO）**；校准后 **0.2616** | 插件坐标 bug |
| §10.4 的 ground-in-odom **0.01876 / 0.02674** | 同定义新值 ≈ **−0.0518 / −0.0438**（各减 0.07053；y 减 0.1919） | 同上 |
| §9.10「off 118 / hybrid 2」 | ❌ **错**：两档都 `采纳 0/N`（门死锁），条数只取决于跑了多久；修门后 **3 / 0** | gicp 门自锁 |
| §12.4「off 会加倍成 **60°**、只剩 1.19 m 条带」 | ❌ **不成立**：活定位器下 `map→odom` 带 −29.75° roll 抵消倾角，`map` 帧两档都平 | **推断**当成了实测 |
| 「39.6/40.4% 出带」 | ⚠️ **与 14.2% 不是同一个量**：前者是另一次 `sensor` 档 mapping 跑（755 波束），后者是本次 nav A/B（96/678）；**可复用的是机理（斜面 ⇒ 定向丢），不是百分比** | 跨跑混用 |
| 「创新 0.11–0.22 / 采纳 0 = 旧图偏置」 | ❌ **错**：那是**门死锁**的像；门修好后旧图也 **100% 采纳**，地图偏置表现为**一次性 `map→odom` 常量修正** | 归因错 |
| 「修出生点抬 6.9 cm」（1c 方案） | ❌ **取消**：车根本没陷进去 | 诊断被推翻 |
| §4.1 / §13.2 / §13.4 的「默认 off」 | ⚠️ **已被 §4.10 取代**（正文保留，§4.10 声明取代） | 默认已改 `auto` |

---

## 五、仍未解决 / 未验证（诚实清单）

### A. 四条硬化（已知、未做）
| # | 问题 | 影响 |
|---|---|---|
| 1 | **默认模型（hzmirm，13 link）底盘自转只有 3%**（0.80 → 0.025 rad/s；`robot11` 焊接版是 100%） | **建图覆盖不够**（26 min 只走 10.6 m）⇒ 新地图没提升 |
| 2 | **`small_point_lio` + 默认模型（`R_bl=I`）SIGSEGV**（~12 s，可复现，无 `/odom`） | **默认模型 + SPL** 这条组合直接崩 |
| 3 | **`cloud_frame=parent` 档疑似 `frame_id` 与点云所在系不一致**（驱动版地面 +0.283 m、`map→odom z=+0.403`） | 若成立 ⇒ **默认挂法**有偏移；**未证实** |
| 4 | `map_archive.sh save` **静默忽略 `--resolution`** | 存图传分辨率会**无声不写盘** |

### B. 基线/资产
- **仍在用旧地图**（带插件偏置；gicp 吸收成常量修正，nav 可用）。新候选：`RMUC2026_v4.pcd`（墙面残差 p50 **0.0436 → 0.0020 m**、gicp score **5.1×**）等，**未提升**。
- **`§10` / `§12` 基线未在"当前 master + 修好插件 + 校准世界"上重跑**（旧表是混合条件下测的）。
- **全局 `−1.0` 覆盖可能只是旧地图的拐杖**（`map→odom` z 差 0.113 m ≈ 插件 0.0705 + 世界 0.0497）⇒ **地图重建后应复测"还需不需要"**。

### C. 未测组合
`hzmirm` / 默认模型在 Gazebo 实跑 · `hzmirm` 云台≠0 · `robot11_mount:=plugin|urdf` · `h_meas` 的 **3–5 cm WARN 带** · `amcl` / `beluga` / `small_gicp` / `fastlio` / `pointlio` 配新默认 · `slam_nav` 配新门 · `RMUC2026` 配全局 scan 修复 · **gz backend** 配以上全部 · 真机。

### D. 文档与运维
- `docs/README.md` 的取代表需要为**新的数字口径**（14.2% / 39.6–40.4% / 41.2% 三者关系）加一行。
- `spl_ab_bench.sh` 收尾会把 `build/` 二进制还原成 `.tmp_uc/bins/after`（zfix_prev）⇒ **跑完要手工还原 zfix `58bebfb8…`**。
- `band_guard` 默认路径新增 **2 个恒假 Action**（不是严格"0 新增"；节点不启动、参数/控制路径不变）。
- **KISS-Matcher 全局重定位调研未做**（曾提议，未派）。

---

## 六、怎么跑（当前正确配方）

```bash
# nav（hybrid 现在自动生效；不再需要显式 gravity_align_mode:=hybrid）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=nav lio:=small_point_lio localization:=gicp \
  nav:=mppi planner:=smac2d spin_speed:=0.0 robot:=robot11 nav_rviz:=True \
  robot11_mount:=sensor band_guard:=True

# 只想回退到旧行为（逐字节）
... gravity_align_mode:=off
```

- `gravity_align_mode:=auto`（默认）⇒ `lio==small_point_lio` 时取 `hybrid`，否则 `off`；t=0 会打一行收据说明解析结果。
- `band_guard:=True` 建议在**回归/改帧/改高度带/改代价图配置**时开。
- 地图：**仍用默认资产**；想试新候选按 `docs/map_assets.md` §4.2 走。

---

## 七、文档索引导航（本轮新增/更新）

| 文档 | 内容 |
|---|---|
| `docs/frame_contract_v3.md` **§4.7** | z 原点规范 |
| 同上 **§4.8** | 世界地面校准是前提 |
| 同上 **§4.9** | `band_guard` 登记 |
| 同上 **§4.10** | ⭐ **默认档决定**（取代 §4.1/§13.2/§13.4 的"默认 off"） |
| `docs/frame_contract_v3_measurements.md` **§10–§18** | 依次：z 原点验收 · URDF 解析 · slam_nav/global-scan · 世界校准 + 插件 bug · gicp 门 · 全局层 · 地图重建 · 守卫 · 默认 auto |

---

# 第二轮补遗：物理 · 地图 · 基线（`e926421..499a4d9`，14 个提交）

> 第一轮把"朝向 + 原点 + 安装参数 + 静默失效"修完并让 `hybrid` 成为默认；第二轮把**物理（底盘/摩擦）**、**地图（重建/提升）**、**基线（在当前 master 上重测）** 三件事收口。

## 一、四条硬化（全部落地）

| # | 缺陷 | 结果 | 提交 |
|---|---|---|---|
| ① | **默认模型自转只有 3.2–13.1%** | 两个串联限制：多刚体 + **轮子对地摩擦**（`mu` 写在 `<link>` 里 Gazebo 不认）⇒ 单刚体 + `<gazebo reference>` μ=0.02 ⇒ **自转 95.1%、前进 98.7%**；几何逐项不变 | `aa77420` |
| ①' | **robot11 自转只有 10.6%** | 它早就是单刚体（113 collision / 1 link）——**唯一缺的是摩擦**（113 条 `<ode>` 全空 ⇒ ODE 缺省 μ=1）⇒ **台架 95.8% / 整栈 92.9%**，前进 55.2%→**98.2%**；`--no-wheel-friction` 与 HEAD 逐字节相同 | `ce97ca1` |
| ② | **`small_point_lio` + 默认模型 SIGSEGV** | ⚠️ **原崩溃未复现（0/53 跑）**、根因未知；但修了三条**确定性**死亡路径：缺键 ⇒ SIGABRT（原来直接 terminate）、**`base_link→livox_frame` 查不到就 `return` ⇒ 一条 `/odom` 都不发**（静态 TF 在 spawn 后 3–10 s 才出现）、`lio_tf_adapter` 的 `xyz_[0..2]` 越界读 | `e0db764` |
| ③ | **`cloud_frame=parent` 的 frame 不一致** | 点算在 **base_link** 系、标签却写 **livox_frame** ⇒ 整朵云偏**一个安装矢量**：默认模型 **(+0.12, 0, +0.175)**、robot11 plugin **(+0.0006, +0.1309, +0.1570)**；修后**墙面/地面残差全 0.0000**；`h_meas` robot11 plugin **0.1045 ERROR → 0.2618 INFO**；`sensor` 档数学恒等、基线不动 | `7a544bf` |
| ④ | **`map_archive.sh save` 静默不写盘** | 接受并校验 `--resolution`（非数/越界 ⇒ exit 2）、显式值优先；调用方去掉 `\|\| true`、改 `PIPESTATUS`（**不能用 `set -e`**：EXIT trap 会改退出码） | `164b807` |

**坡的代价（诚实）**：μ=0.02 ⇒ 静摩擦角 ≈ **1.1°**；要爬坡用 `--wheel-mu 0.1`（≈5.71°，自转回落到 82.3%）。**真实坡道未实测。**

## 二、地图：三次重建、一次提升

| 尝试 | 流水线 | 结果 |
|---|---|---|
| §16 | robot11 + small_point_lio | 覆盖 43%，**底盘不能转**（3%）⇒ 不提升 |
| §23 | robot11 + small_point_lio + 修好插件 | 覆盖仍 43%（robot11 足印 0.60 m vs 通道 0.48–0.52 m）；**SPL 一驱动就发散**（RMUL ATE 5.34 m；RMUC OOM 16.6 GB）⇒ 不提升 |
| §24 | **默认模型 + fastlio**（旧图的流水线） | RMUL 覆盖 **102.3%**、gicp 创新 **3.4 mm**，但墙 0.155 / 地面 −0.047 ⇒ 不提升（**RMUC 21.6% ⇒ STOP**：走廊里 FAST-LIO `No Effective Points` ×1799 ⇒ `/odom` 发散） |
| **§27** | §24 的 v6 + **一次刚体后处理** | ⭐ **四门全过 ⇒ 提升 RMUL**（RMUC 保持旧图） |

**⭐ 鬼影修法（§24，工程结论）**：新增 `cloud_accumulator_target_frame`（**默认 `map` 不变**）⇒ 累积改在平滑的 **`odom`**，落盘后一次刚体搬到 map。
实测 `map→odom` 全程抖动 **x 0.331 / y 0.231 / z 0.449 m / 1.81°**（RMUL）⇒ 在 map 档累积地面必被抹成 ≥0.21 m 厚；换档后**地面只有 1 cm 厚**、墙是细单线、`skipped_frames_jump 0`。

**RMUL 提升（§27）**：刚体 `dyaw 0.000°`、`t (+0.1408, −0.0147, +0.0461) m`（**纯平移**）。

| 门 | 旧默认 | v6 | **v6al（提升）** |
|---|---|---|---|
| 覆盖（known 面积） | 94.77 m² | 96.948（102.3%） | **96.948（102.3%）** |
| 墙面 raw p50 | 0.04365 | 0.1424 | ⭐ **0.01392** |
| 地面 world z | 不可用 | −0.0470 | ⭐ **−0.0006 m**（±6 mm） |
| GICP 采纳 / 创新 p50 / score | 100% / 10.65 mm / 0.0051 | 420/420 / 3.39 mm / 0.00322 | **420/420 / 3.27 mm / 0.00314** |
| **2D 栅格 raw p50** | **0.175** | 0.075 | ⭐ **0.025（±半格）** |
| `map→odom` | (−0.057, +0.130) | (−0.151, +0.180) | **(−0.026, +0.184)**（y 是量具口径） |

> ⚠️ **纠正 §24**：那个"`dyaw −2.0°`"**不存在** —— 真值里 RMUL 的墙只有 **0.40 m 高**，§24 用 `zrange 0.15–1.6` 把该跑 **z 漂移 0.449 m 的抹影**当成了墙面点。收到 `0.15–0.60` 后四个周界面互相一致到 **0.5 mm**；独立墙线斜率量 `|yaw| ≤ 0.35°`。

**备份**：`map/RMUL2026.{pgm,yaml}.bak-20261020-promote` + `PCD/RMUL2026.pcd.bak-20261020-promote`（md5 `6e45a932 / fd23dd82 / 5d0d33eb`，与提升前逐字节相同，未入库）。

## 三、基线重测（§25，当前 master，11 跑）

| 判据 | **默认（auto⇒hybrid）** | `off` |
|---|---|---|
| in-band `[0,2]` | ⭐ **771/771 = 100%** | **337 = 43.7%** |
| band_guard kept-y | **[−0.428, +5.270] 双侧** | **[+0.257, +3.281] 单侧 +y** |
| 地面残余倾角 | ⭐ **0.464°** | **30.305°** |
| `odom→base_link` rpy | +0.06 / −0.47 / −0.63° | **30.20** / +0.49 / −1.07° |
| `h_meas` | **0.2617 m INFO** | — |
| local lethal | 329 [264–373.5] | 299.5 [297.5–307] |
| global stvl lethal | 3727 | 3725 |

> ⚠️ **判据修正**：`off` 的 lethal 已 **299.5 ≠ 0** ⇒ **"代价图空不空"不再是判据**，只认 **in-band / kept-y**（§9.7 的 lethal 0 是"插件 bug + z 原点"的复合像）。

## 四、两个余量/兜底（§26 + §27）

| 项 | 内容 |
|---|---|
| **局部 2D 带负下界** | 新增 `scan_band_min`（默认 **−0.05**）⇒ hybrid 的最低端点距下界从 **7 mm → 69.9 mm（绝对最小 64.2）**；off 仍 43.7%（**不救倾斜档**）；落在 `[−0.05,0)` 的波束 **= 0**；STVL/global 子树逐字节不变；**回滚 `scan_band_min:=0.0` ⇒ 合并文件逐字节等于改动前** |
| **全局 `−1.0` 覆盖** | **保留，但理由改了**：换图后 `map` 帧扫描面 **−0.179 → +0.119**，0.0 与 −1.0 **逐样本相同**（全局 in-band 都 100%）⇒ §15 的病灶被"换图"治好；**但余量只剩 4–12 mm** ⇒ 从"必需"降级为"兜底"（nav2 会静默清空全局 scan 层） |

## 五、第二轮作废/修正清单（补充第一轮）

| 旧说法 | 现在 |
|---|---|
| §10.4 整表（in-band 579/678、lethal 193/288.5、nearest、ahead、incl、z、rpy） | 由 §25 取代 |
| §10.3 的 `h_meas` 0.1894 / ERROR | 作废（插件 bug） |
| 有限波束 677/678 | **771** |
| §10.4 的全局 3713/16886/10176 | **重读为 static+inflation 基线**（默认 stvl 今天 3727；新图干净跑 2163/12260/12812） |
| §12 的 local 数字 + "全局半判不了" | 作废（门修好后 gicp 采纳 279–295/295、global in-band 100%） |
| §24 的 `dyaw −2.0°` | ⭐ **不存在**（宽 z 窗假象；真实 \|yaw\| ≤0.35°） |
| §16 的"map 系斜 30°" | ⭐ **探针口径错**：hybrid 下 `/odom` 发的是雷达位姿 `T_ol`，拿它减 base 真值 ⇒ 差出 −30°；用 TF 重测 **roll +0.38°**，map 系是平的 |
| "lethal 0 = 病" | 作废，改成 in-band / kept-y |

## 六、仍开放

| # | 项 |
|---|---|
| 1 | **`small_point_lio` 一"驱动"就发散**（建图通路；RMUL ATE 5.34 m、RMUC OOM）——根因未定位 |
| 2 | 原 **SIGSEGV** 的根因（0/53 未复现，只有三条加固兜住） |
| 3 | **RMUC 可达性物理阻塞**（0.42–0.50 m 走廊 vs robot11 0.60 m；默认模型进得去但 FAST-LIO 退化）⇒ 旧图继续用；"旧图 389.9 m² 当年怎么建的"仍无答案 |
| 4 | RMUL 资产的**地面↔平台间距多 43 mm**（一个 dz 只能锚一个面）；`x=5.0` 内墙抹影 0.5 m |
| 5 | `hybrid` 的余量虽到 70 mm，但 **LIO z 漂移 −0.065…−0.077 m** 仍在；换世界/低洼处要复测 |
| 6 | **swap 3.99/4.00 GB 满**（本轮无 OOM，但下一轮建图有风险） |
| 7 | `v6.meta.yaml` 的 `file_sizes` 与磁盘不符（只记账）；`docs/map_assets.md` §2.1 的 RMUL 行已过时 |
| 8 | RMUL 冒烟**没发导航目标**（只订阅）；`hzmirm`/`amcl`/`small_gicp`/`mapping` 组合未测 |

## 七、当前正确配方（第二轮更新）

```bash
# nav（hybrid 自动生效；RMUL 用新图、RMUC 用旧图；带余量与守卫）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp \
  nav:=mppi planner:=smac2d spin_speed:=0.0 robot:=robot11 nav_rviz:=True \
  robot11_mount:=sensor band_guard:=True

# 想更抓地（爬坡）就重生成底盘：--wheel-mu 0.1（≈5.71°，自转 82.3%）
# 想回退重力对齐：gravity_align_mode:=off（逐字节）
# 想回退带余量：scan_band_min:=0.0（合并 YAML 逐字节）
# 想回退 RMUL 地图：cp map/RMUL2026.{pgm,yaml}.bak-20261020-promote ... + PCD 同名
```


---


## 来源 6/7：`research/session_digest.md`

> **STATUS（2026-10，仓库整理）**：本文 §0.3/§2.2.5 的「41.2% 波束被高度带丢掉」已被 `docs/frame_contract_v3_rationale.md` §7⑥ 与 `docs/tilted_lidar_fidelity.md` §K.1.3（39.6% / 40.4%）取代；其余内容仍有效。
# 本工作期总汇总（先看这份）：`robot:=robot11` 全程 + 感知/LIO/导航/建图/资产

> **这份文档是什么**：把本工作期（**2026-10-05 → 2026-10-08**，按 `git log` 作者日期 **161 个提交**）
> 在 `docs/` 下散落的 **67 份文档**里的改动**合并成一份**。你想"只读一份就知道改了什么、为什么改、
> 怎么查出来的、现在的数字是多少、怎么退回去"——读这一份就够；要抠细节再按每条的"证据"列跳回原文档。
>
> **它是怎么造出来的（可核对）**：全部内容来自
> ① `git log`（提交是"改了什么"的唯一真值）；② 下列文档的原文（逐条引用到 `§`）：
> `README.md`、`robot_models.md`、`tilted_lidar_fidelity.md`、`gazebo_gui_troubleshooting.md`、
> `lio_divergence_no_impact.md`、`gicp_divergence_and_jitter.md`、`gicp_initialpose_latency.md`、
> `lio_drift_diagnosis.md`、`slope_speed_limiting.md`、`traversability_plan.md`、
> `path_clearance_and_contact.md`、`timestamp_construction_audit.md`、`slam_toolbox_scan_drops.md`、
> `slam_toolbox_tuning.md`、`slam_drops_loopclosure_odom.md`、`map_assets.md`、`mapping_2d_from_cloud.md`、
> `continue_mapping.md`、`nav2_vs_custom_planner.md`、`architecture.md`、`tf_interface_contract.md`、
> `sim_real_contract.md`、`params_ownership_checklist.md`、`issues_and_findings.md`、`worlds.md`、
> `ground_segmentation_slots.md`、`stvl_local_costmap.md`、`mapping_small_point_lio.md`、
> `lio_slots.md`、`localization_slots.md`。（**没有**读到的源，见 §5.3 的"源可核性"表。）
>
> **证据分级（全文逐条执行，不要混读）**：
>
> | 标记 | 含义 |
> |---|---|
> | **【实测】** | 有可复现产物：跑出来的 JSON/NPZ/CSV、`ros2 param get` 回读、源码 `file:line`、日志原文 |
> | **【推断】** | 由实测推导但**没有**直接测量（写出推断链与证伪条件） |
> | **【未验证】** | 明确登记为没做/没跑完/只有一个样本的事 |
> | **【来源】** | 上游文档/issue/规范/论文（带 URL） |
>
> **答案都是"实测"的**：本工作期的一大特点是**每条结论后面都有一次"判决性测量"**；
> 反过来，凡是**没有**测量支撑的，本文一律标 **【推断】/【未验证】**，**绝不升级成事实**。
>
> ⚠️ **一处日期口径差异（必须知道）**：`tilted_lidar_fidelity.md` 内部把 §I/§J 标为 "2026-10-09"、
> §K/§L/§M 标为 "2026-10-10"，但对应的提交（`a7497f0`、`2a5a8b6`、`18f2738`、`de3b78c`、`ef927e2`）
> 在 `git log` 里都是 **2026-10-07 / 2026-10-08**。**本文的时间线一律以 git 日期为准**，
> 引用时保留原文档的 `§` 编号（它比日期更稳定）。

---

# §0 一页速查（给"没时间"的人）

## §0.1 帧语义：谁发哪条边、谁是"假"的

**唯一合法的帧树（`tf_interface_contract.md` §三）**：

```
map ──(重定位：amcl | slam_toolbox-local | icp | gicp | small_gicp | cartographer)──► odom
      ──(里程计：LIO；或 lio:=cartographer 全包)──► base_link
      ──(URDF 固定关节，robot_state_publisher)──► livox_frame / imu_link / l2..l11 …
      ──(fake_vel_transform，20 Hz，哨兵云台机制)──► base_link_fake
```

| 帧 / 边 | 谁发（唯一发布者） | 语义要点 | 依据 |
|---|---|---|---|
| `map→odom` | **重定位模块之一**（互斥） | 全局锚定；**连续 25 帧被拒 ⇒ 停发**（宁可不发也不编造） | `gicp_divergence_and_jitter.md` §3.3 |
| `odom→base_link` | `lio_tf_adapter`（`lio:=fastlio\|pointlio`）／`small_point_lio` 自身／cartographer（`lio:=cartographer`） | **同一条边只能一个发布者**；Gazebo 真值 `publish_odom_tf=false` + remap 到 `/odom_ground_truth` | `tf_interface_contract.md` §一/§三、`issues_and_findings.md` #5 |
| `base_link→base_link_fake` | `fake_vel_transform`（20 Hz） | **不是脏帧**：哨兵云台载体；nav2 的 `robot_base_frame` 就是 `base_link_fake`，`/cmd_vel` 再旋到 `/cmd_vel_chassis` 下发。**T6"改回 base_link"已撤销** | `tf_interface_contract.md` §五 T6 说明 |
| `base_link→livox_frame` / `→imu_link` | `robot_state_publisher`（URDF 固定关节） | **矩阵值随 `robot:=` 槽位变**（3 档：`plugin`/`urdf`/`sensor`） | `tilted_lidar_fidelity.md` §C.1/§J.6.1 |
| `camera_init→body`、`aft_mapped*` | LIO 内部帧 | **不再泄漏到导航层**（T1–T5 已实施） | `tf_interface_contract.md` §五 |
| `/odom_ground_truth`（话题） | Gazebo `planar_move` 插件 | 真值里程计；**`.twist` 是指令回显，不是实测速度** | `tilted_lidar_fidelity.md` §M.9 ② |
| `map` 系原点 | — | **= 出生点相对系**（`(0,0,0)` = 出生点），不是场地绝对坐标 | `map_assets.md` §1、`continue_mapping.md` §0 |

### ★ "三个 `livox_frame`"——本工作期最贵的一课（`tilted_lidar_fidelity.md` §I.2）

同一个名字 `livox_frame` 在本仓被**三样不同的东西**共用，**混起来就会得出"物理错了"的错误结论**：

| # | 它是什么 | 由谁决定 | `plugin` 档 | `urdf` 档 | `sensor` 档 |
|---|---|---|---|---|---|
| ① | **物理传感器系**（实物安装姿态） | `body_to_livox` 关节 rpy | `0 0 0`（帧正） | `−30°`（真的斜） | `−30°`（真的斜） |
| ② | **点云数据实际表达的系** | 插件出点公式 `axis = sensor_rot·mount_rot_·ray`、原点 = 传感器原点 | **水平**（= 父 link 朝向） | **还是水平**（倾角没进数据！） | **斜 30°**（`<cloud_frame>sensor</cloud_frame>`） |
| ③ | **`header.frame_id` 字符串** | `<sensor name="livox_frame">` | `livox_frame` | `livox_frame` | `livox_frame` |

**判据（一眼分辨，不用猜）**：对点云做 RANSAC 地面平面拟合，看**法向与"它自称的那个帧"的 z 的夹角**：

* ≈0° ⇒ 帧与数据一致（`plugin` 实测 **1.054°**；`urdf` 也是 1.02°，**但那个帧自己斜了 30°** ⇒ 不自洽）；
* ≈30° ⇒ 数据真的在斜的传感器系里（`sensor` 实测 **29.092°**），此时**必须**经 TF 转到 `base_link` 才是重力对齐（实测 **1.035°**）。

**推论（§I.6 的 RViz 配方）**：`urdf` 档在 RViz 里"看着像平放扫到的东西被倾斜了"是**字面正确**的
—— 数据真的水平（两档传感器系点云刚体配准 **0.0004°** / 残差 p50 **0.074 mm**），斜的是那个帧；
**没有任何一个 `Fixed Frame` 能让"点云"和"车/代价图"同时看起来正**（这本身就是"帧与数据差 30°"的定义）。

## §0.2 两条代价图链（全局 `map` / 局部 `odom`）+ 3D→2D 的"高度信息死在哪一级"

```
/livox/lidar/pointcloud ──►[①地面分割 ground:=linefit|patchwork]──► /segmentation/{ground,obstacle}
        │                              │
        │                              └─[★判据层 rm_ground_traversability]（同一进程内，只降不升）
        │                                    · 坡度/台阶判据 ⇒ ground 降级成 obstacle
        │                                    · 自击掩膜 self_mask_*（默认关；robot11 开）
        │                                    · 近地剔除 obstacle_near_ground_m（默认 0；robot11 = 0.05）
        │                                    · 走廊连续量 ⇒ SlopeSpeedLimiter ⇒ /speed_limit
        │
        └──►[②pointcloud_to_laserscan]──► /scan（LaserScan：**没有 z**）
                                              │
                                              ├─►[③nav2 local_costmap/obstacle_layer.scan]（帧 = odom）
                                              │     projectLaser（z 强行置 0）→ TF 到代价图帧
                                              │     高度带 min/max_obstacle_height **在本帧里量**
                                              │
                                              ├─►[④nav2 local/global STVL（voxel）]（3D 点云直接进）
                                              │
                                              └─►[⑤inflation_layer]──► 代价梯度 ──► planner / controller

全局另一条腿：map/*.pgm|yaml ──►[⑥static_layer（map_server，先验图）]──► global_costmap
              PCD/*.pcd ──(pcd_to_nav2_map.py，离线)──► map/*.pgm|yaml（**2D 先验图从 3D 点云投影**）
```

| 环 | 在哪一节详述 | 本期改了什么（一句话） |
|---|---|---|
| ① 地面分割（linefit / patchwork） | 本文 §2.2.8；`ground_segmentation_slots.md` §3/§10/§11 | `th_dist 0.125→0.08`；接入"坡度/台阶"判据层；linefit 的 `gravity_aligned_frame` C++ bug 修掉 |
| ★ 判据层（坡度/台阶/自击/近地/限速） | 本文 §2.2.1/§2.2.7；`slope_speed_limiting.md` §1；`robot_models.md` §12.3 | 新增：连续量→速度上限（发 nav2 原生 `SpeedLimit`）、近地剔除、自击掩膜 |
| ② `pointcloud_to_laserscan` | 本文 §2.2.6；`ground_segmentation_slots.md` §11 | `max_height 0.1→1.0`（离地覆盖 **0.326 → 1.226 m**） |
| ③ `obstacle_layer.scan`（2D 链） | 本文 §2.2.1/§2.2.5；`tilted_lidar_fidelity.md` §K.1 | **结论是"不动"**：高度带在这条链上**没有逐点高度可判**（结构原因，见下） |
| ④ STVL（3D 链） | 本文 §2.2.9；`stvl_local_costmap.md` §2 | 局部代价图也上 STVL（槽位 `local_obstacle:=stvl`） |
| ⑤ `inflation_layer` | 本文 §2.2.2；`tilted_lidar_fidelity.md` §K.3 | robot11 槽位几何：`robot_radius 0.3565→0.300`（外接→内切）+ 膨胀 0.70/0.75→0.60/0.65 |
| ⑥ static_layer / 2D 先验 | 本文 §2.5.1；`map_assets.md` §3 | 默认 2D 先验提升到 `RMUC2026_v3` 会话产物（旧份留 `*.bak-20261007`） |

**★ 最反直觉、也最该记住的一条（`tilted_lidar_fidelity.md` §K.1）**：
**`obstacle_layer.scan` 的 `min/max_obstacle_height` 在这条 2D 链路上不是"障碍物高度闸"，而是"点云原点的代价图帧 z"的常量闸。**
因为 `LaserScan` 没有 z ⇒ `laser_geometry` 的 `projectLaser` 只能产出 `z ≡ 0` 的点，随后整体 TF 到代价图帧。
实测（robot11、静止）：**950 条波束的 odom z 全部落在 `[0.0564, 0.0815]`（跨度 25 mm），
落在这个带外的 = 0/950**，而同一跑 `odom→livox_frame` 的平移 z = **+0.064 m**（与 p50 吻合到 1 mm）。
⇒ **"按几何把高度带重新定基"在 2D 链路上没有可定的对象**（把闸门抬到任何值只是把整条平盘一起抬高）。
真正能用上"高度"的地方只有一处：**投影成 2D 之前**（判据层的"近地剔除"）。

## §0.3 参数归属表：每个高度/半径/距离参数**属于哪个帧**，写错会怎样

| 参数（键） | 住在哪个帧 | 默认 / robot11 取值 | 写错会怎样（**有实测的给数字**） |
|---|---|---|---|
| `linefit.sensor_height` | **传感器系**下"地面"的 z（= −离地高） | 0.226 / **0.2595** | 地面线整体偏移 h；robot11 用 0.226 ⇒ 偏 **3.35 cm**（`robot_models.md` §11.6） |
| `linefit.gravity_aligned_frame` | 分割前先把点云**只旋转**到这个帧 | `""` / `""`（robot11 仍空） | 本仓该键曾有 C++ bug：设 `base_link` ⇒ `/segmentation/ground` **恒 0 点**（已修，见 §2.2.4） |
| `p2l.min/max_height` | **点云自带帧**（`target_frame: ""` 时不建 TF） | −1.0 / **1.0**（原 0.1） | 旧值 ⇒ 离地 **>0.326 m** 的几何整条方位变 `inf` ⇒ 2D 图/代价图看不见护墙（`ground_segmentation_slots.md` §11） |
| `p2l.target_frame` | 是否把点云 TF 到别处再切片 | `""` / `""`（`sensor` 档 = `base_link`） | 一开就引入 TF/MessageFilter 故障面（2026-09-23 刻意避开）；`sensor` 档必须开 |
| `obstacle_layer.scan.min/max_obstacle_height` | **代价图帧**（local = `odom`） | 0.0 / 2.0（**本期不动**） | 按"障碍物高度"去调它 = 调错对象（§0.2 末）；`sensor` 档实测 **41.2%** 波束被丢 |
| `obstacle_layer.scan.obstacle_min_range` | 同帧，波束长度闸 | 0.1 | 挡掉 <0.1 m 自击波束（好）；同时"贴墙 0.1 m"也被挡（代价） |
| `obstacle_near_ground_m`（新增） | 判据层：`dz = z − 局部地面 g(x,y)` | 0.0（关） / **0.05** | 关 ⇒ 近场地面残留经 p2l 投成 lethal：车那格 **42（非 free）**、半径圆内 `≥99` **156**、free **239**；开 ⇒ **0 / 0 / 999**，到最近 lethal **0.531 → 1.534 m**（§K.2.3） |
| `self_mask_*`（新增） | **传感器系 + 重力对齐**：113 个 collision AABB ∪ 近场死区 | 关 / robot11 **开**（`r_xy ≤ 0.2416`、`z ≥ −0.2295`） | 关 ⇒ 近场自击格把"局部地面"钉成 **+0.000 m**（= 雷达自身高度）、"参考坡度"钉成 **0.0°**、静止也被压到地板 0.60 m/s（§12.3.3） |
| `local/global_costmap.robot_radius` | **它是 `inscribed_radius`**（本仓没有 footprint 多边形） | 0.22 / **0.300（内切）**（原 0.3565 外接） | 用外接值 ⇒ **全局**图上"车那格"被判 **99**、圆内 **0 格 free**（静止！）⇒ "车一开始就在膨胀团里"（§K.3.1） |
| `inflation_layer.inflation_radius` | 同帧；必须 ≥ `robot_radius` | local 0.5 / **0.60**；global 0.55 / **0.65** | 软带厚度决定"贴边"程度；`cost_scaling_factor` 与半径无关（本期不动） |
| `speed_limit_lookahead_m` | 传感器系的**朝向走廊**（帧 rpy=0 ⇒ +x = 车头） | 3.0（不改） | 再大也没用：平地最近只看得见 **1.78 m**（上坡面 ~2.6 m）（`slope_speed_limiting.md` §4） |
| LIO `extrinsic_T/R` | **点云实际表达的系**，不是"实物几何" | `[0,0,0.05]` / `I`（**三份配置一个字节没改**） | 光改外参补不了"点云不在 `livox_frame` 里"这个错（§I.8 原则 3） |
| `lio_tf_adapter.xyz/rpy` | `T_base_link←body` 的杆臂（`body` = 雷达/IMU 位置） | `[-0.12,0,-0.125]` / 按槽位算 | 丢了杆臂 ⇒ 原地转时"地图绕车画圆"（`issues_and_findings.md` #18） |
| 插件 `<tilt_rpy>` | 安装倾角（射线方向；点云仍表达在父 link 系） | 缺省 = 单位阵（其它模型逐字节不变） | 与关节 rpy **二选一**；两处都写 ⇒ 转 60° |
| 插件 `<cloud_frame>` | 点云表达在 `parent` 还是 `sensor` 系 | 缺省 `parent` / `sensor` 档 = `sensor` | 选 `sensor` 而下游不做重力对齐 ⇒ 地面在 odom/图里是 **30° 斜坡** |
| robot11 几何 | `livox_frame` z = **0.157028**；地面 = **−0.102499** ⇒ 雷达离地 **0.2595**；足印内切 **0.300** / 外接 **0.3565**；轮 r=0.058 | — | 这些数**是从 mesh 顶点 + 零位 FK 量出来的**（`robot_models.md` §9.4），不是抄的 |

## §0.4 现在能用的 / 还不能的（一句话一览）

| 能力 | 状态 | 证据 |
|---|---|---|
| 默认模型（`robot` 留空）全链路 | ✅ 能用；**本期没有改它的任何默认路径**（每个开关默认关 / 每处覆盖只在槽位文件里） | `robot_models.md` §11.2.1、§12.6、§J.2 |
| `robot:=robot11` 建图 / 感知 / 状态估计 | ✅ 雷达看得见（地面 **5689~6641 点/帧**）、`/odom` 10 Hz、RTF 0.32~0.46 | `robot_models.md` §11.2、§13.8 |
| `robot:=robot11` + GUI | ✅ 在"虚拟屏 + 软件 GL"下两档都活满窗口、都在画（`gzclient` 473 MiB、`gzserver` 3202 MiB **与视觉档位无关**） | §13.4.2 |
| `robot:=robot11` 短目标导航（≲0.4 m） | ✅ 能走到（`--goal-forward 0.5`：真值走 **0.2977 m**、残余 **0.1902 m** < 容差 0.25） | §L.7 |
| `robot:=robot11` 长目标 / 需要转向的目标 | ❌ **两个独立原因**：① 该出生点正前方 **0.42 m 就是实体障碍**（物理不可达）；② 反向 2.0 m 需要"先转头再走" ⇒ 依赖角速度通道，修前只执行 **0.55%** | §L.0、§L.2、§L.5 |
| 角速度通道（修后） | ✅ 整栈盲发 `wz=1.0`×12 s：真值 **+35.42°**（修前 +3.79°，**9.3×**）、执行率 **0.55% → 5.15%**、60 s 自由漂移 **+8.20° → +0.006°** | §M.7 |
| `robot11_mount:=sensor`（物理斜装 + 账也对） | ⚠️ 帧↔数据自洽（29.028° vs 1.004°）、下游重力对齐生效；**但代价图那一层仍坏**（odom 斜 30° ⇒ 41.2% 波束被高度带丢） | §J.7 第 3 条 |
| 坡度/台阶前瞻限速 | ⚠️ 有效但**没根治**：首击 **110.8 → 50.8 m/s²**（目标 ≲30 未达）；提速档把"0/4 到点"变成 **2/2 到点** | `slope_speed_limiting.md` §0/§3、`lio_divergence_no_impact.md` §3.3 |
| 目标朝向 | ❌ **本仓 nav2 不检查朝向**：`goal_checker_plugins` = `PositionGoalChecker`（只声明 `xy_goal_tolerance`）⇒ "原地转 180°"会**立即 SUCCEEDED**（`max|wz| = 0.0`、Δyaw 0.005°） | §M.7、§M.9 ① |
| 2D 先验图的"坡道面" | ❌ 仍然看不见（这是 2D 先验的固有上限：全场台阶边沿覆盖率只有 **35.98%~54.42%**，判据是 80%） | `map_assets.md` §3.1、`traversability_plan.md` §9.1 |
| 落空/负障碍、净空（限高） | ❌ 没做（`traversability_plan.md` §9 明确登记；抬到 1.226 m 之后"头顶结构"会成为假障碍，换场地要重估） | `traversability_plan.md` §9 |

## §0.5 本期最该记住的五句话

1. **"CAD 网格能不能进仿真"要分两问**：**视觉**可以（但要按预算抽稀 + 量化验证），**碰撞绝对不行**（一接触就掉到 0.2 倍实时、常驻 +587 MB）。
2. **帧是一等公民**：同一份数据配上错误的 `frame_id`，会让"物理正确的模型"在 RViz/nav2 里看起来全错；**判据要用矩阵/一致性，不要用 rpy 分量**。
3. **"看起来像"的观感往往是"刚性旋转"**：地面变 30° 斜坡、环状图案不变 = 帧的问题；一侧彻底没有地面、最近地面环从 0.31 m 变成 1.96 m = 物理倾角。两件事可以**同时**存在。
4. **先量后改**：本期每一个"修"前面都有一次判决性测量；被否掉的候选（紧门限、1000 Hz 插件、角速度限幅、只放松回环阈值、footprint 多边形…）都留下了数字。
5. **默认路径逐字节不变 + 一键回退**：所有改动都做成"槽位/开关/覆盖文件"，默认关；每个改动都有一行回退配方（§6.3）。

---

# §1 时间线：阶段、关键提交、以及"我们推翻过哪些结论"

## §1.1 阶段总览（按 git 作者日期）

| 阶段 | 日期 | 主题 | 关键提交（示例） | 产出文档 |
|---|---|---|---|---|
| **P0 场地与建图地基** | 10-05 | `world:=RMUC2026`（STL→world/2D/PCD）、`lio:=small_point_lio` 打通与调参、定位槽位（beluga / icp / gicp / small_gicp）、nav2 槽位（`nav:=mppi`、`planner:=smac2d`）、构建口径修复 | `7c0006b` `280888e` `eb7b811` `98baa55` `2ff42d4` `21fc301` `2c42339` `6ae2ada` | `worlds.md`、`lio_slots.md`、`localization_slots.md`、`algorithm_matrix.md`、`build_optimization.md` |
| **P1 重定位 / 时间戳 / 建图稳健性** | 10-05~10-06 | `/initialpose` 饥饿修复、1 ns 戳截断修复、2D 先验改从点云投影、`ground` 槽位、可通行性设计计划、LIO 漂移分段诊断、slam_toolbox 节点加密与丢帧裁决、续建/存档/守卫 | `c7bcd6d` `a281636` `7afbe01` `1a9ce8c` `fc3382d` `e866cfa` `5cb93cb` `0a9c5b5` `10f2a84` | `gicp_initialpose_latency.md`、`timestamp_construction_audit.md`、`mapping_2d_from_cloud.md`、`ground_segmentation_slots.md`、`traversability_plan.md`、`lio_drift_diagnosis.md`、`slam_toolbox_tuning.md`、`continue_mapping.md` |
| **P2 模型槽位（`robot:=`）** | 10-07 | opt-in 槽位机制、用户那份哨兵 URDF（`hzmirm`）逐字接入 + 补件、"0.8 m + −7.22° ⇒ 6.32 m 近场盲环"实测 | `d3940b5` `d6232ea` `a2c6349` `712741f` | `robot_models.md` §1–§8、`params_ownership_checklist.md` |
| **P3 `robot11` Phase 1–2** | 10-07 | 12 个 STL 打包与逐 mesh 量测、`base_link` 碰撞策略（ODE 代价实测）、槽位接入、**30° 是 roll 还是 pitch 的实测判定**、"雷达装在底盘凹槽里" | `f361f00` `599fc09` `0142bd9` | `robot_models.md` §9–§10 |
| **P4 Phase 3：让雷达看得见** | 10-07 | A 方案（碰撞挖视锥 = 76 个 box）、根 link 改名 `body→base_link`、插件新增 `<tilt_rpy>`、斜置感知链槽位化、nav2 几何覆盖层 | `d4e8232` `82e4946` `a1d9c15` `23c1dc8` | `robot_models.md` §11 |
| **P5 Phase 4–5：GUI / 自击 / 视觉件** | 10-07 | 自击掩膜、横幅改成"运行时读生效值"、`robot11_visual` 档位、**QEM 视觉 LOD（2,821,320 → 163,995 面）** | `1ca09eb` `cf60b84` `646277f` `4ec728e` | `robot_models.md` §12–§13 |
| **P6 斜装逐帧追查（§A–§I）** | 10-07~10-08 | 逐帧点账本（"有没有被裁"的定论）、代价图/膨胀半径归因、`robot11_mount:=plugin\|urdf` 开关、**三个 `livox_frame` 的判决**（H1/H2/H3）、Gazebo 侧 mesh 姿态直接读数、`model://` 解析自动化、gzclient 卡住取证 | `6c9ae96` `b7d625b` `a7497f0` `527fc10` `e2bcf3d` `96974db` `3b48c7c` | `tilted_lidar_fidelity.md` §A–§I、`gazebo_gui_troubleshooting.md`、`robot_models.md` §14–§15 |
| **P7 两个 LIO bug + `sensor` 档（§J）** | 10-08 | bug ②（`/cloud_registered` 多乘一次 TF）、bug ③（`odom→base_link` 用共轭发姿态）、新增 `robot11_mount:=sensor`（物理斜装 + 账也对） | `2a5a8b6` `1116b05` `7542973` | `tilted_lidar_fidelity.md` §J |
| **P8 代价图侧收尾（§K）** | 10-08 | 近地剔除（`obstacle_near_ground_m`）、`robot_radius` 外接→内切、插件 `point = range·axis` 漏起点修复、高度带"为什么不动" | `360d493` `1c89473` `1403f42` `18f2738` | `tilted_lidar_fidelity.md` §K |
| **P9 长目标归因 + 角速度修复（§L/§M）** | 10-08 | 六个候选逐条判决（根因 = 实体障碍 0.42 m）、可达性前置判据/闸门、**底盘塌成单刚体（角速度 1.03% → 16.2%）** | `94f0d6b` `de3b78c` `ef927e2` `9de6b72` `d7c4da0` `d3034aa` | `tilted_lidar_fidelity.md` §L–§M |
| **P10 撞击 / 贴边 / 限速** | 10-06~10-07 | 贴膨胀边→撞击→定位失效的链条归因、`w_smooth 0.4→0.0`、实时坡度/台阶判据、前瞻限速（含提速档）、"没撞击却飘"归因 | `1811149` `4a7996c` `6f0c27a` `ca89b69` `4adf0d8` `dad39c5` | `path_clearance_and_contact.md`、`slope_speed_limiting.md`、`lio_divergence_no_impact.md` |
| **P11 研究（未落地）** | 10-08 | 纯 C++ 规划库（`tdt-nav-kit`）接进 nav2 的三种形态、契约、效率账、落地坑 | `d4965bd` | `nav2_vs_custom_planner.md` |
| **P12 资产提升与规范** | 10-07 | 地图/先验资产规范、逐个清单、2D/3D 默认对提升到 `RMUC2026_v3`、退役资产进 `attic/` | `95846c3` `7f9e354` `2192024` `c8d65f9` | `map_assets.md`、`continue_mapping.md` §补 |

## §1.2 ★ 推翻过的结论："当时以为 → 后来的测量 → 更正"

> 这张表是本期最有价值的部分之一：**每一次更正后面都跟着一次判决性测量**。引用的 `§` 是原文档章节。

| # | 当时以为（错误认知） | 判决性测量（数字） | 更正后的结论 | 证据 |
|---|---|---|---|---|
| 1 | "点云少了很多，肯定被下游裁减了" | 一帧 30000 条射线里只有 **11960 条有回波（39.9%）**；`/livox/lidar`、`/livox/lidar/pointcloud`、`/cloud_registered` 每帧点数 **11960 / 11960 / 11968** | **没有任何一级在裁**；规模由"有多少条射线真的打到东西"决定。少掉的是 Phase 2 的**假自击点**（75.5% → 28.5%） | `tilted_lidar_fidelity.md` §B |
| 2 | "URDF 里写的是 `rpy=0 -0.5236 0`（pitch），所以是 pitch" | MID-360 方位 360° ⇒ roll/pitch 的**射线方向集合完全相同**（仰角谱、地面环半径、盲区都一样）；唯一判据 = "最朝下的那一束指向 body 的哪个方位"（roll→±y 实测 75°/105°；pitch→±x 实测 165°/195°） | **仿真不能替用户决定哪个是真的**；默认取 **roll**（SolidWorks CSV 机器生成、15 位有效数字；URDF 那行只有 5 位）；用户后来**按实物确认 = roll** | `robot_models.md` §9.5/§10.5；`Fidelity` §I.3 |
| 3 | "近场点 100% 落在云台 `l10/l11` 上（中位距离 0.14 m）" | Phase 4 逐点复核：近场点**中位半径只有 0.021 m**（p95 0.048、max 0.121）；113 个 collision AABB 只"命中" **3 个点** | 真因是**插件 `point = range·axis` 漏了射线起点**（`start = minDist·axis`，minDist = 0.1 m）⇒ 每个点朝传感器内移 **0.1 m**；独立证据 = 地面点 z 随距离单调变化（r 0.25–0.35 → −0.208；r 2–4 → −0.253；几何真值 −0.2595） | `robot_models.md` §12.3.1；`Fidelity` §K.4.3 |
| 4 | "`hzmirm` 的地面分割全废 = 几何原因（0.8 m 高 + 下视 7.22° ⇒ 6.3 m 盲环）" | 几何那条**仍然成立**；但该槽位的 YAML 同时开了 `gravity_aligned_frame: "base_link"` ⇒ **`/segmentation/ground` 恒为 0** 里有一部分是这个 **C++ bug** | 归因要按"**几何 + bug**"两条并列重读 | `robot_models.md` §11.4 |
| 5 | "LIO 的 `odom` 帧被转了 30°（缺一条 LiDAR→IMU 外参，H1）" | 把 `/cloud_registered` 按 `T(base_link←livox_frame)` **反变换**回去：地面 **30.970° → 1.041°**；从发布的 `odom→base_link` **反解 LIO 状态** `R_ol` = `[0.086, 0.405, 9.105]°`（只有 yaw） | **odom 没有被转 30°，它水平到 0.1~0.4°**；30° 是**发布环节多转的一次**（bug ②） | `Fidelity` §I.1/§I.5 |
| 6 | "地图跟着车转 = 位姿图回环门槛太松，收紧就行" | 收紧后收尾日志变成 `Score histogram: Count: 0`（**一条回环都没命中**）⇒ 位姿图完全没有全局锚定 | **方向错了**；真因是 ① 插件**混用两套时间基**（`header.stamp` 仿真钟 / 逐点 `offset_time` 墙钟）② 先验给错源（喂 LIO 自己的 `/odom`：同源 + 晚到）⇒ 换成**独立底盘里程计** | `issues_and_findings.md` #19/#20/#21 |
| 7 | "本仓底盘对 yaw 有巨大死区：要 1.9 rad/s 得给 6 rad/s" | 用**同一行内的"指令/真值"配对**重测（n=75~109、符号一致率 1.00）：指令 0.34→实测 0.36、0.65→0.67、1.07→1.11、**2.83→2.83**（原地） | **≈1:1，底盘听得懂 yaw 指令**；旧表作废，**"要 1.9 得给 6"不要再引用** | `lio_drift_diagnosis.md` §2.3 的 2026-10-07 更正；`lio_divergence_no_impact.md` §2 |
| 8 | "`/scan` 丢帧是刚打开的**回环**（节点加密）造成的" | 同一份**完美里程计**（ATE = 0.000 m），只把 TF 晚发 **130 ms** ⇒ 丢帧 **0 → 62（4.26%）**；六格里**没有一条丢帧落在回环 ±2 s 内**（随机基线 1.5%） | 回环是**放大器不是开关**；直接因 = "这一帧的 `odom→base_link` 比它自己晚到、且晚过下一条 `/scan` 的到达间隔" | `slam_drops_loopclosure_odom.md` §0 |
| 9 | "把 `scan_queue_size` 调大就能消掉丢帧日志" | 队列 `2` 把 `queue is full` 从 **62 → 0**，但同一跑出现 **70 条另一个原因**的丢帧（`timestamp earlier than all the data in the transform cache`），位姿图节点数 **58 → 35** | **丢帧总量没减少，只是换了个名字** ⇒ 不推荐；正解是修根因（LIO 戳对齐） | 同上 §0 末 |
| 10 | "车 90 s 没动 = 地图没建好 / 规划器有问题" | 把 nav2 **完全摘掉**、只给底盘插件固定 `vx`：robot11 只能前进 **0.4199 m**（请求 6.25 m），默认模型 **0.5402 m** | 根因是**场地里 0.42 m 处有实体障碍** ⇒ `--goal-forward 2.0` **物理上不可达**；控制器侧那些日志都是**结果** | `Fidelity` §L.0/§L.2 |
| 11 | "短目标能走 ⇒ 长目标只是慢/更长" | `--goal-forward 0.5` 真值只走 **0.2977 m**、残余 **0.1902 m** < `xy_goal_tolerance 0.25` ⇒ `Goal succeeded` 是**容差成功**；且 §K.5 表里"0 条 `collision ahead`"**只对短目标成立**（长目标是 **135 条**） | 两件事要分开：短目标是"落在容差里"，长目标是"目标在障碍后面（不可达）"；**口径更正**已写进 §L.1.3 | `Fidelity` §L.1.3/§L.7 |
| 12 | "`gt_yaw_delta_deg` 报 473.416 / 251.651 度" | 真值其实是 **8.26° / 4.39°** | **工具 bug**：`_xy()` 的 yaw 已是度，代码又 `math.degrees()` 了一次 ⇒ 全部多乘 **57.2958**；已修，旧读数不要再引用 | `339e205`；`Fidelity` §L.1.3 |
| 13 | "robot11 角速度只执行 1.03% = 单位/缩放/mixing bug" | 源码级：`planar_move` 的 `OnUpdate` 就是 `model_->SetLinearVel/SetAngularVel`，而 `Model.cc:746-771` 对**每个 link 各设一遍同一个 `(v,ω)`**；同一插件在"极简单 link box"上能做到 **99.9%** | 通道本身没坏；根因是"**驱动器与模型结构不自洽**"（多刚体 + 关节树） | `Fidelity` §M.2/§M.3 |
| 14 | "把插件 `update_rate` 提到 1000 Hz 就能修角速度" | 角速度确实 1.03% → **7.8%**（台架 +53.80°），**但**纯物理正前方盲推从 **0.4077 m → 2.5652 m**、底盘 z 从 **0.1522 → 0.4005 m**（**爬上了实体障碍**），60 s 漂移 0.308 → 0.297°/s（没解决） | **不采用**（这是"必须做反向验收"的教科书例子） | `Fidelity` §M.5 |
| 15 | "抽稀视觉件能省下 GB 级 GUI 内存"（Phase 4：rviz2 **1381 → 243 MiB**） | Phase 5 在**同一脚本/世界/flags/虚拟屏**下重跑：`full` 314/322 MiB、`decimated` **314 MiB**，逐秒曲线形状**几乎完全相同** | **本相位没有测出这个收益**；三条可能原因已列但**没有定论**；抽稀**确凿**省下的是三角形（17.2×）、磁盘（141.07 → 8.20 MB）、网格解析/上传/绘制的工作量 | `robot_models.md` §13.4.3 |
| 16 | "`gzclient` 被 `exit code -9` = 内存不够（OOM）" | 用户机 `MemTotal 31.1 GiB`、内核日志**无 OOM 记录**、会话 cgroup `memory.events` 的 **`oom_kill = 0`**；而"GUI 卡住不出来"另有真因：`model://` 解析不到 ⇒ 回落在线模型库**同步阻塞**（实测 **48.03 s**，且是被 Ctrl-C 打断的） | **`-9` 是外部 SIGKILL，发送者无法指认**（无 `auditd`）；⭐ **用户 2026-10-10 回忆是用户自己手动 `pkill -9`（"因为太卡了或者直接卡住了"）**，另一候选才是本仓 bench 脚本的全机 `pkill`；卡住 = mesh 解析 + 在线模型库 | `gazebo_gui_troubleshooting.md` §0 / §4.2 |
| 17 | "回退档**逐字节相同**" | 精确核对的结论是：**模型体（去掉注释后）逐字节相同**（两份都是 35025 B / 35410 B，`diff` 为空），整份文件多出的是文件头说明 | 表述改成"模型体（去注释）逐字节相同" | `d3034aa` |
| 18 | "`/get_entity_state` 能拿到 Gazebo 眼里的 link 姿态，只是被 grep 过滤掉了" | `ros2 service list` 里 **0** 个 `/get_entity_state`；本仓 gzserver 只加载了 `libgazebo_ros_{init,factory,force_system}.so`，**没有** `libgazebo_ros_api_plugin.so` | "那个服务根本不存在"；改用 `gz model -m robot -i`（Gazebo transport，不经 ROS）拿到**直接读数** | `Fidelity` §H.4 |
| 19 | "`/odom_ground_truth.twist` 是实测速度" | 真值 yaw 12 s 只转了 **7.09°**，而该字段中位 = **57.296°/s = 1.0 rad/s**（= 刚设进去的指令值；`OnUpdate` 先 `SetAngularVel` 再在同一回调里读 `WorldAngularVel`） | **它是指令回显**；要量"到底转了多少"只能用**位姿差** | `Fidelity` §M.9 ② |
| 20 | "本仓 nav2 会检查目标朝向" | `goal_checker_plugins: ["general_goal_checker"]` = `PositionGoalChecker`（只声明 `xy_goal_tolerance`；1.1.20 里没有 `yaw_goal_tolerance`）⇒ "原地转 180°"**立即 SUCCEEDED**（`max|wz| = 0.0`、Δyaw 0.005°） | **不查朝向**；要真判转向得换 `SimpleGoalChecker`（本期**没改**，参数在别的任务目录） | `Fidelity` §M.9 ① |

---

# §2 逐项改动详述（主体）

## §2.0 读法与统一模板

每条改动都按同一个模板写（缺项写"文档未给"）：

> **现象**（用户看到 / 日志看到）→ **原因**（机理，必要时给公式 / `file:line`）→
> **过程**（怎么查的：**判决性测量**、被否掉的候选）→ **改动**（文件 / 参数 / 生成物 / 提交）→
> **原理**（为什么这样修是对的、通用教训）→ **效果**（before → after 的**实测数字**）→
> **回退**（一键怎么做）→ **证据**（原文档 `§` / 提交）

分层顺序：**模型层 → 感知/代价图层 → LIO 层 → 导航/限速层 → 资产/建图层 → 工具与方法论 → 研究结论**。

## §2.1 模型层（CAD → 仿真件、碰撞/视觉分离、model://、斜装、自击、单刚体）

### §2.1.1 CAD 网格**不能**直接当仿真件（碰撞与视觉要分开处理）

| 项 | 内容 |
|---|---|
| **现象** | 用户：「机器人模型放不进去仿真有啥意义」；带 GUI 时 `gzclient` 消失 / rviz2 黑屏 5 s 退不掉；车"在 Gazebo 里没有" |
| **原因** | 上游给的 12 个 STL 是**给人看/给机床用**的档：整车 **2,821,320 面 / 141.07 MB**，其中 `base_link.STL` 一项 **2,078,226 面 / 103.91 MB**（99.1 MiB）。放进 ODE 的 trimesh 里，**ray-vs-trimesh 求交代价是 O(三角面数)**，碰撞每帧都要做；视觉件每帧都要渲染。**这是"用途不匹配"，不是"慢一点"** |
| **过程（判决性测量）** | Gazebo Classic 实测（极简世界、ODE quick/50、`max_step_size 1 ms`、单模型、无头隔离）：**A 原网格**：悬空 RTF 1.002、峰值 RSS **760 MB**；**A2 原网格 + 接触**：RTF **0.200**（最低 **0.04**）、窗口内只推进 **0.49 s** 仿真时间；**B 4 个 box**：1.002 / **174 MB**；**C 抽稀 3000 面**：1.002 / 173 MB；**E 无底盘基线**：172 MB ⇒ **常驻 +587 MB 与有没有接触无关** |
| **改动** | ① 视觉：保留原网格（`<visual>` 零损失）→ Phase 5 换成**专用 QEM 视觉 LOD**（见 §2.1.2）；② 碰撞：`base_link` = **4 个 DP box**（`--band 0.005 --merge-tol 0.012` 的最优 4 段）；轮 = `cylinder(r=0.058, l=0.0452)`；其余 mesh → 包围盒 box。产物 `inventory/collision_assets.json`＋`meshes/generated/*_collision.stl`（备选） |
| **原理** | **"视觉/碰撞/感知"三件是三种不同用途的几何，允许而且应该不同**（SDF 规范明文："simpler collision models are often used to reduce computation time"；Gazebo 官方："simple shape **which is preferred**"）【来源】。反过来：**别用"稳态悬空 RTF 正常"当作"原网格能用"的证据**（A 的 1.002 是因为底盘离地 54.6 mm、压根没有接触对被求值） |
| **效果** | 4 box 的客观保真度：**覆盖率 1.000**（1605/1605 体素，真网格表面没有一个点在 box 外 ⇒ 不漏撞）、box 总体积 **0.065121 m³**（只比凸包胖 **9.4%**）、幻影体积占比 **16.36%**（pitch 30 mm）、最大内间隙 **69.3 mm**。RSS 760 → **174 MB**；接触工况 RTF 0.200 → **1.002** |
| **回退** | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --body-collision boxes`（回 Phase 2 的实心 4 box）／`--body-collision mesh`（抽稀网格碰撞，**未验**） |
| **证据** | `robot_models.md` §9.3（表）、§11.3 第 1 项、§13.2.1【来源】；提交 `f361f00` `d4e8232` |

### §2.1.2 视觉件的 QEM 抽稀：**2,821,320 → 163,995 面**（Phase 5）

| 项 | 内容 |
|---|---|
| **现象** | 同上（GUI 卡）；Phase 4 的 `decimated` 档当时指向的是**碰撞档抽稀件**（`l11` 只有 1,200 面 ⇒ 云台/发射机构糊成一团） |
| **原因** | 原始面数不是"每帧渲染"的档；标准做法 = **按预算抽稀 + 量化验证**（不是"换个工具就信"） |
| **过程（判决性测量）** | ① 先查标准做法（带 URL）：Blender/游戏与 Gazebo 侧"每 link ≤50k、整车 ≤200k"、`<50k`/`≤100k` 两篇独立来源取严；② 抽稀工具逐个试：**MeshLab `meshlabserver` 在本机无 GL，`GLEW initialization failed` 直接死**；`trimesh.simplify_quadric_decimation` 缺 `fast_simplification`；`open3d`/`pymeshlab`/Blender 都没装 ⇒ **VTK `vtkQuadricDecimation`（pyvista 0.46.4）采用**；③ 用"边界边不增长 + 三视剪影 IoU + 单向表面误差"三条把它**量出来** |
| **改动** | `tools/scripts/regress/robot11_decimate_visuals.py`（生成 + 验证 + 清单）；产物 `meshes/decimated/*.stl`（12 个 / **8.20 MB**）+ `inventory/visual_decimation.json`；`robot11_visual:=decimated`（**默认**）\| `full`；xacro 由**生成器拥有** |
| **原理** | 三条通用教训：① **抽稀要按"预算 + 容差"，不是按"工具身份"取信**；② 误差口径要**说清偏低/偏高**（本工具默认采样口径**偏低 ≤0.22 mm**，是设计如此：宁可不虚报）；③ **外观检查 ≠ 感知检查**（Gazebo 射线传感器走 ODE、**只与 `<collision>` 求交** ⇒ 视觉件抽稀**不可能**改变点云；"雷达凹槽还在不在"要单独量） |
| **效果** | 面数 **2,821,320 → 163,995（−94.2%）**、磁盘 **141.07 → 8.20 MB**；逐 mesh 全部过容差（最差包围盒差 **0.297 mm**、最差 p99 **0.18 mm**、最差剪影 IoU **0.9919**、12/12 过）；`base_link` 自击率 `t<0.12 m`：原件 0.3833 → 抽稀 **0.3810**（Δ**−0.23 pp**，凹槽没被抹平） |
| **回退** | `robot11_visual:=full`（回上游原始 STL）；或删 `meshes/decimated/`（但默认档会加载失败） |
| **证据** | `robot_models.md` §13.1/§13.3/§13.5；提交 `646277f` `4ec728e` `bf0353b` `15175ea` |

### §2.1.3 碰撞 **76 个 box 的由来**（让射线"挖"出视锥）+ 根 link 改名

| 项 | 内容 |
|---|---|
| **现象** | Phase 2 接入后：**75.5% 的点是 `r<0.12 m` 的近距回波**、`/segmentation/ground` 只剩 **529 点/帧**（默认模型 2626）、**LIO 连 `/odom` 都没发出来**（有发布者、0 条消息） |
| **原因** | 两条独立的根因：① `body` 的 4 个 DP box 是**实心包络**，而雷达原点 z=0.157 **正好在第 4 个 box 内部**（box 顶 0.213）⇒ 每条朝下的射线**先打盒内壁**；② 模型根 link 叫 **`body`**，而全链路契约帧名是 `base_link`（`small_point_lio` 源码硬编码 `lookupTransform(lidar_frame,"base_link")`）⇒ **TF 断成两棵树**，日志逐帧 `Failed to lookup transform from base_link to livox_frame: ... not part of the same tree` |
| **过程（判决性测量）** | ① 离线 30000 条与 Gazebo 同源射线求交：**80.5% 落在 <0.12 m、看见地面 0%**（与 Gazebo 实测 75.5% 同量级，差 = Gazebo 丢掉 <0.1 m 回波）；② `/odom` 消息数：Phase 2 = **0**、A 方案 = **188** |
| **改动** | ① `body` 碰撞 = DP box **减去雷达视锥**（锥半角 **100.22°**、半径 **0.5 m**、内清空 0.05 m）后分解成 **76 个 box**；云台 `l10/l11` = 7/22 个细 box；② 生成时把根 link 改名 `body → base_link`（**上游 SolidWorks CSV 里本来就叫 base_link**） |
| **原理** | **"碰撞体的形状"要服从"传感器必须看得见"这条功能约束**，而不只是"包住几何"；**最小包围 ≠ 可用碰撞**。改名这条是"**契约帧名是硬编码的，模型必须服从契约**" |
| **效果** | `robot11` 从"看不见"变成：地面 **5637 点/帧**（默认模型 2739）、点/帧 11951、`/odom` 188 条、`/scan` >4 m 波束 **≈81/帧**（Phase 2 = 0）；自击 75.5% → **29.0%**（剩下的 29% 是**物理真实**的云台回波，中位距离 0.14 m，真车 360° 雷达也会照到） |
| **回退** | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --body-collision boxes`（回实心 4 box ⇒ 立刻变回"看不见"） |
| **证据** | `robot_models.md` §11.1/§11.2/§11.3 第 1–2、9 项；提交 `d4e8232` |

### §2.1.4 `model://` 解析 + "等在线模型库"的同步阻塞（48~100 s 卡顿）

| 项 | 内容 |
|---|---|
| **现象** | 用户：「gazebo 加载不出来吗，rviz 倒是挺好的」；日志 `[Msg] Waiting for model database update to complete...` 之后长时间无输出，随后 `[Err] [Visual.cc:2956] No mesh specified`（车在 Gazebo 里**没有视觉**） |
| **原因** | URDF 的 `package://robot11/meshes/decimated/<link>.stl` 会被 sdformat 在 URDF→SDF 时改写成 **`model://robot11/...`**，而 gazebo 的 `model://` 解析根只有 `$HOME/.gazebo/models` + `GAZEBO_MODEL_PATH`；**解析不到时 `SystemPaths::FindFileURI()` 无条件回落到在线模型库 `http://models.gazebosim.org/` 并同步阻塞**（实测 stall **48.03 / 76.34 / 99.67 s**，不设上限） |
| **过程（判决性测量）** | ① 用户 19:50 那次日志：`t_cloud_stop − t_click` 不是问题，但"等模型库"实测 **48.03 s 且是被 Ctrl-C 打断的**（⇒ 再等也不会好）；② 先怀疑网络 ⇒ **受控实测把代理做成黑洞**，结论与直觉相反：**不是网络慢，是它一定会等**；③ 排除 OOM（见 §1.2 第 16 条） |
| **改动** | **两条腿**：**(a) launch 侧** `_gazebo_model_path_setup()`（`OpaqueFunction` + `AppendEnvironmentVariable`，排在 include gzserver/gzclient **之前**）⇒ 只对 `robot:=robot11` 执行；**(b) 包侧** `package.xml` 的 `<export><gazebo_ros gazebo_model_path="${prefix}/.."/></export>` ⇒ 任何 gazebo 入口都吃得到。两者**各自单独就够** |
| **原理** | ① **"解析不到就上网找"是 gazebo 的设计**，所以"本地一定解析得到"是使用方责任；② 修法要**分作用域**：launch 侧对其它槽位 0 动作 0 日志，包侧是**全局 env 增量**（⚠️ 这是本期唯一的"不是零改动"处） |
| **效果** | 改造前 vs 现在（同一条命令、无任何手工环境变量）：「等在线模型库」**出现、stall 51.64 s → 没有**；`No mesh specified` **34 → 0**；gzclient 事件数 **275 → 27**；轻量入口的 `rchar` **4.21 → 12.47 MB（+8.26 MB ≈ 12 个 mesh 的 8.20 MB）** —— 这条"+8.26 MB 且 fd 扫描抓到 `.stl`"才是"解析链真的通了"的正证据 |
| **回退** | 删 launch 里 `ld.add_action(OpaqueFunction(function=_gazebo_model_path_setup))`（启动侧）／删 `package.xml` 那一行并重建（包侧）；手工 workaround 仍可用：`GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PWD/install/robot11/share" ros2 launch …` |
| **证据** | `gazebo_gui_troubleshooting.md` §0/§3/§5.1/§5.1.6/§6.2–§6.4；`robot_models.md` §14；提交 `e2bcf3d` `96974db` `3b48c7c` `c7f57d8` |

### §2.1.5 雷达"斜 30°"的三种档位：`<tilt_rpy>` / `<cloud_frame>` / `robot11_mount`

| 项 | 内容 |
|---|---|
| **现象** | 用户：「雷达给我按我给你的放好…我要看到倾斜放置最原始的效果…点云少了很多…urdf/雷达/点云图像都是平放置的很奇怪」；后来（§I）：「点云看着像平放扫到的东西倾斜了，不是倾斜放置扫描到的东西」 |
| **原因** | **"倾角只能记在一个地方"**：记在**插件**（帧正、点云正、射线斜 —— 自洽但画不出斜的 mesh）**或**记在**关节**（帧斜、mesh 斜、射线斜 —— 但点云仍留在水平系）。两档的**世界射线逐条相同**（逐点差 ≤5×10⁻⁵ m），因为插件出点公式 `axis = sensor_rot·mount_rot_·ray` 里 `sensor_rot` 会跟着 link 姿态走 ⇒ 两种乘法乘出**同一个矩阵** |
| **过程（判决性测量）** | **H1**（odom 被转 30°）：反变换 `/cloud_registered` ⇒ **30.970° → 1.041°**；反解 `R_ol` 只有 yaw ⇒ **不成立**。**H2**（只斜了 label）：两档传感器系点云刚体配准 **0.0004°**、残差 p50 **0.074 mm**，而地面最近环 **0.3146 m** / 离地 0.2482 m ⇒ 最陡下俯 **38.27°**（水平安装不可能 < 1.96 m）⇒ **前半成立、后半不成立**。**H3**（右侧盲区 = roll 应有几何）：右侧两扇区 3586 点里 **0 个地面点**，最低仰角 **+21.89°/+19.67°**（解析下界 +19.11°）；左侧 2694 个地面点、最低仰角 **−36.87°**（下界 −37.22°）⇒ **成立，两档都是这样** |
| **改动** | ① 插件新增 SDF 参数 **`<tilt_rpy>`**（缺省单位阵 ⇒ 其它模型逐字节不变，`82e4946`）；② `robot11_mount:=plugin\|urdf`（`b7d625b`）；③ 插件新增 **`<cloud_frame>`**（缺省 `parent`；`sensor` ⇒ `axis = mount_rot·ray`）＋第三档 `robot11_mount:=sensor`（`1116b05`，`sensor` 档的 launch 只给它加 `gravity_aligned_frame: base_link` / `target_frame: base_link`） |
| **原理** | **五条原则**（`Fidelity` §I.8）：① 先分清**三个 `livox_frame`**（物理系/数据表达系/`frame_id` 字符串）；② 倾角**只能记一处**，然后把"点云表达 + `frame_id`"一起改到自洽；③ **外参要按"点云实际表达的系"给，不是按实物几何给**；④ **`odom` 不保证重力对齐**（`fix_gravity_direction: true` + 初始 `R=I` ⇒ odom ≡ 初始化那一刻的身体系）；⑤ **每个"高度"参数都属于某个具体的帧** |
| **效果** | 三档并排（静止、同世界/出生点）：`原始云地面法向 vs 自己的 frame_id` = **0.957° / 1.023° / 29.092°**；`vs base_link`（账对不对）= **0.957° / 30.920° / 1.035°**；`/scan` 波束在 odom 里超出 `[0,2] m` 的比例 = **0.000 / 0.000 / 0.404**；自击掩膜命中率三档都 **≈29.2%**（**不用重烘**：掩膜作用在旋转后的 `cloud_proc` 上，那份坐标两档逐点相同）；local lethal = **461 / 404 / 518**。⇒ **只有 `sensor` 档"数据在斜的传感器系里 + 账也对"两条同时成立** |
| **回退** | 去掉 `robot11_mount:=…`（默认 `plugin`）；只退"点云表达在传感器系" ⇒ 去掉 xacro 里的 `<cloud_frame>`；整套 ⇒ `git revert <对应 commit>` |
| **证据** | `tilted_lidar_fidelity.md` §A/§C/§I/§J；`robot_models.md` §15；提交 `82e4946` `b7d625b` `1116b05` `6c9ae96` `a7497f0` `527fc10` |

> ⚠️ **`urdf` 档坏在哪（逐条算式，`Fidelity` §C.4）**：① `/cloud_registered` 被多转 **30.72°**（64.6% 的点落到水平面以下）；
> ② `p2l` 高度带在 `urdf` 档"歪打正着仍然对"（点云没变），但 `/scan` 一旦被 TF 消费就变成**斜平面**
> `z_odom = −sin30°·r·sinφ = −0.5·r·sinφ` ⇒ `obstacle_layer` 把一半波束按高度丢掉（lethal 340 → 75）；
> ③ `linefit` 的 `sensor_height` 在那一档"幸运地对"——**是巧合不是设计**；
> ④ 自击掩膜的 113 个盒子是"帧正"假设下烘的，若将来点云真变成斜系表达**必须重烘**。

### §2.1.6 自击掩膜（`self_mask_*`）：把"近场自身回波"剔出限速判据

| 项 | 内容 |
|---|---|
| **现象** | 用户带 GUI 那次：**车静止**时限速被钉在 **0.60 m/s 地板**；日志 `why=step d=0.00 m 台阶≈0.164 m`（其中 9 条 `limit=0.60`）；`~/traversability_stats` 里"参考坡度"= **0.0°**（真值 ~2.9°） |
| **原因** | 雷达装在底盘凹槽里 ⇒ **26.9~29.0% 的点落在 `r ≤ 0.09 m`**（近场自击，是**物理真实**的回波）；它们在**前瞻走廊的第一个粗格**里造出 `d = 0.00`：**局部地面被算成 +0.000 m**（= 雷达自身高度！真值 −0.223 m）、台阶残差 **0.023~0.203 m**（帧间跳）、"参考坡度"被钉成 **0.0°** |
| **过程（判决性测量）** | ① 逐点复核：近场点中位半径 **0.021 m**、不在任何 collision 几何上（113 个 AABB 只命中 3 点）⇒ 与"内移 0.1 m"的插件 bug 吻合（§2.2.3 就是它的根因修复）；② 离线射线验证（`robot11_self_mask.py --verify`，30000 条）：**地面环（最近 0.3457 m）被掩 0 个** ✅、真 mesh 表面 <0.75 m 命中被掩比例 **0.2310 → 0.9992** |
| **改动** | `(a)` 本模型 12 个 link 的 `<collision>` 在 `livox_frame` 下的 **113 个 AABB**（逐个 ≥5 mm 膨胀，z 下限抬到地面以上 0.03 m）**∪** `(b)` 近场死区 **`r_xy ≤ 0.2416 m` 且 `z ≥ −0.2295 m`**；参数文件按 robot 槽位选（默认**关**、`robot11` 开） |
| **原理** | ① 掩膜**只作用于判据/限速**，**不动** `/segmentation/*` 的标签（动标签 = 改共享契约，影响默认模型）；② `(b)` 的上界是**几何硬约束**：下俯 30° + FOV 下沿 7.22° ⇒ 最低射线的地面交点在 **0.3416 m**，取 `0.3416 − 0.10 = 0.2416 m` 留余量；③ **两个掩膜盒子表是生成物**：模型改了必须重跑 `--emit`（`--check` 会报错） |
| **效果** | 静止窗口（掩膜关 → 开）：`self_masked` **0 → 3374 点/帧（29.4%）**；走廊最近格 `data_min_d` **0.00 → 0.20 m**（37/37 → 46/46 帧）；近场 max 台阶残差 **0.023~0.032（日志里出现过 0.128~0.203）→ 0.002~0.005 m**；"参考坡度" **0.0 → 2.9~3.1°**；限速落在地板的帧数 **9/129 → 0/46**（min 0.606 m/s）；**真台阶仍限速**：静止窗 `why=step` 40/46 帧、触发格 2.3~2.8 m、`台阶=0.180 m`；独立复测（114 帧静止 + 75 帧行驶）里行驶 75 帧中 **`why=step` 28 帧**、限速最低 **0.6 m/s** |
| **回退** | `config/traversability_self_mask_robot11.yaml` 的 `self_mask_enable: false`（或撤掉 launch 里 `self_mask_params` 那一路） |
| **证据** | `robot_models.md` §12.3/§12.6/§13.6；`Fidelity` §K.2.4（`dz` 三档实测：0.05 m 的闸有 **>2×** 余量）；提交 `1ca09eb` |

### §2.1.7 ★ 底盘**塌成单刚体**：`planar_move` 的 `Set*Vel` 只对单刚体自洽

| 项 | 内容 |
|---|---|
| **现象** | `robot:=robot11` **几乎不执行角速度指令**：盲发 `wz=1.0 rad/s` 持续 12 s（请求 **687.5°**）真值只转 **+3.79°**（**0.55%**）；反向 2.0 m 目标里 RPP 的 rotate-to-heading `wz=-0.75` **连发 45 s**、`vx` 恒 0、车只转 **6.03°**；**无指令 60 s 自由漂移 yaw +8.20°** |
| **原因** | 底盘驱动是 `libgazebo_ros_planar_move.so`，它的实现（`gazebo/physics/Model.cc:746-771`，本机读源码核对）是 **`SetLinearVel`/`SetAngularVel` 对模型里每一个 link 各设一遍同一个 `(v, ω)`**。这对**单刚体恰好自洽**（绕质心转时正是"每个 link 同一个 ω"），对**多刚体 + 关节树则不自洽**（第 i 个 link 的质心速度本应是 `ω × r_i`）⇒ 关节/接触约束求解器每步都要把它掰回来，**反作用把底盘的角速度吃掉**。`robot11` 是 **13 link / 12 joint**，默认模型只有 **7 link / 6 joint** ⇒ 前者被吃得更狠 |
| **过程（判决性测量）** | 先做**台架**（只 Gazebo + 底盘插件，摘掉 LIO/nav2/雷达，一次 <40 s、RTF≈1.00），可信度由三条外部锚点钉住（复现了 §L 的数）：正前方盲推极限 **0.4199 → 0.4077 m**、默认模型 **0.5402 → 0.5401 m**、原样 `wz` **+3.79°（墙钟）→ +7.09°（12 仿真秒）**。四个结构对照（同一世界/出生点、`wz=1.0` × 12 仿真秒）：极简单 link box（无接触）**+681.25°（99.9%）**／robot11 塌成单刚体 **+111.68…113.32°（16.2~16.5%）**／robot11 原样 **+7.09°（1.03%）**／全关节改 `fixed` **+21.31°（3.10%）**／仅转向关节 fixed **+16.69°（2.43%）**／零重力 **+128.02°（18.6%）**／默认模型 **+40.01°（5.82%）**（它自己零重力上限 48.1%） |
| **改动** | `tools/scripts/regress/robot11_weld_chassis.py`（生成器 `robot11_make_sim_xacro.py --chassis rigid` **默认调用**）：① 从 `base_link` 对 `j2…j11` 做 FK；② `l2…l11` 的 `<collision>/<visual>` pose **逐条**变换到根 link 系后挂上去（**几何一块都不改**）；③ `<inertial>` 用**平行轴定理**合成（`M = 9.4121 kg`、`COM = (0.003551, 0.007121, 0.073122)`、`Izz = 0.126766 kg·m²`）；④ 删掉 `j2…j11`，**保留** `livox_frame`/`imu_link` 及其固定关节；⑤ 去掉指向被塌掉 link 的 `<gazebo reference>` 材质块 |
| **原理** | ① **"没有执行机构的自由度"就是负担**：本栈**没有任何**轮子/云台控制器（`.xacro` 自己写着"上游 `j2..j9` 是 continuous 但我们不加轮子控制器"）⇒ 这 12 个关节在运行期唯一的作用就是**与 `planar_move` 较劲**；② **自证必须有"不依赖 FK 正确性"的物理锚点**：AABB 逐条相等这条会"两边一起错、检查照样通过"（实测踩过一次：`rpy="1.5708 -1.5708 0"` 被按"6 个数"解析 ⇒ 读成零向量），加上"**整车最低点必须落在轮子上 = −0.10250 m**"之后那次退化当场报错；③ **接触/摩擦是第二个独立限制项**（剂量-响应单调：`mu` 1.0/0.5/0.2/0.05/0.001 ⇒ `+7.09/+17.26/+54.75/+96.91/+126.80°`），但它**只解释 18.6% → 1.03% 那一段**，主因仍是结构不自洽 |
| **效果** | 台架：**1.03% → 16.2~16.5%（15.7×）**。整栈（同协议）：盲发 `wz=1.0`×12 s 真值 **+3.79° → +35.42°**（LIO +33.75°）、执行率 **0.55% → 5.15%**（**优于默认模型的 3.98%**）；**60 s 无指令自由漂移 +8.20° → +0.006°**（LIO −0.091°，与默认模型 −0.154° 同量级）；反向 **1.2 m** 目标：真值位移 **1.2102 m**（请求 1.2）、残余 0.0794 m、**`Goal succeeded`**。**被否掉的修法**：插件 `update_rate` 100 Hz → 1000 Hz 确实把角速度提到 7.8%，但**同一次改动让车"爬上"了实体障碍**（盲推 0.4077 → 2.5652 m、底盘 z 0.1522 → 0.4005 m）⇒ **不采用** |
| **回退** | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --chassis articulated`（生成物**模型体去注释后与 2026-10-09 的 HEAD 逐字节相同**，两份都是 35025 B、`diff` 为空）；塌陷工具本身 `rm -f tools/scripts/regress/robot11_weld_chassis.py` |
| **证据** | `tilted_lidar_fidelity.md` §L.5/§M.0–§M.12；提交 `ef927e2` `9de6b72` `d7c4da0` `d3034aa` |

## §2.2 感知 / 代价图层

### §2.2.1 近地剔除（`obstacle_near_ground_m`）：2D 链路上**唯一**能用上"高度"的那一级

| 项 | 内容 |
|---|---|
| **现象** | 用户：「那个膨胀半径感觉一直把地面当成障碍物算进去」；实测**车那格 = 42（非 free）**、车半径圆（0.3565 m）内 **`≥99` 156 格、free 只有 239 格**、到最近 lethal **0.531 m**、到最近 `≥99` **0.175 m** ⇒ `RegulatedPurePursuitController detected collision ahead` → `Controller patience exceeded` → 自旋恢复 |
| **原因** | 三层叠加（**逐条带数**，`Fidelity` §D.3）：① 近场地面残留（水平 **0.26~0.40 m**，linefit 因 `r_min 0.2` + `max_dist_to_line 0.05` 最难判的那一圈）；② 插件 `point = range·axis` 的**系统内移 0.1 m** 把近处地面整体抬高；③ `p2l` 把 3D 点**投成 2D**（LaserScan 没有高度）⇒ `obstacle_layer` 把它们标在**雷达高度**上 ⇒ **"地面"在局部代价图里就是货真价实的 lethal 格**。⚠️ 全局图那一路是 3D 的（STVL、`min_obstacle_height 0.0`）⇒ 同样的地面点（odom 里 z ≈ −0.07）会被滤掉（所以两张图的最近 lethal 略有差别 0.386 vs 0.400 m） |
| **过程（判决性测量）** | ① **先给"贴地点"一个判据**（在投影成 2D **之前**）：用判据层本来就有的局部地面 `g(x,y)`（0.20 m 粗格 → 4×4 个 0.05 m 细格"最低点"的 p05）算 `dz = z − g`；② **同一帧回放 A/B**（把一帧真点云 11568 点回放给**活的** `ground_segmentation_node`，只换阈值）：0.00/0.05/0.10 ⇒ ground 4683/**5985**/7286、obstacle 6885/**5583**/4282（**总数恒为 11568 = 不丢点**），`step_edge` 中位 **2011/2011/2011（逐字相同）** |
| **改动** | 新增参数 `obstacle_near_ground_m`（**0.0 = 关**），`robot:=robot11` 槽位 = **0.05**；"升"（obstacle→ground）显式写在两个分割节点里、**只在 `nearGroundEnabled()` 为真时**执行 ⇒ 默认路径连掩码都不填、行为逐字节不变；`~/traversability_stats` 多两个**私有**诊断字段 |
| **原理** | ① **信息一旦跨过 `p2l` 就永久丢失**（LaserScan 没有 z）⇒ 想用高度就必须在"投影之前"；② 阈值取 **0.05 m** 的三个实测依据：近场地面环 `dz` p05/p50/p95 = **+0.0035/+0.0057/+0.0095**（100% ≤ 0.05，余量 >5×）、水平 0.40–0.50 m = **+0.0018/+0.0089/+0.0125**、真障碍（~0.15 m 低矮件）在 0.50–0.75 m 的 `dz` p95 = **+0.1408**（只剔掉贴地那一层：`frac dz≤0.05 = 0.775`，**留下的正是 0.10–0.17 m 的本体**）；③ 与台阶判据**结构上互斥**：台阶闸 `step_height_threshold = 0.15 m` > 0.05 ⇒ 台阶点不可能被剔；④ `dz` 是"同一个量在同一个帧里相减"⇒ **与 `sensor_height` 标定、与 odom 原点都无关** |
| **效果** | 在线 A/B（robot11、静止、唯一变量 = 阈值）：车那格 **42 → 0（FREE）**；车半径圆内 `≥99` **156 → 0**、free **239 → 999**；到最近 lethal **0.531 → 1.534 m**、到最近 `≥99` **0.175 → 1.178 m**；径向剖面 0–0.3565 m 的 `≥99` 格 **全 0**；`/segmentation/obstacle` 中位 **6178.5 → 5851**、`/scan` 只少 **53 条/帧**（全是 0.30–0.50 m 的贴地回波） |
| **回退** | `config/traversability_near_ground_robot11.yaml` 的 `obstacle_near_ground_m: 0.0`（一个数），或去掉 launch 里 `near_ground_params` 那一路 |
| **证据** | `Fidelity` §K.2；提交 `360d493` `18f2738` |

### §2.2.2 `robot_radius`：**外接 0.3565 → 内切 0.300**（"能走多窄" vs "保证不撞"）

| 项 | 内容 |
|---|---|
| **现象** | "车一开始就在膨胀团里"：**全局**图上（静止！）外接那一档车那格 = **99**、车半径圆内 **0 格 free**、到最近 lethal 只有 **0.135 m** |
| **原因** | 本仓**没有 footprint 多边形** ⇒ `robot_radius` 在 nav2 里的真实语义 = **`inscribed_radius`**（"能走多窄"）。用**外接**半径（0.3565 = 凸包顶点最远）会把"车那格"在全局图上判成 inscribed；而**车那格非 free 的真因并不是半径** —— 先是"贴地点被标成 lethal 格"（§2.2.1），**再**被大 `inscribed_radius` 放大成"整个足迹非 free"（这条**纠正了 §D.3 的归因顺序**） |
| **过程（判决性测量）** | 干净静止三档 A/B（近地剔除都开、**唯一变量 = 半径/膨胀**）：① 外接 **0.3565/0.70**；② 内切 **0.300/0.60**；③ 0.22/0.50（默认模型那组，只为分离变量）。结果：**局部**三档都是"车那格 0 + 圆内 0 个 ≥99"（⇒ 局部问题已被近地剔除解决）；**全局**只有 ② 在静止时车那格 = 0（① 是 99、③ 是 77） |
| **改动** | `src/bringup/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml`（**只含几何键**，launch 用 `_Nav2ParamsForSlot` 深合并成临时 YAML ⇒ 其它槽位**原样返回公共段路径**）：local/global `robot_radius 0.3565 → 0.300`、local `inflation 0.70 → 0.60`、global `0.75 → 0.65`；`cost_scaling_factor` 不动 |
| **原理** | ① **半径的语义决定取值**：要"能走多窄" ⇒ 内切；要"保证不撞" ⇒ 外接或真 footprint；**两者不能用一个数同时满足**；② **折中必须写清**：内切圆会让**四个角在转弯时扫到** 0.300–0.3565 m 这一圈里的障碍（底盘是 600×600 切角，角点在 0.3565、边到原点 0.300）；兜底 = 场地没有更贴的窄缝 + 软带仍在 + 前瞻限速与碰撞监控仍在链路上 |
| **效果** | 采用 **0.300/0.60/0.65**：全局车那格 **99 → 0**、车半径圆内 **0 → 123 free**、到最近 lethal **0.135 → 0.804 m**；局部圆内 free **723/1008 → 871/1005**；软带 0.30/0.35 m 与默认模型（0.28/0.33 m）同量级。**真足印多边形：试了，没采用**（原因 1：nav2 的 `footprint` 参数声明成 `std::string`，本仓的运行时深合并写嵌套序列会让 `rcl` 报 `Sequences cannot be key at line 127` 并把**所有** nav2 节点一起 exit；原因 2：量到的收益只剩"转弯角不扫障碍"这一条） |
| **回退** | 改 `nav2_params_sim_robot11_costmap.yaml` 的两个键回去；或 `git revert 1c89473` 的该文件 |
| **证据** | `Fidelity` §K.3；`robot_models.md` §9.4（内切 0.300 / 外接 0.3565 的来源）、§11.9（覆盖层机制与运行时回读）；提交 `1c89473` |

### §2.2.3 插件 `point = range·axis` 漏了射线起点（0.1 m 的系统内移）

| 项 | 内容 |
|---|---|
| **现象** | 原始云的**"地面峰"随距离变化**（像斜面）：`r 0.25–0.35 → −0.208`、`0.35–0.50 → −0.219`、`0.5–1.0 → −0.237`、`2–4 → −0.253`（几何真值 **−0.2595**）；近场一团点（28.5% 落在 `r<0.12 m`）不在任何 collision 几何上 |
| **原因** | 源码级一行：射线起点是 `start = minDist·axis + offset.Pos()`（`minDist` = SDF `<range><min>` = **0.1 m**），而 `range` 是**从射线起点**量的距离 ⇒ 命中点应为 `range·axis + minDist·axis + offset.Pos()`，旧代码写的是 `point = range·axis` —— **既漏 `minDist·axis` 也漏 `offset.Pos()`**，于是每个点都朝传感器方向**内移 0.1 m**（近处仰角大 ⇒ 抬得高） |
| **过程（判决性测量）** | ① **可核对的代数事实**：`point = range·axis` 与 `InitializeRays()` 的 `start = minDist·axis + offset.Pos()` 用的是**同一个 `axis`**；② 跨模型 A/B（同一编译只差那一行）：默认模型的 `obstacle` **3716 → 3782（+66）**、`voxel_grid` **1129 → 1303（+174）**、`inscribed` 反而**降 580**；robot11 的**地面峰 −0.280 → −0.155 m**（旧值"随距离变"、新值是**尖峰**：`z ∈ [−0.16,−0.15]` 单桶 3283 点 = 1–4 m 环的 **76%**） |
| **改动** | `src/simulation/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp`：`point = range_from_origin_ ? (range*axis + minDist*axis + sensor_offset.Pos()) : (range*axis)`；回退开关 SDF **`<range_from_origin>false</range_from_origin>`**（**缺省 = 修好的语义**）；启动日志打一行便于确认加载了哪一版 |
| **原理** | ① 这**只让坐标回到几何真值**（修复后 `point` 与 `start + range·axis` 逐位相等）；② 敢设成**默认**（而不是只给 robot11）的依据 = **默认模型没有任何一项变坏**；③ **代价必须登记**：它改的是**所有模型**的点云 ⇒ 任何"按旧点云烘出来的东西"都要重核（`traversability_self_mask_robot11.yaml` 的 113 个 AABB 与近场死区都是按旧点云烘的）——实测掩膜命中率**没有塌**（自击团离传感器只有 0.02~0.05 m，0.1 m 内移对它的**相对**位置影响最小）；⚠️ 若将来把 `minDist` 改大（>0.1 m），掩膜**必须重烘** |
| **效果** | robot11：**车半径圆内 `≥99` 13 → 0、free 601 → 851**（"修完这一行才和 §K.2.3 的圆内 0 个 ≥99 一致"）；`/scan` 有限波束 **864 → 772**（旧的 0.05–0.10 m 自击带里那批"被内移 0.1 m 后落进带内"的假波束消失）；局部 lethal/inscribed/free **338/11730/38277 → 231/8599/43960**（三项都往"更干净"走）；自击团点数 **3294 逐位相同**（⇒ 掩膜不用重烘的量化依据） |
| **回退** | 在对应模型的 xacro 的 `<plugin>` 里加 `<range_from_origin>false</range_from_origin>`（**逐字节**回旧行为，不需要回滚代码） |
| **证据** | `Fidelity` §K.4；`robot_models.md` §12.3.1；提交 `1403f42` `8e73c8b` |

### §2.2.4 linefit 的 `gravity_aligned_frame`：`Eigen::Affine3d` **未初始化**（点云被塌到原点）

| 项 | 内容 |
|---|---|
| **现象** | 该键一开（`base_link`）⇒ `/segmentation/ground` **恒 0 点/帧**（与"把输入点云预先转 −30/+30/−60/+60"无关 ⇒ 只与"走没走那条分支"有关） |
| **原因** | 源码（`ground_segmentation_node.cc`）：`Eigen::Affine3d tf;` —— **Eigen 的默认构造不清零**（只把仿射最后一行置 `(0,0,0,1)`），随后 `tf.translate(0,0,0)`、`tf.rotate(q)` 在**垃圾矩阵**上叠旋转 ⇒ `pcl::transformPointCloud` 把点全塌到 ~1e-310 |
| **过程（判决性测量）** | 三条独立证据：① **最小 C++ 复现**（Eigen 3.4）：`(tf * Vector3d(0.1,0.2,0.3))` → `(7.5e-310, 7.5e-310, 7.5e-310)`，换成 `Identity()` → `(0.1, 0.323, 0.160)`；② **离线复算**：同一帧先转 −30° ⇒ 判地面 **5100/11711 = 43.5%**（其中 81% 真是最低那层地面）；不转 ⇒ 23%（那 23% 里只有 0.7% 是真地面）；③ **回放活的节点**：`""` ⇒ **3969** 点/帧；`"base_link"` ⇒ **0** 点/帧 |
| **改动** | 一行（+注释）：`Eigen::Affine3d tf = Eigen::Affine3d::Identity();` |
| **原理** | ① **结构性论证**：该修复**只动 `if (!gravity_aligned_frame_.empty())` 分支内部**，默认路径根本不进这块代码 ⇒ 默认行为**不可能**变化（同一帧实测也确认了 3969 → 3969）；② 修好之后"点云留在传感器系 + `gravity_aligned_frame: base_link`"这条**更接近真机**的链路**重新可用**（本仓 `robot11` 仍然选择"在源头对齐"，因为 p2l 不用建 tf2 MessageFilter） |
| **效果** | 修前 → 修后：`gravity_aligned_frame: ""` ⇒ **3969 → 3969**（逐字节口径一致）；`= "base_link"` ⇒ **0 → 3969**（obstacle 7742） |
| **回退** | 把那一行改回 `Eigen::Affine3d tf;`（或 `git revert 0142bd9`） |
| **证据** | `robot_models.md` §11.4；提交 `0142bd9` |

### §2.2.5 2D 高度带"为什么不能重新定基"（结论 = **不动**，并把原因写进参数文件）

| 项 | 内容 |
|---|---|
| **现象** | 想让"近场地面残留"不被标成 lethal ⇒ 很自然想到调 `obstacle_layer.scan.min/max_obstacle_height` |
| **原因** | **结构原因**（三行可核对）：① `nav2_costmap_2d` 的 `laserScanCallback` 走 `projector_.transformLaserScanToPointCloud(...)` —— `laser_geometry` 的 `projectLaser` 对 LaserScan 只能产出 **z ≡ 0**；② `obstacle_layer.cpp` 的标记循环在**代价图帧**里量高度（本仓 `third_party/nav2/.../obstacle_layer.cpp:470,476`），而 `min_obstacle_height` 的 nav2 默认是 **0.0**（本仓 base 参数里根本没有这个键 ⇒ 走默认，运行期回读确认 = 0.0）；③ 合起来 ⇒ **每条波束在代价图帧里的 z = 那一帧"点云原点"的 odom z（一个常量）**，与它打到的三维点**无关** |
| **过程（判决性测量）** | robot11、`plugin` 档、静止：`/scan` 有限波束 **950** 条；波束在 odom 里的 z：min/p05/p50/p95/max = **+0.0564/+0.0637/+0.0652/+0.0807/+0.0815 m**（跨度只有 25 mm）；落在 `[0,2]` 之外的 **0/950**；同一跑 `odom→livox_frame` 的平移 z = **+0.064 m**（与 p50 吻合到 1 mm） |
| **改动** | **保持 0.0 / 2.0 不动**，并把"这两个键在 2D 链路上的真实语义 = 点云原点的代价图帧 z 的常量闸"写进 `nav2_params_sim_robot11_costmap.yaml` 的注释；对 `plugin`（默认）档：波束 z ∈ [0.056, 0.082]、`[0,2]` 全收 ⇒ **现状已经正确，改任何值都只会变坏**。对 `sensor` 档：波束 z 跨度 **−0.394 … +2.361 m** ⇒ 单靠一条 `[lo,hi]` **永远**只能救回半个环（真修得改代价图帧或 LIO 的 odom 重力对齐，两者都超出范围且会把默认模型一起拖下水） |
| **原理** | **"能调的参数"不等于"能改的语义"**：当一个键作用在错误的帧里时，调它的数值区间只是在移动一个常量，不会产生"逐点高度判别"的能力；正确的做法是**在信息还存在的那一级动手**（§2.2.1 的近地剔除） |
| **效果** | 验收表里"`/scan` 出带比例（`sensor` 档）"要求 ~0 而实测 **0.3960** ⇒ **未达成**，但这条被记成"2D 链路结构上定不了基"，替代修法（近地剔除）对**两档都成立**且不依赖"odom 的 z 是什么意思"；同时更正 §J.5 的一处口径（见 §2.3.4 与 §5.1） |
| **回退** | 无（未改）；若要试验：改这两个键 + 重跑 `run_tilt_mount_probe.sh` 即可（但**预期只是整盘平移**） |
| **证据** | `Fidelity` §K.1（含 §K.1.2/§K.1.3/§K.1.4） |

### §2.2.6 `pointcloud_to_laserscan` 高度带：`max_height 0.1 → 1.0`（离地覆盖 0.326 → 1.226 m）

| 项 | 内容 |
|---|---|
| **现象** | 场地里 **0.40 m 以上**的护墙/立柱在 `/scan` 里**整条方位是 `inf`**；`inf_is_valid=true` 时下游把 `inf` 当"10 m 处有回波"**沿射线清成 free** ⇒ 这才是真正危险的方向；也是"坡脚撞击点在图上看不见"的成因之一 |
| **原因** | `target_frame: ""` ⇒ z 过滤发生在**点云自带帧**（`livox_frame`，离地 0.226 m）⇒ `max_height 0.1` 的实际含义是"离地 **+0.326 m** 以内"。换算错了**对象**（以为是离地高度，实际是传感器系 z） |
| **过程（判决性测量）** | 同一批帧、同一朵 obstacle 云、**只换 z 过滤**，并排打印两套带：0.20–0.30 m 带 96.3% → 97.0%；0.30–0.40 m **83.3% → 98.5%**；**0.40–1.00 m 0.0% → 100%**；≥1.00 m **0.0% → 98.5%**（linefit；patchwork 同向） |
| **改动** | `src/perception/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml`：`max_height: 0.1 → 1.0`（`min_height: −1.0` **不动** —— 它的语义是"雷达下方 1.0 m 以内也要"，比场地最低几何（坑底 −0.2 m）更低 ⇒ 不丢东西） |
| **原理** | ① **`min_height` 保持不动的理由是"不丢东西"，不是"对称美观"**；② 与 COD 2026 的 `min 0.01 / max 1.00` **数字看着像、含义不同**：他们配 `target_frame: chassis`（地面高度附近的帧）⇒ 那组数是**离地**的；本仓不做 TF ⇒ 同样的 `1.0` 落在**离雷达 1.0 m = 离地 1.226 m**；③ **风险与回退必须成对写**：抬到 1.226 m 意味着**头顶结构**（悬挑/限高杆/桥）也会被投影到地面 ⇒ 假障碍；本场地没有这类几何（已用 `PCD/RMUC2026.pcd` 与 `RMUC2026.stl` 复核：driving 区内 z>1.2 m 的几何都是墙/立柱的**上方部分**，其正下方本来就有同一面墙）；换场地/真机若有悬挑**要重估** |
| **效果** | 实时链路同帧 A/B（202 帧）：`/scan` 有限束数 linefit **856.8 → 934.7（+77.8，+9.1%）**、patchwork **1113.8 → 1163.5（+4.5%）**；只在新带里出现的束 p50/max = **+83/+143**（linefit）；新带**丢掉**的束 = **0.0**；同一方位回波**距离**的变化 p50 = **0.000 m** ⇒ **纯增量**（老回波一个不动） |
| **回退** | 把 `max_height` 改回 `0.1`（`ground_segmentation_slots.md` §14 的回退表） |
| **证据** | `ground_segmentation_slots.md` §11/§12/§14；提交 `cb3a756` `97d0dbc` `819160c` |

### §2.2.7 地面分割槽位 `ground`：linefit ↔ Patchwork++（含 `th_dist 0.125 → 0.08`）

| 项 | 内容 |
|---|---|
| **现象** | 22~35° 的**缓坡/倒角**被 linefit 判成障碍（实测障碍率 **35.7~44.3%**）⇒ 坡面从 `/scan` 消失、2D 图里也看不见；反过来把 `th_dist` 放松又会"吃"掉低矮障碍 |
| **原因** | linefit 的 `max_slope ±0.4` / `max_dist_to_line 0.05 m` 对"缓坡"过于敏感（把坡面整片删成地面）；Patchwork++ 的 `th_dist` 是"点到拟合平面的距离小于它才算地面" = **地面层厚度**，任何竖直特征最下面 `th_dist` 那一段会被并进地面 |
| **过程（判决性测量）** | ① 上游考证：`url-kaist/patchwork-plusplus` pin **`3e6903a1`**（= tag v1.4.1）、根 `LICENSE` **BSD-2-Clause**；⚠️ 它的 `ros/package.xml` 写 **GPL-3.0** 而 `ros/LICENSE` 是 **MIT** ⇒ **许可不清 ⇒ 不 vendor ROS wrapper**，只 vendor `cpp/**` 自己写薄节点；② 参数扫描：`th_dist` 扫 0.05/0.06/**0.08**/0.10/0.125 五档（合成地形）——**0.125 时 0.15 m 薄墙在 p2l 口径下只剩 11.8% 覆盖**，0.08 时 0.15/0.30/0.40 m 薄墙 **100%** 覆盖、0.2/0.3 m 平台顶面 100% 覆盖；③ 整栈 smoke A/B（`bench/ground_slot_smoke.sh`，同脚本/同时序/同场地） |
| **改动** | 新增 `ground` 槽位（`linefit`（默认）\| `patchwork`）＋ vendored Patchwork++（只 `cpp/**`）＋自写薄 ROS 节点；`config/ground_segmentation_sim.yaml` 的 **`th_dist: 0.08`**（**唯一偏离上游默认的算法键**）。**默认仍然是 linefit** |
| **原理** | ① **"许可证不清就不 vendor"** 是一条独立于技术收益的决策；② **换算法前先给"硬要求"定判据**（"矮/低障碍要留成障碍"与"缓坡不该被判障碍"是**互相拉扯**的两个要求）⇒ 用它反推出参数；③ **不换默认要有理由**：本次只做了离线 + 建图链 smoke，**没做 `mode:=nav` 整栈回归** ⇒ "可以 A/B，但建议先别换默认" |
| **效果** | 缓坡：linefit 判障碍 **35.7~44.3% → patchwork 2.5~3.0%**；STL 倒角上 patchwork 判地面的比例是 linefit 的 **2.6 倍**（69% vs 27%）；`/scan` 里 **<1.5 m 的近距假回波 91.32 → 34.43（2.65×）**；`/map` 占用 **6544 → 7756（+18.5%）**、已知格 **+17.2%**；分割器 CPU **0.0836 → 0.0562 核（省 33%）**；真实帧中位 **0.52 ms vs 1.01 ms（1.94×）**。**新代价**：倒角附近平地上出现少量假障碍（8.8~10.8% vs linefit 1.0~1.2%）、−0.2 m 坑的下坡倒角反而更差（52.5% vs 8.1% 障碍率）、0.05~0.20 m 高度带方位覆盖率 94~100% → 84~93% |
| **回退** | `ground:=linefit`（默认，等于没改）；只回退 `th_dist` ⇒ 改回 `0.125`（坡面更好，但 **0.15 m 矮墙会从 `/scan` 消失**） |
| **证据** | `ground_segmentation_slots.md` §1/§3.1/§5.3/§6.1/§7/§8/§9；提交 `1a9ce8c` |

### §2.2.8 局部代价图也上 STVL（`local_obstacle:=stvl | stvl_both`）

| 项 | 内容 |
|---|---|
| **现象** | 局部图原本只靠 `/scan` 的 2D 投影 ⇒ **高度信息丢失**（同 §2.2.5）；想判断"这团点是不是障碍"在 2D 链路上没有依据 |
| **原因** | 局部图 `global_frame: odom`、`obstacle_layer.scan` 走 2D；而 STVL（spatio-temporal voxel layer）**直接吃 3D 点云**并带时间维（衰减） |
| **过程（判决性测量）** | 四次跑 A/B（`stvl_local_costmap.md` §3），参数来源 = COD `singlenav2_params.yaml:200-262`（逐键对账） |
| **改动** | `local_obstacle:=stvl`（局部替换）／`stvl_both`（两侧都上）；默认不变 |
| **原理** | **"降维发生在哪一层"是可选的**：把 3D 直接喂给局部图 ⇒ 多一条"带高度/带时间"的通路；代价是**多一路消费者**（一致性与算力）——这正是"一份点云被切成多份"的权衡（`architecture.md` §3.2.5） |
| **效果** | 见 `stvl_local_costmap.md` §3 的四次跑表（本文档未逐项转抄，属**可核对但本文未复算**的一类，见 §5.3） |
| **回退** | 槽位改回默认（`obstacle_layer` 那条路） |
| **证据** | `stvl_local_costmap.md`（§0 速查 / §2 机制 / §3 A/B / §4 回退 / §5 未验证）；提交 `4798ea2` `b2c8e1a` |

## §2.3 LIO 层

### §2.3.1 时间戳 1 ns 截断：`/scan` 丢帧 **116 → 10**、TF 等待 p90 **136 → 13 ms**

| 项 | 内容 |
|---|---|
| **现象** | `slam_toolbox` 持续刷 `Message Filter dropping message: frame 'livox_frame' at time <t> … 'discarding message because the queue is full'`，**每 2.5 仿真秒一条**（min=p50=p90=2.5 ⇒ 系统性）、约 **4.0~4.3%** 的 `/scan`，每次跑都有 |
| **原因** | **四步因果链**（每步都可单独验证）：① 插件用**仿真钟**盖戳（`rclcpp::Time`，整数纳秒，精确）；② `/scan` 那条链**不经过 LIO**、逐字段抄 header（`ground_segmentation_node.cc:135-136`、`pointcloud_to_laserscan_node.cpp:149`）⇒ `/scan` 的戳 == 插件戳；③ **另一条链（LIO）把插件戳转成 float64 秒再转回整数纳秒**：`nanosec = static_cast<uint32_t>((ts - floor(ts)) * 1e9)` —— **截断**，同一个 double 在 ~1e3 s 处的往返误差 ~1e-7 ns，方向随数值而变 ⇒ 约 **40%** 的帧落在 `X.99999999x` ⇒ 截断后变成 `X−1 ns`；④ tf2 的 `MessageFilter` 判据是"在**消息自己的戳**上 `canTransform`" ⇒ **低 1 ns 的同帧样本不合格**（被判成"要外推到未来"）⇒ 只能等**下一帧 TF**（~0.1 s）；队列深度 1 时新来的 `/scan` 把还在等的那条顶掉（`message_filter.hpp:411-419`） |
| **过程（判决性测量）** | ① 新增**逐纳秒戳差仪器**（探针加整数 ns 指标 + 离线复算器，`5410412`）；② 实测：**Δns = 0 的扫描一条都没丢（1658 条 / 0 丢），116 条丢帧的 Δns 全部 > 0**；③ `scan_queue_size` 1/5/10 的 A/B：**4.2% → 0** —— 但这条**不是**被采纳的修法；④ 上游考证：默认 1 由 PR #526（2022-08-24）引入，README 明写 "Should always be set to 1 in async mode"；维护者口径 = 这是 **TF/时间戳问题**不是 slam_toolbox 的 bug（#720/#777/#806/#834 全以"用户侧 TF 配置"关闭）；真正的上游修法在 `ros2/geometry2#544`（**2022-07 提出，至今 open**）；⑤ 可比项目（COD 2026 / TurtleBot 4 / uOttawa rover）**都没配** `scan_queue_size` |
| **改动** | **只改 LIO 的戳构造**（截断 → **四舍五入 + 进位**），让 TF 戳与点云戳**逐纳秒对齐**；`slam_toolbox` 的参数**一个都没改**（保持默认 `scan_queue_size=1`）；`timestamp_construction_audit.md` 顺带全仓扫了一遍：**`FAST_LIO` 有完全相同的 bug**（实测 Δ(TF戳−`/scan`戳) = −1 ns 占 **40.0%**、丢帧 **126/3003 = 4.2%**，与修复前的 `small_point_lio` 逐项吻合）；**`point_lio` 代码逐字相同**（判定同一 bug，但**没能实测**——该槽位在仿真里第 100 帧就被自己的 `pcd_save` 打死）；**全仓没有第 3 处**截断类构造 |
| **原理** | ① **根因修复 ≠ 队列补丁**：调大队列只是把"等待"吸收掉，被吸收的消息最终仍可能因超时/外推失败而丢（§1.2 第 9 条）；② **"谁的时间戳在骗人"要用逐纳秒的仪器去量**，不能靠"看起来对不对"；③ 真正把"1 ns"放大成"丢数据"的只有 `scan_queue_size=1`；costmap 侧是 `MessageFilter(queue=50)` + `transform_tolerance`，**只会多等一帧、不会丢** |
| **效果** | 验证跑 `fix01`：丢帧 **116 → 10（4.3% → 0.35%）**、**2.5 s 节律消失**、等待 TF 的 **p90 136 ms → 13 ms**；丢帧影响量化：被丢的 4% 里 **58.6% 被两个处理门证明本来就会忽略**，剩下 41.4% 的代价上界 = 被选中的帧晚 **0.1 仿真秒 / 2 cm** ⇒ 结论原本是"**不改默认**"，修完根因后连这条代价也基本消失 |
| **回退** | `git revert a281636`（回到截断语义 ⇒ 丢帧日志会回来）；**不要**用调大队列来"消日志" |
| **证据** | `slam_toolbox_scan_drops.md` §0/§6/§7/§8/§9；`timestamp_construction_audit.md`（结论先行 + §3/§4.4）；提交 `a281636` `5410412` `c73e9d0` `5c3d7af` |

### §2.3.2 bug ②：`/cloud_registered` 对"已经在 odom 的点"又乘了一次 `T(base_link←livox_frame)`

| 项 | 内容 |
|---|---|
| **现象** | `robot11_mount:=urdf` 档：`/cloud_registered` 里**地面变成 30.97° 的斜面**、**64.6% 的点落到水平面以下**（`plugin` 档是 0/11620）；RViz 固定帧取 `odom`/`map` 时整个场景像"被刚性转过" |
| **原因** | 回调收到的点来自 `small_point_lio.cpp:92-98` 的 `p_odom = R_ol·(extrinsic_R·p_lidar + extrinsic_T) + t_ol` —— **已经是 odom 系**；而 `small_point_lio_node.cpp:141` 又 `lookupTransform("base_link", lidar_frame)`、`:191` 做 `R*p+T`、`:159` 写 `frame_id="odom"` ⇒ **对已在 odom 的点再乘一次变换、还盖 odom 的标签** |
| **过程（判决性测量）** | ① **反变换检验**（同一条判据两个独立口径）：按 `T(base_link←livox_frame)` 反变换回去，地面从 **30.970° → 1.041°**（回到水平）；② **逐点残差**（最近邻配对）：`H_a`（`p_pub ≈ R(base←livox)·p_raw + t`）残差 p50 **2.67 cm**、82.4% 的点在 5 cm 内；`H_b`（不转）p50 **11.47 cm**、只有 10.2% 在 5 cm 内 ⇒ **H_a 赢 4.3 倍**；③ **地面法向分量**：`R(base←livox)·n_raw` 与实测发布的地面法向夹角 **0.088°** ⇒ 是**同一个刚体变换** |
| **改动** | 删掉那次 TF 查询与逐点刚体变换，直接把回调里的点写进消息（`*pointer = point.x()/y()/z()`）；`frame_id` 仍是 `odom`；文件里留日期 + 机制 + 影响面的中文注释 |
| **原理** | **`frame_id` 是承诺**：既然写 `odom`，数据就必须真的在 odom 里；**"默认档看不出来"不等于"没问题"** —— `plugin` 档那条 TF 是**纯平移**（`|t| = 0.2044 m`），所以它只把整朵云平移了 0.2 m，看不出"多转" |
| **效果** | `urdf` 档：**30.970° → 1.054°**；默认档：帧内几何（点数/地面倾角/扇区/自仰角谱）**逐项不变**、只把整朵云搬回真正的 odom 坐标（−t = −0.2044 m）；副作用检查：`/global_costmap/voxel_grid` 每帧点数 **761 → 791.0**（量级内）、`/segmentation/*`、`/scan` 点数不变 |
| **回退** | `git revert <J 的 LIO commit>`（默认档**逐位不变**，revert 后与 §I 那一轮等价） |
| **证据** | `Fidelity` §I.5.1/§I.5.3/§J.1.1/§J.2；提交 `2a5a8b6` `7542973` |

### §2.3.3 bug ③：`odom→base_link` 的姿态用了**共轭**而不是**合成**（假俯仰 4.890° → 0.317°）

| 项 | 内容 |
|---|---|
| **现象** | `urdf` 档：发布的 `odom→base_link` rpy = `[0.383, **4.890**, 7.701]°`，`/odom` 窗口内 pitch 跨度 **4.924°**，而真值 `/odom_ground_truth` 只有 **0.003°** ⇒ **nav2 以为车在 odom 里俯仰 4.9°** |
| **原因** | `small_point_lio_node.cpp:99` 取 `lookupTransform(lidar_frame,"base_link")`、`:109` 算 `T_ob = T_bl⁻¹·T_ol·T_bl` —— 这是一个**相似变换（共轭）**，不是坐标合成。正确写法由**链式法则**给出：`p_odom = T_ol·p_livox = T_ol·(T_bl·p_base)` ⇒ `T_odom←base = T_ol·T_bl`；等价说法：位姿一致性 `T_ol = T_ob·T_bl⁻¹` ⇒ `T_ob = T_ol·T_bl`。**共轭只在 `T_bl` 是纯平移时"姿态碰巧对"** |
| **过程（判决性测量）** | ① 写了一个 **30 行 tf2 单测**（`tools/scripts/tiltmount/tf2_compose_order_test.cpp`，不进 colcon）把语义钉死：`tf2::Transform` 的 `A*B` 就是矩阵序 `R_A·R_B`；旧写法对一个"只有 yaw 9° 的水平姿态"给出 `(0.306, 4.486, 7.810)°`（**与实测那 4.9° 的符号/量级都对得上**）；② **正演** `R_bl·R_ol·R_blᵀ` 与实测 rpy **逐位相同**（0.383/4.890/7.701）⇒ 机理钉死；③ 按代码语义**反解** LIO 状态 `R_ol` = `[0.086, 0.405, 9.105]°`（水平，只有 yaw）⇒ 说明"物理正确值就是 `R_ol`"（在"点云其实表达在水平系"的现状下） |
| **改动** | 姿态取**合成** `T_ol · T_bl`；⚠️ **平移故意保留旧值**（不是漏改，理由见下） |
| **原理** | ① **"共轭/相似变换"与"坐标合成"是两件事**，只有当被夹在中间的那个变换是纯平移时才会重合；② **读表要用矩阵/一致性，不要用 rpy 分量**：`sensor` 档的 **−4.9° pitch 不是假俯仰**，而是"绕斜 30° 的 odom 转 yaw"在 ZYX 展开下**必然出现**的项（矩阵本身是对的：`R_ob == R_x(30)·Rz(Δ)`、`R_ob·R_bl⁻¹ == R_ol` 两条恒等式见 §J.3 单测与 §J.4.1）；③ 一条**记法更正**：§I.9 的 C++ 单测原把"正确合成"写成 `T_ol·T_bl⁻¹`（在 `T_bl := T(livox←base)` 的记法下差一次求逆），以 §J 为准 |
| **效果** | `urdf` 档：假俯仰 **4.890° → 0.317°**（< 0.5° 验收线；那个 30° 出现在 **roll** 上 = "odom 定义在斜的初始传感器系"这个物理事实）；默认档：**姿态与平移逐位不变**（`T_bl` 旋转 = 单位阵 ⇒ 合成与共轭姿态相同；平移显式取旧值） |
| **平移那半"故意不改"的实测代价** | **把平移也改成物理真值 ⇒ 会打断两张代价图**：`/scan` 盘面在 odom 里的 z（p50）**+0.063 m → −0.091 m**；落在 `obstacle_layer` 高度带之外的波束 **0/948 → 950/950**（robot11）、**1239/1239**（默认模型）；局部代价图 lethal/inscribed **461/16937 → 0/0**（整张图 62500 格全 free）、默认模型 **445/11029 → 0/0**。机理：本仓 `rm_simulation.launch.py` 把车生成在 `z=0.2`、**LIO 在坠落中就做了重力初始化** ⇒ odom 的 z=0 比"落定后的地面基准"高 **0.2~0.35 m**，而旧的共轭平移**恰好**补回了这 0.2044 m 的一部分（"歪打正着"），所以今天的默认路径"能用"。修它必须**分两步**（平移 + 高度带重新定基都在 `src/nav/rm_navigation/**/params`，属别的任务目录） |
| **回退** | `git revert` 该 commit（默认档逐位不变）；只退 `sensor` 档 ⇒ 去掉 `<cloud_frame>` |
| **证据** | `Fidelity` §I.5.3/§J.1.2/§J.3/§J.5；`robot_models.md` §15 尾部；提交 `2a5a8b6` |

### §2.3.4 GICP 重定位：三道接受判据 + 环路内低增益滤波（治"自传播发散"与"到点后抖动"）

| 项 | 内容 |
|---|---|
| **现象** | 用户：「他喜欢贴着膨胀半径走而且容易撞击进去然后重定位出问题」；「到了点还在飘/四处抖动」。复现到的失败签名：`map→odom` 在一次 12.7 m 行驶里**单调漂到 y=+0.97 m**，而 **fitness 一直是 0.0022~0.0029 m²、100% 被采纳**（**对既有健康判据完全不可见**）；机器人**静止**的 120 s 里定位误差仍在长（p50 14.4 cm → max **47.8 cm**）；跑 12.7 m 实际走了 **29.7 m**、`/cmd_vel` 角速度换向 **35 次**、**15 次 recovery**；用户那次更极端：`map→odom` 到 **(4.98, 62.1, 8.28)** |
| **原因** | **不是"GICP 算错了"，而是"地图没有分辨力"**：本场地先验图（大片地板 + 规则重复墙体）让**任何位姿都能配得很"好"** —— 离线自相似探针（与节点**同一份代码路径**）：25/25 收敛、**偏 0.07~2.5 m 的位姿 fitness 仍只有 0.002~0.10 m²**，**远低于接受阈值 `max_fitness_score = 0.3`**；只有真值本身是"完美"极小（0.00022）。⇒ **单靠 fitness 永远分不出"真值"和"差 0.5 m 的孪生位姿"**；"每帧采纳 + 用它做下一帧初值" = 在一个连续极小（自相似平台）上**随机游走**，而初值来自 LIO ⇒ LIO 的误差直接喂进配准、跳盆地。**yaw 方向是可观测的**（yaw 扫描 25/25 收敛、离真值最大 0.035 m ⇒ 误差 < 3°）⇒ 失败总是"位置在自相似方向上滑" |
| **过程（判决性测量 + 被否候选）** | **① 运动一致性门限**（创新 = 本帧配准结果 ⊖ 本帧初值 = "GICP 这一帧想施加的修正量"）：阈值 `gate_trans_rate 3.0 m/s`（= MPPI `vx_max 2.5` 的 4 倍余量；健康跑行驶段逐帧步长 p95 = 74 mm）、`step_min 0.10 m`、`step_max 0.6 m`、`gate_yaw_rate 60°/s`（`wz_max 2.5 rad/s = 143°/s`）；`dt` 用**点云戳差**不用墙钟（墙钟过载会把非法大跳变"合法化"）。**② 合理性/定义域**：判据作用在**结果蕴含的 `map→base_link`**（不是 `map→odom`，因为"车在图里的位置"才是物理量）：`plausible_margin_xy 1.0 m`（PCD 包围盒 30.2×18.4 m）、`z ∈ [−0.9, +0.7]`（实测 `map→base_link` z ≈ **−0.30 m**；用户那次 `map→odom` z 涨到 **+8.28**）。**③ 连续拒绝 ⇒ 定位失效**：`lost_after_rejections = 25` 帧（10 Hz ⇒ 2.5 s）⇒ `estimate_valid_ = false`、停止配准、ERROR、等 `/initialpose`（**绝不编造位姿**）。**④ 环路内低增益滤波**。**被否掉的候选**：**紧门限**（0.8 m/s / cap 0.12 m / 20°/s）—— 实测拒绝率 **18%** 且拒的是**合法修正**（创新 0.1226 m / 0.538°、fitness 0.0024），发布值被冻住而里程计继续走 ⇒ 创新 0.31→0.50→0.95→2.24 m ⇒ 连续 25 帧判失效（**死锁**：越拒越偏、越偏越拒）；**"只平滑输出、初值仍取上一帧测量"** —— 实测无效（静止 120 s 漂 40 cm 照旧）；**更紧的 `max_correspondence_distance`**（1.5→0.8~1.0 m）—— **未测，不写进默认值** |
| **改动** | `src/localization/rm_localization/gicp_registration`：`GicpNode::pointcloudCallback` 的"接受/拒绝"这一步（**不改 GICP 数学、不改 TF 契约、不加任何节点/无 watchdog**）；发布值 `T_map_odom_` **同时是下一帧初值**，朝"最近采纳的测量"按 `α = 1 − exp(−dt/tau)` 推进（`tau = 1.0 s`；50 Hz 定时器 + align 帧末尾两条路径共用同一个 `steady_clock` 基准）；新键全部有默认值、非法值 WARN 回退；`gate_enable:=false` + `smoothing_enable:=false` = 改动前行为 |
| **原理** | ① **滤波必须在环路里**：配准答案在一小片连续极小里跳，"每帧把测量当新位姿"（增益 1）= 把跳变**积分**进去 ⇒ 静止也会漂；低增益 ⇒ 抖动 **~1/α**、随机游走 **~√α**；`tau=1.0 s` 的依据：采纳率 10 Hz ⇒ α≈0.095，而 LIO 慢漂移实测 1~3 cm/s ⇒ 滞后仅 1~3 cm（低于 LIO 自身噪声），跳变噪声被压 ~10 倍；② **门限只该切灾难性跳变，不该管噪声**（噪声交给滤波）；③ **"让失败可见"**：`~/converged` 现在 = `被采纳 && score ≤ warn` ⇒ 新判据拒掉的帧会让 `converged=false`；④ **高频运动仍由里程计提供**（本节点只发 `map→odom`，**绝不碰** `odom→base_link`） |
| **效果** | **静止 A/B（单变量、同一二进制，只切两个开关；150 s）**：`map→odom` **逐次更新步长 p95 0.0087 → 0.0005 m（17×）**、max 0.0543 → 0.0283；**@10 Hz 重采样 p95 0.0086 → 0.0005（17×）/ max 0.0161 → 0.0008（20×）**；yaw 逐次步长 p95 **0.127 → 0.008°（16×）**；**累计修正路程 4.085 → 0.247 m（16.5×）**；融合位姿逐样本跳变 p95 **0.0048 → 0.0020 m（2.4×）**、max 0.0129 → 0.0027（4.8×）；相对真值误差两次都稳定 **8.5 cm 且不随时间增长**（⇒ 改动前那 4 m 的修正路程是**均值回复的抖动**，真正的漂移发生在**行驶**时）。**行驶跑（a→b/c）的导航结局反而变差（SUCCEEDED → ABORTED）是上游 LIO 自己失控主导的，不能当验收**（真实收益 = "不再把假位姿喂给 nav2"）。**P0 回归**：PASS、采纳 **762/762**、**拒绝 门限 0 / 越界 0（连续 0）**、fitness 0.0027~0.0031、align 14~17 ms；附带收益："发目标前等 `map→odom` 稳定"的判据从旧文档的 **3.4 cm / 30 s** 降到 **0.0020 m / 5 s** |
| **回退** | `--gicp-kv "gate_enable: false" --gicp-kv "smoothing_enable: false"`（= 改动前行为），或 `git revert 4f29d35` |
| **证据** | `gicp_divergence_and_jitter.md` §0–§7；`path_clearance_and_contact.md` §2.5/§4.4；提交 `4f29d35` `642c174` `25e33ad` |

### §2.3.5 GICP 的 `/initialpose` 饥饿：一次点击 **22.7 s → 0.24 s**

| 项 | 内容 |
|---|---|
| **现象** | RViz 的 `2D Pose Estimate` 点下去"没反应"：实测一次点击 **13.2 s** 才被处理、另一次 **14 s 内完全没被处理**；本轮复现 **22.7 s / 22.6 s**，且**只在点云流停止后 0.6~0.7 s** 才被处理 ⇒ 饱和期间等于"永不处理" |
| **原因** | `/initialpose` 与点云订阅**同处一个 `MutuallyExclusive` 回调组**（align 组）；点云 10 Hz、PCL 后端单帧 align **400~490 ms** ⇒ 该组**接近 100% 占用**，点击回调要排在在飞的 align（以及后面排队的点云）之后 ⇒ **调度饥饿**（不是 RELIABLE 丢包：样本没超时、没丢） |
| **过程（判决性测量）** | ① 节点空转（无点云）时：**~15 ms**；② 饱和时：**≈13.2 s / 22.7 s**，并把"应用发生在停流后 0.63~0.71 s"对上 ⇒ 根因钉在**调度**上（同一探针、同一条 RELIABLE 消息，只换回调组）；③ 三个观测量（apply 日志 → `~/pose` → 下一个 `/tf` tick）互相吻合到 **~2 ms** ⇒ 量的不是"日志什么时候打"，而是"`map→odom` 什么时候真的变了" |
| **改动** | **handoff + apply 拆分**：点击回调只做 **POD 交接**（拷原始 pose/covariance/header + 置 `pending_initial_pose_valid_`）跑在**自己的第三个回调组**（不是 TF 组，也不是 align 组）；真正的"应用"（TF 查询 + `T_map←odom = T_map←base·T_base←odom` + 状态写入 + no-improve 清零 + 一条 `~/pose`）在**下一帧点云的帧首**执行 |
| **原理** | ① **"人手点"的话题不该与"高频数据"共享调度预算**：不放 TF 组是因为那个组的契约是"50 Hz TF/状态发布绝不被拖慢"（改前 `/tf` 掉到 2 Hz 就是它被拖慢的后果）；② **串行性不变量"保住且更强"**：写 `T_map_odom_` 的代码现在就在 `pointcloudCallback` 里（先应用、再配准，同一条回调序列），连"两个回调互相插队"的可能都不存在；③ **QoS 保持 RELIABLE**（改成 BEST_EFFORT 会在网络抖动时**静默丢点击**）——交接变便宜之后 RELIABLE 不再有"占着组"的代价 |
| **效果** | 饱和场景（PCL 400~490 ms/帧 @10 Hz）：**22.783 / 22.408 / 18.478 / 12.484 s → 0.162 / 0.303 / 0.263 / 0.197 s**（handoff 日志 **0.0003~0.0004 s**，`~/pose` 与 `/tf` 生效与 apply 差 ~20 ms）；轻载（400 点云）：before 0.040 s / after **0.041 s** ⇒ **没有引入额外开销**；`Release(-O3)` 口径：before **0.000 s**（bug 被 `-O3` 大幅掩盖）/ after **0.121~0.137 s**（上限 = "到下一帧点云开头"）⇒ **修复后的延迟与 align 成本解耦**，修复前是**无上界**的 |
| **回退** | `git revert c7bcd6d`（会回到"点一次没反应十几秒"）；注意这不是"性能问题"而是**可用性问题** |
| **证据** | `gicp_initialpose_latency.md` §1–§4.3/§6/§8；提交 `c7bcd6d` |

### §2.3.6 LIO 漂移诊断：正常工况 **1 cm/m**、唯一复现的失效模式 = **高角速度甩转（抛硬币）**

| 项 | 内容 |
|---|---|
| **现象** | 用户：「才跑一小会儿里程计就飘」；另有"续建之后位姿飘"的怀疑 |
| **原因（四个候选逐条判决）** | **(a) 正常累积** ❌ 解释不了"才一会儿"：直线 **0.0105 m/m**、原路返回 **0.0104 m/m**、窄沟 **0.0031 m/m**；整程 ATE RMSE **0.036 m** / max 0.096 m（51.4 m / 408 仿真秒）。**(b) 卡死/几何退化** ❌：窄沟段 `/scan` 有限束数最小 **115**（其余 ~850）、车速只有指令的 **62%**、两段都撞墙钟上限，但该段 ATE RMSE **0.024 m**、**0.0031 m/m**；逐秒误差增量 vs 束数 **r = 0.065**、vs RTF **r = −0.026**（≈0）。**(c) 转弯诱发** ✅ **唯一复现出来的失效模式**（**与地图冷热无关**）：温和转垃圾无害（180°@0.32 rad/s 增量 −0.003 m、慢转@0.162 −0.0004、快转@1.104 −0.008 m），**但进到高角速度工况就是抛硬币**：同一档甩转（指令 3.8~6、实际 0.8~1.9 rad/s）**6 次里坏 3 次**，坏起来从 LIO max **0.62 m** 到位置跑飞 **10.68 m**（`map→odom` 甩到 13.34 m）；**最冷的一次（启动后 15 s）反而最干净 0.042 m，最暖的一次（走过 34 m）坏了** ⇒ 不是"地图没长好"。**(d) 续建污染存档的假象** ❌：续建的 `map→odom` **全程恒等**（0.08, 0.02, −0.014）直到 426 s；每一段 ATE RMSE 0.014~0.053 m、每米 0.004~0.015 m，与从零建图**没有差别**。**污染是真的**（只读复测 PCD 的 z 到 **+23.10 m**），但它**不在实时位姿链路上** |
| **过程（判决性测量）** | ① 分段脚本化路线（6 类：直线/原路返回/慢转/快转/窄沟/角速度扫频）+ **50 Hz 同步时间轴** + 逐段 ATE；② 对齐口径：用**起步静止窗口**估一次常量刚体变换、**全程沿用**（**绝不逐段重对齐** —— 那会把漂移当坐标系差减掉）；③ **段序本身是实验设计的一部分**（破坏性段必须放最后：第一版把甩转放最前 ⇒ 后面所有"干净段"结论全部作废）；④ 两条跑法护栏（上一轮 `gzserver` 没死干净 ⇒ 量到的是上一轮那台车；不要同时跑两个仿真） |
| **改动** | **只加工具与文档**（`tools/scripts/diag/`），**没有改算法**；本报告的最大价值是一条**作废声明**（见下） |
| **原理** | ① **"抛硬币"型失效要按"必要条件/充分条件"拆**：冲击是必要条件，但决定成败的是"这一次有没有让扫描匹配连续失配"（`conv_false` 采样数：发散的三跑 384/948/966，没发散的 5 跑**全 0**）；② **采样饥饿会造出假结论**（单线程 `rclpy.spin_once()` 在 ~80 Hz `/tf` 洪水下被饿死 ⇒ "LIO 位移是真值 2.6 倍"的假结论）；③ **把"当时以为的"写进文档并标注作废**，比删掉更有用 |
| **效果** | 给用户的直接回答：**正常开就不该飘（1 cm/m）；出现高角速度原地旋转 ⇒ "很快就飘"完全合理**（6 次坏 3 次、0.6~10 m / 几十秒，而且 `map→base_link` 会跟着一起飘，**slam 救不回来**：LIO 崩时融合位姿与 LIO **1:1 同步**，比值 0.869）；**不要把"续建存档被污染"当成实时位姿飘的原因**（它坏的是 3D 点云资产） |
| **回退** | 不涉及（只加工具/文档）；作废的旧结论（"要 1.9 rad/s 得给 6 rad/s"）**不要再引用**（§1.2 第 7 条） |
| **证据** | `lio_drift_diagnosis.md` §0/§2.3/§4/§5/§6/§7；`slam_toolbox_scan_drops.md` §0 的 2026-10-06 后续；提交 `e866cfa` `7d03f16` |

### §2.3.7 slam_toolbox：**节点加密让回环真的触发**（0.797 → 0.313 m）+ 丢帧裁决

| 项 | 内容 |
|---|---|
| **现象** | 用户：「slam_toolbox 的配置怎么改，建图漂移能小一点 / 能自己纠回来？」；每次建图都有 `queue is full` 丢帧日志 |
| **原因** | ① **回环为什么常常不触发**（三道闸门，**主闸门是"节点太稀"**）：`loop_match_minimum_chain_size = 10` 要求 **10 个连续的旧节点**同时落在 `loop_search_maximum_distance = 3 m` 半径内；而实测**节点间距 0.80 m**（= 1.5×`minimum_travel_distance`）⇒ 10 个连续节点要跨 **≈8 m**，**几何上不可能塞进 3 m**；二级闸门 `LinkNearChains` 更紧（1.5 m 内要 10 个连续节点）。② **单帧能纠多少**：`correlation_search_space_dimension 0.5` ⇒ 平移搜索 **±0.25 m** ⇒ **LIO 一次跳 >0.25 m，逐帧匹配救不回来，只能等回环**。③ 丢帧的直接因**不是**回环（见 §1.2 第 8/9 条） |
| **过程（判决性测量）** | 四个配置的实测（同一路线 9.9 m 矩形环、同一速度）：**A2 现状**：位姿图 13 节点、环秩最多 1、全跑只有 1 次"疑似回环"、`err_map ≡ err_lio`（差 ≤3 cm）；**B（只把 chain 10→4）**：**环秩全程 0** —— 一次回环都没有；**C（只把节点加密：`minimum_travel_distance 0.5→0.2` 等）**：**32 节点 / 环秩 8 / 6 次疑似回环**，末段 `err_map 0.045` 优于 `err_lio 0.082`；**D（chain4 + 半径 4 + 阈值放松，但节点间距不变）**：环秩只到 **2**。⇒ **只有"加密节点"真的让回环触发** |
| **改动** | `mapper_params_online_async_sim.yaml`：`minimum_travel_distance 0.5 → 0.2`、`minimum_travel_heading 0.5 → 0.2`（该版本里其实是**死参数**，PR #888：`shouldProcessScan` 根本不看它；改它只为"应用的就是被测过的那一组"）、`minimum_time_interval 0.5 → 0.25`；**不碰** `loop_*` 全组、`correlation_search_space_dimension`、`scan_queue_size`、`max_laser_range`、`transform_*`、`map_update_interval`；也**不碰** `mapper_params_localization_sim.yaml` |
| **原理** | ① **先考证"这个参数到底有没有人读"**（`mode` 在 2.6.10 里是"半死"参数：唯一读它的是 Ceres 插件、只做一件事 `enable_fast_removal`；"建图/定位/终身建图"由**起哪个可执行文件**决定；本机**没有** `lifelong_slam_toolbox_node` 可执行文件 ⇒ COD 队的 `mode: lifelong` 实际跑的就是普通 async 建图）；② **漂不是配置被改坏了**：我们的 `mapper_params_online_async_sim.yaml` 与上游出厂版**逐项相同**（只差 `max_laser_range` 10 vs 20、`base_frame`，外加缺 4 个无关项）；③ **"没有回环 ⇒ 位姿图永远不做全局优化"**（`MapperGraph::CorrectPoses()` 全仓只有两个调用点：`TryCloseLoop` 与 RViz 手工回环） |
| **效果** | 节点间距 **0.797 → 0.313 m**；环秩 **1 → 8**、回环修正事件 **0~1 → 6**（同一路线/速度/3 分钟）；同路线地图占用格 **1613 → 1893**、已知格 **32549 → 36714**（墙面更实）；**代价测不出来**：RTF **0.80 → 0.76**、`gzserver` CPU% **52.6 → 52.7**。丢帧裁决（另一份报告）：**回环不是丢帧的原因**（六格里没有一条丢帧落在回环 ±2 s 内，随机基线 1.5%）；**"更好的里程计"买到的不是精度**（ATE RMSE 已 **0.021 m**，而真值 0.000 m）、**而是 TF 的时延与稠密度**（TF 比同戳扫描晚 **7.7 ms**，阈值 132 ms ⇒ **17 倍余量**；把它人为拖到迟到 **130 ms** ⇒ 丢帧 **0 → 4.26%**）；⚠️ **`scan_queue_size` 别当解药**（队列 2 ⇒ `queue is full` 62→0，但换出 70 条另一种丢帧、位姿图节点 58→35） |
| **回退** | 三个键改回 0.5/0.5/0.5；或 `git revert 5cb93cb` |
| **证据** | `slam_toolbox_tuning.md` §0/§1/§2/§3/§4；`slam_drops_loopclosure_odom.md` §0–§5；提交 `5cb93cb` `d3dde82` `3a2b54d` `d0fe0a0` `c7260da` |

## §2.4 导航 / 限速层

### §2.4.1 前瞻坡度/台阶限速：连续量 → 速度上限（**单一阈值源**、零新增节点）

| 项 | 内容 |
|---|---|
| **现象** | 用户：「他喜欢贴着膨胀半径走而且容易撞击进去」；实测**车以 ~1.7 m/s 冲上 `map(−3.6,3.1)` 的坡脚时 IMU \|a\| 冲到 79~90 m/s²（≈8~9 g）**（静止基线 ≈9.8）；这一下把 LIO 打散（`loc_err` 0.06 → 2.2 m 用 2.4 s，之后**车已停住**它仍继续跑到 2000 m+） |
| **原因** | ① 场地的**坡道面**（坡度 ≤22%）被 `linefit`（`max_slope ±0.4`/`max_dist_to_line 0.05`）当成"地面"整片删掉 ⇒ `/segmentation/obstacle` 没有它、`/scan` 没有它、2D 先验图也没有它（实测**静态图判"自由"而场地高度 >0.10 m 的格 88 576 个 = 自由区的 51.6%**）⇒ 规划器无从绕、控制器无从躲；② 物理：刚性底盘（`planar_move` 是**速度控制**、无悬架）在 ~2 ms 内被要求获得 `v·slope ≈ 0.17 m/s` 的垂向速度 ⇒ **`a ≈ v·tanθ/Δt`** |
| **过程（判决性测量）** | ① **标定锚点**：用实测 `v ≈ 1.7 m/s`、10% 坡、IMU **79~90**（取 86）反算 **`Δt_eff = v·tanθ/a = 1.98 ms`**；② **目标**：把预期峰值压到 **30 m/s²**（= 实测发散那一击 86 的 1/3）⇒ `v ≤ 30×0.00198/tanθ = 0.0593/tanθ`；③ 离线合成扫掠（`bench/slope_speed_offline.cpp`，不需要 Gazebo、同一份判据代码）定 `brake 0.4 m/s²`（0.8/0.5/**0.4**/0.3 ⇒ 过坡脚 1.18/0.90/**0.70**/0.65）；④ 整栈 A/B（`ssl_off` vs `ssl_on3`，同目标 `(-12.64,-0.31)`） |
| **改动** | ① 判据层扩展（header-only `rm_ground_traversability`）：同一帧的粗格缓存上再算**"车前/车后 1~3 m 走廊"**的连续量（局部坡度、**坡度变化** `dtan`、**台阶残差** `step`）；② `SlopeSpeedLimiter::update()` 三条规则取最紧 + 刹车距离界 + 速率限制；③ 发 **nav2 原生 `nav2_msgs/SpeedLimit`**（`controller_server` 订阅 `speed_limit_topic` → MPPI `setSpeedLimit()` **按比例缩放 `vx_max/vx_min/vy/wz` 四个约束**）；④ **单一真源** = `src/bringup/rm_nav_bringup/config/traversability_criteria.yaml` 的 `speed_limit_*` 一组键（另有一份同样的表在 `slope_speed_limit.hpp`，`check_slope_speed_table.py` 断言两处逐键一致 + 与 nav2 的 `vx_max`/`max_decel` 对账） |
| **原理** | ① **速度表**：`v(d) = sqrt(v_req² + 2·brake·d)` 是"以 `brake` 减速就正好在 d=0 降到 `v_req`"的运动学反解 ⇒ **是"过坡脚前开始减速"，不是"在坡脚上减速"**；② **必须"已承诺特征 + 距离前推"**：雷达**看不见车下与近处地面**（MID360 下视 −7.22°、离地 0.226 m ⇒ 平地最近看得见 **1.78 m**），所以测量到的特征距离**永远 ≥1.0~1.8 m** ⇒ 只用"距离刹车界"时上限永远降不到 `v_req`（离线扫掠：过坡脚仍 1.2~1.4 m/s）；做法 = 承诺（记住 `v_req` 与距离）→ 每帧按 `上一帧上限 × hold_decay_factor(1.0) × dt` 前推 → 前推到 0 才释放；③ **绝对坡度规则只在"没有承诺"时参与**（否则会把已承诺的减速顶回 ~1.4 m/s）；④ **速度地板不许配 0**（`speed_limit_floor_mps 0.45`，校验脚本会拦）；⑤ **前瞻用"朝向走廊"而不是"沿规划路径"**：这两个地面分割节点**不订阅 `/plan`、不查 TF**（那是它们"零额外故障面"的前提），而云本身在传感器帧里、livox 安装 rpy=0 ⇒ 帧的 +x 就是车头；⑥ **抗噪**：格内抬升取细格最高点的 **p90**（不是 max：实测 max 噪声可达 0.15 m）、定位型特征要过**支撑格数**（同一 0.40 m 距离带内 ≥2 格一致）、`Δ坡` 死区 4°、台阶死区 0.04 m |
| **效果** | 整栈 A/B（`ssl_off` → `ssl_on3`）：IMU 全程峰值 **110.8 → 50.8 m/s²（−54%）**；IMU>30/>50 样本 **7/3 → 1/1**；两次**都到点**（status=4，最近 0.25/0.23 m）；`|v|` p50/p90/max **1.23/1.82/1.88 → 0.71/1.36/1.87 m/s**；`|v|<0.6` 时长 **4.8 → 9.4 s**；限速器行为：362 行 `[slope_speed]`、limit p50 0.85 / min 0.45 / max 2.00、**不限速占比 9%**、`why` = slope_change 122 / step 197 / slope 12 / none 31；**平地回归**：限速开/关都 PASS、无 recovery，用时 **3.2 → 3.6 s**（+0.4 s）、真值 max\|v\| 1.54 → 1.19 m/s |
| **没走通 / 没帮上（别重复走）** | ① **"标障碍"这条路**：坡面本身可行驶，把它标成障碍是错的；实测只把第一击 86.4 → 68.2 m/s² ⇒ **必须限速，不能靠 occupancy**；② **只朝前看的走廊**：两次整栈跑里最重的一击都发生在**倒车**时（`v = −1.04/−1.00 m/s`），车尾后面的坡脚不在走廊里 ⇒ 第一版峰值 66.5/67.7/86.5；③ **"拿 `/odom` 的 `twist.linear.x` 符号判运动方向"这条路没走通**：机器人确实在倒车（`/cmd_vel` 与真值都是负的），但 `[slope_speed]` 一直打"前进" ⇒ **方向判据没有生效，根因未定位**（订阅建了吗？时间戳/时钟？符号约定？）⇒ 默认改成**双向窗口**（不依赖任何话题语义）；④ **调 nav2 的全局限速**（`velocity_smoother.max_velocity 2.0→1.0`）不是替代方案：它不看地形、平地也慢 |
| **回退（三种粒度）** | ① 只关限速：`speed_limit_enable: false`（注意本节点是**构造期**读参数，`ros2 param set` 运行时**不生效**，要重启节点或改文件）；② 连判据一起关：`traversability_enable: false`；③ **残留上限的手工恢复**（节点被 SIGKILL 时最后一次上限会留在 MPPI 里；正常退出会自己发回 `NO_SPEED_LIMIT`）：`ros2 topic pub --once /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.0}"` |
| **证据** | `slope_speed_limiting.md` §0–§7；`traversability_plan.md` §9；`path_clearance_and_contact.md` §7.1；提交 `ca89b69` `bb0fe03` `71f6c87` `fc226bb` `72155e4` |

**★ 提速档（用户嫌慢之后的下一步，`lio_divergence_no_impact.md` §3/§4）**

| 键 | 改前 | 改后 | 依据（实测） |
|---|---|---|---|
| `speed_limit_peak_target_mps2` | 30.0 | **45.0** | 唯一直接由"用户嫌慢"推动的物理量；45 < 实测最小致损簇（>50 那一档）；实测峰值 50~88 出现在**限速已生效**的旧表跑里 ⇒ 30 这个目标值本来就没把峰值压到 30 附近 |
| `speed_limit_floor_mps` | 0.45 | **0.60** | 限速器 **96% 时间在触发** ⇒ 地板就是巡航速度下界 |
| `speed_limit_slope_change_deadband_deg` | 4.0 | **5.5** | `why=slope_change` 占限速行 **56~79%**（4~5.5° 的场地起伏被当特征）；真坡脚 Δ坡 实测 **9~16°**，不会漏 |
| `speed_limit_step_deadband_m` | 0.040 | **0.060** | `why=step` 占 13~33%；0.06 m 仍远小于判据自己的台阶闸 0.15 m |
| `speed_limit_slope_knots_mps` | `[1.66,1.10,0.83,0.66,0.55,0.45,0.45]` | **`[2.48,1.65,1.24,0.99,0.82,0.61,0.60]`** | 按 `v = round_down_2(45·Δt_eff/tanθ × 0.975)` 重算，反算峰值 43.4~43.9 ≤ 45 |
| `speed_limit_step_knots_m` / `_mps` | `[0.040]` / `[0.45]` | **`[0.060]` / `[0.60]`** | 与台阶死区同步（两处必须一致） |

**提速档的效果（最硬的一条 = 结局）**：旧表 **0/4 到点**（1 次 ABORTED + 3 次超时，`|v|<0.6` 占 61~73 s），
新表 **2/2 到点**（都在 ~27 s 里走完 22.6 m，计划 23.4 m），`|v|` p50 **0.05 → 1.16/1.19 m/s**、
`|v|<0.6` 时长 **61.6 s → 0.3/0.6 s**、recovery **22 → 0/4 次**、`conv_false = 0`、`mo_stale = 0`、
`lio_err max ≤ 0.54 m`、单帧 step ≤ 0.201 m（= 2.0 m/s，正好是底盘上限）。
**但"更快的表更安全"不能说满**：地形冲击簇**每跑都有**（峰值 57~192 m/s²），
`f2` 那次峰值 **192.2**（全部跑里最高）却到点且没发散，`nb3` 只有 67.2 却发散了
⇒ **分得开的是"GICP 有没有被打进拒绝级联"**（`conv_false`：发散的两跑 384/966 vs 没发散的全 0）。

### §2.4.2 `--goal-forward 2.0` 走不到的归因：六个候选逐条判决 + **可达性前置判据**

| 项 | 内容 |
|---|---|
| **现象** | `robot:=robot11` 短目标能走（0.5 m）、**长目标（2.0 m）不动**；控制器侧一堆 `detected collision ahead` / `Controller patience exceeded` / 自旋恢复 |
| **原因（判决：实体障碍 + 角速度通道，**两个独立机制**）** | ① **根因 = 正前方 0.42 m 处有实体障碍**：把 nav2 **完全摘掉**、只给底盘插件发固定 `vx=+0.25`（25 s、500 条指令、请求 6.25 m），robot11 **只能前进 0.4199 m**（之后 20 s 一动不动，最后一次 >2 mm 的运动在 t=5.32 s）；**默认模型同协议只有 0.5402 m** ⇒ **不是 robot11 特有，是场地/出生点的性质**。② **"0.5 m 能走"是容差成功**：真值只走 **0.2977 m**、停在离目标 **0.1902 m** < `xy_goal_tolerance 0.25`。③ **控制器侧全是结果不是原因**：`/plan` 10 条正常、`/cmd_vel` 真的发过 `vx=1.0`（89 条 >0.01）、车也真的动到 **0.4197 m** 就推不动（与盲推极限 0.4199 m 差 **0.2 mm**）。④ **地图侧两个真实现象（不是根因，但会改变失败的形状）**：`slam_nav` 下全局图只覆盖已建出来的那一块（目标在图外 ⇒ `off the global costmap`），而 `mode:=nav`（完整先验图）对同一个目标直接 **0 条 plan**（`failed to generate a valid path` ×10；目标格 = 31/inflated、窗内 23 个 lethal、到 ≥99 格 0.158 m）⇒ **完整地图也说不通** |
| **过程（判决性测量）** | **六候选逐条**（`Fidelity` §L.6）：① **地图/空间** ⇒ **部分成立但不是根因**（`slam_nav` 有 10 条 plan、末点精确落在目标上，只是路径**穿过尚未建出 lethal 的障碍**；`mode:=nav` 反而拒绝规划）；② **控制器/规划器参数**（`regulated_linear_scaling_min_radius 0.9`）⇒ **反驳**（实测 `/cmd_vel` 发到 **vx=1.0**，远高于任何"限速"；且**默认模型用同一套控制器参数也一样失败**：0.674 m 后 20 s 只动 5 mm）；③ **BT/恢复** ⇒ **反驳（是结果）**（FollowPath **进了** 45 次 EXECUTING、`number_of_recoveries` 最大 **7**、恢复序列跑满，但都发生在**车已经被推不动之后**）；④ **局部代价图/滚动窗口** ⇒ **反驳**（目标**在**局部图内 91/91；局部图前方 0.15–0.80 m 就有 ≥99/lethal 带，**与物理一致**）；⑤ **出生自转 ~10°** ⇒ **成立但换了归因**（robot11 的 yaw 在 60 s 内自由漂到 **+8.20°**（默认模型 −0.17°），而**指令角速度只有 0.55% 被执行** ⇒ "自转"是**不受控的漂移**）；⑥ **（本轮新增，决定性）场地里的实体障碍** ⇒ **成立** |
| **三处独立的几何自洽（让"实体障碍"从猜测变成测量）** | ① 前进极限 + 车体外接半径 = `0.4199 + 0.3565 = **0.776 m**` ≈ 在线代价图里 x≈0.78 的那条 lethal 列；② 后退极限 + 外接 = `1.5753 + 0.3565 = **1.932 m**` ≈ 先验图 `RMUL2026.yaml` 的西墙 x≈−1.9；③ **同一跑内部**：导航跑里车被推到的峰值真值位移 **0.4197 m** ≈ 纯物理推的极限 **0.4199 m**（差 0.2 mm） |
| **改动（新验收口径）** | `nav_goal_forensics.py` 新增：**`--preflight-reach`**（发目标**之前**先用绕过 nav2 的盲推量出"这个出生点物理上能走多远"）、**`--reach-gate`**（目标超出实测可达区就**不发目标**，在 JSON 里记 `goal_gate`）、**`--goal-yaw-only-deg`**（原地转目标）、**`--dump-traces`**（真值/里程计完整时间线 —— "无指令漂移"只能从它读）；新工具 `run_nav_goal_forensics.sh` / `nav_goal_report.py`（把 `forensics.json` 读成判决表） |
| **原理** | **"任何基于目标的验收，必须先跑一次盲推可达性探针"** —— 在**不可达的目标**上，"发目标 → 看车动不动"永远是红叉，而红叉的原因与控制器无关。判据（本出生点实测）：`reach = {fwd_m: **0.3992**, back_m: **1.4370**}` ⇒ 可达区是"前 0.40 m / 后 1.44 m"，`--goal-forward ±2.0` **都在区外** |
| **效果** | 反向 **1.2 m**（在可达区内、闸门放行）：真值位移 **1.2102 m**（请求 1.2）、残余 **0.0794 m**（< 容差）、**`Goal succeeded`**、`/cmd_vel_chassis` 的 `max|vx| = 0.9624`、`n(vx>0.01) = 121`、`collision ahead` 14 条、恢复 4 次；反向 **2.0 m** 加闸门 ⇒ `in_reach=false` ⇒ **不发目标**（0 plan / 0 条 `/cmd_vel` / 位移 0）；反向 2.0 m **不加**闸门 ⇒ 0 条 plan、8 次恢复、真值 0.0733 m、**Δyaw −179.7°**（恢复 Spin 这次真的把车转了 180° —— **角速度通道修好的直接后果**） |
| **回退** | 三个新工具是纯增量（**不进任何 launch/节点**）：`rm -f tools/scripts/tiltmount/{nav_goal_forensics.py,nav_goal_report.py,run_nav_goal_forensics.sh}`；`--preflight-reach/--reach-gate/--goal-yaw-only-deg/--dump-traces` **默认关**，回退不影响任何旧命令 |
| **证据** | `Fidelity` §L.0–§L.12（含 §L.1.3 的口径更正）、§M.8；提交 `94f0d6b` `de3b78c` `9de6b72` `339e205` |

### §2.4.3 两个"顺带查到"的缺陷（与上面的结论无关，但**必须登记**）

| # | 缺陷 | 判据性数字 | 现状 |
|---|---|---|---|
| ① | **本仓 nav2 不检查目标朝向**（`goal_checker_plugins: ["general_goal_checker"]` = `PositionGoalChecker`，只声明 `xy_goal_tolerance`；1.1.20 里没有 `yaw_goal_tolerance`） | 原地转 180° 目标 ⇒ nav2 **立即** `Reached the goal!`、`FollowPath SUCCEEDED`，`/cmd_vel` 与 `/cmd_vel_chassis` 的 `max|wz| = **0.0**`、真值位移 0.00012 m、**Δyaw 0.005°** | **没改**（参数在 `src/nav/rm_navigation/**/params`，按规则只许走既有槽位覆盖）。要让"原地转"变成真判据需换 `nav2_controller::SimpleGoalChecker` |
| ② | **`/odom_ground_truth.twist` 是"指令回显"，不是实测速度** | `planar_move` 的 `OnUpdate` 挂在 `ConnectWorldUpdateBegin` 上：**先** `SetAngularVel`、**再**在同一个回调里 `UpdateOdometry()` 读 `model_->WorldAngularVel()` ⇒ 该字段**恒等于刚设进去的命令值**（实测：真值 yaw 12 s 只转 **7.09°**，而该字段中位 = **57.296°/s = 1.0 rad/s**） | **要量"到底转了多少"只能用位姿差**（`/odom_ground_truth.pose` 或 `/odom`），**不能信 twist** |

### §2.4.4 贴膨胀边 → 撞击 → 定位失效：**"这次不是调参能修的"**

| 项 | 内容 |
|---|---|
| **现象** | 用户：「他喜欢贴着膨胀半径走而且容易撞击进去然后重定位出问题」 |
| **原因（三件事分工）** | ① **贴边**：`SmacPlanner2D` 的平滑器**没有代价项**（`smoother.cpp:100-190`：更新式只有 `w_data·(x−y) + w_smooth·(y₊₁+y₋₁−2y)`，代价只做 `cost>252` 的硬回退），而本仓 `w_smooth 0.4 / w_data 0.1`（4:1）⇒ **它把 A\* 用 `cost_travel_multiplier=4.0` 买来的余量又往内切带外沿拉**（计划路径到内切带距离 p50 只有 **0.25 m**）；② **致命一击与贴边无关**：撞的是**地图看不见的坡道面**（§2.4.1 的原因①）；③ **最要命的是"停发之后"**：`map→odom` 冻结后 nav2 仍用**控制器**（不是 recovery）继续开车 —— 最坏一次**盲开 110 m**、撞到 **474 m/s²**、足迹余量 min **0.05 m** |
| **过程（判决性测量）** | **11 次跑 / 9 个配置**（单变量）：`base2`（用户同参）⛔ 卡死、健康段余量 p50 0.750 / p05 0.347 / min 0.304、IMU **86.4**（7 个 >30）、定位误差 max 2.21 m、停发后行程 0.0 m；**`v_nosmooth`（`w_smooth 0.4→0.0`）** ✅ SUCCEEDED、计划到带 p50 **0.25 → 0.335**、执行余量 p50 **0.57 → 0.85 m**（健康段 0.75 → **1.03**）；**`v_ginfl`（全局膨胀 0.55→0.85 + `cost_scaling_factor 8→4`）** ✅ SUCCEEDED 且**唯一一次全程没挨重击**（IMU max 27、>30 样本 0）—— **但同一配置复跑 `v_ginfl2` 就灾难性失败**（IMU 76、盲开 **110.5 m**、定位误差 **12 883 m**、15 次 recovery）；`v_navfn` ✅ 成功但健康段 IMU 仍 **72.9**；`v_combo`/`v_mul12`/`v_mppiclr` ⛔ 更差；`v_rpp` ⛔ 69 s 只走 2.6 m（**没跟起来**，另一种失败）；`v_nolost`（`lost_after_rejections 25→0`）✅ 到点且 `mo_stale = 0`，但它**根本没触发那条策略** ⇒ **不能**用来证明"永不放弃更好"，它真正的用处是**反证"撞击 → 必然丢定位"不成立**（同样是 ≥80 m/s² 的撞击，有的跑散了、有的没散） |
| **改动** | **已应用**：`planner:=smac2d` 的 `smoother.w_smooth 0.4 → 0.0`（提交 `4a7996c`）。**建议但未应用**：让 2D 图别再丢"坡道面/低矮几何"（⇒ 后来由 §2.4.1 的判据层 + §2.2.6 的高度带部分落地）、给"定位失效"加**停车互锁**、`ObstaclesCritic` 的膨胀参数与局部图对齐。**明确不做**：放松 GICP 判据 / 改成"永不放弃" |
| **原理** | ① **"贴边"与"卡死"不是同一件事**：`v_mul12` 把 A\* 余量做得最漂亮（健康段 min 0.350 m、边缘<0 为 0%）**结局照样卡死**（定位误差 16.2 m）⇒ 只治贴边不治链；② **撞击不必然致命，但它是这条链唯一的入口**：把 11 次跑按健康段 IMU 峰值分档，**9 次 ≥70 m/s²**，其中 3 次仍成功 ⇒ LIO 会不会被撞散是**概率性**的；③ **"停发 TF 之后没人踩刹车"是被放大的主因**（575/581 个样本有 `/cmd_vel_nav` 控制器指令，不是 recovery） |
| **效果** | 见上表（`v_nosmooth` 的余量与 `v_ginfl` 的"唯一一次没挨重击"）；另有一条**独立佐证**（`map_opt` p05 **−0.61 m**：雷达看见了图上更近的几何） |
| **回退** | 把 `w_smooth` 改回 0.4（或 `git revert 4a7996c`）；其余都是"未应用的建议" |
| **证据** | `path_clearance_and_contact.md` §0–§8；提交 `1811149` `4a7996c` `6f0c27a` `acb93c1` `2e7cf61` `4da000e` |

## §2.5 资产 / 建图层

### §2.5.1 2D 先验的 canonical 化（"当前默认是哪一份"）+ PCD→2D 的坑

| 项 | 内容 |
|---|---|
| **现象** | `map/` 与 `PCD/` 里同时躺着**合成图、`/scan` 累积图、点云投影图、多次会话产物**；"到底哪一份是当前的"靠人记；建图时"坡道在 3D 点云里清清楚楚、在 2D 栅格图上基本看不见" |
| **原因** | ① 2D 图来自 `/scan`，而 `/scan` 是 `linefit → pointcloud_to_laserscan` 的产物（**传感器系窄高度带，改前有效 ≈0.05–0.33 m**）⇒ **坡度够缓的斜面被判成地面 ⇒ 从 `/scan` 里彻底消失**（实测坡道段有限束数只有其余段的 31%）；② 没有命名/回滚规范 ⇒ "提升一份新图"这件事本身没有留痕 |
| **过程（判决性测量）** | ① **同一把尺子**（`verify_low_terrain.py`，真值 = `RMUC2026.stl` → 0.05 m"最高可站立面"高度图）量**四张 2D 候选**：`RMUC2026.yaml`（提升后）边沿被表示 **52.71%**、撞击窗 `map(-3.6,3.1)±0.6` **31/31 = 100%**、可行驶斜面被标占用 **5.21%**；`.bak-20261007`（提升前）**49.44% / 28/33 = 85% / 3.29%**；`_spl`（`/scan` 累积）**37.43% / 32/33 / 4.05%**（未知占 60%）；`_cloud`（点云投影，**已否决**）**35.98% / 30/33 / 7.23%**（超 5% 判据）。② **同窗口逐格对照**（`map_prior_ab.py`，公共窗口 `x∈[−24.80,3.60] y∈[−8.92,6.98]`）：新图边沿 **49.44% → 54.42%**（多表示 208 个真值边沿格）、未知格 **34819 → 24743（多"看见" 25.2 m²）**、新窗口**完全包含**旧窗口；代价 = "坡面中段凭空多出来的真障碍" **35 → 55 格（0.09 → 0.14 m²，差 0.05 m²）**、两图互换各有 3~4 千格占用/自由翻转（两次独立建图的正常噪声）。③ 3D 先验同尺子（GICP 自相似探针）：点数 **178,543 → 203,352（+14%）**、`xy` 足迹 **415.6 → 449.3 m²（+8%）** |
| **改动** | ① 默认对**就地提升**：`map/RMUC2026.pgm\|yaml` = `RMUC2026_v3` 那次会话的 2D 图（`577x326 @ 0.05 m`、`origin [−25.2,−9.09,0]`、占用 **9769** / 自由 **146198** / 未知 **32135**）、`PCD/RMUC2026.pcd` = 同一次会话的 3D 云（**203,352 点 / 6.51 MB**、8 字段、体素 0.10 m）⇒ 2D 与 3D **同源同场次**；旧份留 `*.bak-20261007`（人工命名，与工具自动的 `.prev-<时间戳>` **刻意区分**）；② 退役的合成资产进 `map/attic/`、`PCD/attic/`（**只搬不删**）；③ 命名规范写进 `map_assets.md` §1：`<world>_<tag>` 一个 `<tag>` = 一次会话的全部产物（`.posegraph/.data/.meta.yaml` + `PCD/<world>_<tag>.pcd`） |
| **原理** | ① **"哪一份是当前的"必须由文档回答，不能由文件时间回答**；② **"就地覆盖、不要 rm 再建"**：`install/` 是**逐文件软链**，删了重建会让软链悬空；③ 提升要**两个判据同时看**（"该表示的表示了" 与 "不该占的没占"）—— 只看前者会把图越描越胖；④ **2D 图有固有上限**：全场台阶边沿覆盖率四张都在 **35.98%~54.42%**、都够不到工具写的 80% 判据（激光扫不到台阶立面的下缘）⇒ 这是**性质**不是缺陷（`traversability_plan.md`） |
| **效果 / 回滚** | 回滚 = 把 `*.bak-20261007` 覆盖回 `map/RMUC2026.pgm\|yaml` 与 `PCD/RMUC2026.pcd`（`map_assets.md` §4.4 给了逐条命令）；`map_archive.sh restore` 走 `.prev-<ts>` 那条链 |
| **证据** | `map_assets.md` §0/§1/§2/§3/§4；`mapping_2d_from_cloud.md` §0/§6/§11；`worlds.md` §4；提交 `7f9e354` `2192024` `c8d65f9` `95846c3` |

**★ PCD→2D 的四个坑（都是实测）**

| # | 坑 | 数字 / 判据 | 处置 |
|---|---|---|---|
| 1 | **`/scan` 累积图里坡道 100% 是 unknown 或干脆在图外** | 坡面 **1680/1680 = 100% unknown**；走廊 **2688 格全在图外**（整条 6.72 m² 在窗口外） | 改用 `pcd_to_nav2_map.py`（点云投影）：坡面 **98.8% free**、走廊 **97.3% free** |
| 2 | **判据必须"相对局部地面"** | 0.05 m 栅格；占用 = 相对**局部地面**的高度超过阈值（默认 **0.15 m**，与 STL 管线同口径）**或**"又陡又高"；**缓坡（≤25°）保持 FREE**；法向 `k=8`、`--ground-cell 0.20 m`、p05；窗口/原点沿用**出生点相对系**（`amcl initial_pose=(0,0)`、gicp `initial_pose=[0,0,0]` **不用改**） | 规划器对坡道/走廊 5 个目标：`/scan` 图 **2/5**（3 个直接拒绝）→ 点云图 **5/5 全部给出路径** |
| 3 | **累积点云的"垂向拖影"会让 3D→2D 投影大面积误占** | `PCD/RMUC2026_lt.pcd` 在真值完全平坦（0.000±0.001 m）的 `map(−5,2)` 处：单帧累积点云的 **z 跨度 0.154 m**、单个 0.05 m 细格内 z 跨度中位 **0.085 / p90 0.169 m** ⇒ 重投影后 `occupied` **122.6 m²**（旧 `/scan` 图只有 22.7 m²），其中 **94.7% 的占用格真值局部高差 ≤0.10 m** | **实时链路不受影响**（判据跑在**单帧**上、没有累积拖影）；要重出先验图时，**更稳的路线 = 直接存"跑图那次的 `slam_toolbox` `/map`"**（对垂向拖影免疫，实测 **10.1 m² 占用 / 可行驶斜面误占 1.82%**） |
| 4 | **"图被掰歪的那一下"要能立刻报警** | `map_odom_jump_gate.py`（盯 `map→odom` 单步增量、超阈值打 `WARNING` + 发 latched `~/jumped`，**故意只报警不拦**）实测抓到：4 次跑共 **6 次单步跳变**，最大 **Δxy = 11.95 m、Δz = 7.36 m、Δyaw = 45.1°、Δmax-rot = 69.4°** | **只报警不拦**的理由：拦就得当**第二个 `map→odom` 发布者**（破坏"每条边恰好一个发布者"契约） |

### §2.5.2 续建 / 存档 / 会话身份：把"这次 launch 是谁"挂到 ROS 图上

| 项 | 内容 |
|---|---|
| **现象** | 用户：「能不能就是我在上次基础上继续建，手动指定一个地图名字去覆盖之类的」「以后换地图换场地会不会有干扰」；以及一次**真实事故**：「`save` 把位姿图写到**别人那套会话**的名字上」 |
| **原因** | ① `mode:=mapping` 起来时若同名存档已存在，slam_toolbox 会**反序列化并接着建**（`map_file_name` + `map_start_pose`），走完再 `save` 就是同名覆盖 ⇒ 缺的是"**场地/出生点守卫**"与"**存档名解析**"；② 事故的机理：`save` **信任了 `map/.session.yaml`**，而那个文件可能是**另一个还活着的会话**写的（同一台机上并行两套栈） |
| **过程（判决性测量）** | ① 换场地续建**默认拒绝并终止 launch**（除非显式 `map_allow_world_mismatch:=True`），依据 = sidecar `.meta.yaml` 记录的 world/spawn（`map` 系是**出生点相对系**，换 world 就是换原点 ⇒ 串味的地图叠在一起 = **假墙**）；② 续建后**一次性一致性检查**（`map_resume_check`：对不上就喊，`strict` 直接收栈）；③ 事故复盘留了原文与处置；④ 覆盖前**自动备份**（`.prev-<时间戳>`，默认留 **3** 代）+ `restore/backups` |
| **改动** | ① `mode:=mapping` 支持续建（`0a9c5b5`）＋ `cloud_accumulator` 让 **3D 先验也能跨会话续建**（map 系累积 + 同名存档 + 同一份场地守卫，`8b14aec`）；② `map_asset_guard.py` + `map_archive.sh`（`save/info/list/adopt`，`10f2a84`）；③ **会话播报器**（把"本次 launch 是谁"挂到 ROS 图上，`9d180ff`）；④ **`save` 严格解析会话名**（**活栈优先 / 多会话拒绝 / `--check`**，不再信任 `.session.yaml`，`02306fb`）；⑤ `cloud_accumulator` **数据卫生**（高度带 / 跳变速度闸 / TF 退化帧跳过）＋写前体检与备份（`59eb110`） |
| **原理** | ① **"同名覆盖"必须有守卫**：地图资产的价值在于"它属于哪个世界/哪个出生点"，这个信息必须在**文件里**（sidecar），不在人脑里；② **"谁写的"要有证据链**：会话播报器 + 严格解析 = 把"猜"换成"读 + 拒绝"；③ **续建 ≠ 拼接**：分块照建，但每块之间用**同一个存档名续上**，不需要手工拼（替代旧的"分块 + 手工拼接"建议）；④ **续建污染的边界要说清**：污染是真的（PCD z 到 +23.10 m），但它**不在实时位姿链路上**（§2.3.6） |
| **效果** | 续建日志（t=0 就打）会明确写出"从零建图 / 续建 + 绝对路径 + 存档 world/spawn"；`save` 在"多会话"情况下**拒绝**而不是猜；`map_archive.sh info/list` 可查每一代 |
| **回退** | 续建 = 删/改名存档后重新起栈；`save` 严格解析 = `git revert 02306fb`（**不推荐**：那会恢复"信任 `.session.yaml`"的事故面）；换场地允许 = 显式 `map_allow_world_mismatch:=True` |
| **证据** | `continue_mapping.md` §0/§3/§7/§10/§11；`map_assets.md` §1/§4；提交 `0a9c5b5` `8b14aec` `10f2a84` `15d5c57` `133742d` `9d180ff` `02306fb` `59eb110` |

### §2.5.3 世界资产 `RMUC2026`（由附件 STL 生成的全场）

| 项 | 内容 |
|---|---|
| **现象** | 需要一个"RMUC 级别全场"用来跑建图/导航；附件是 `easystl.stl`（毫米单位、Z 朝上、29.15×16.05 m） |
| **原因 / 改动** | 三处相对 `RMUL2026_world.world` 的改动：`<scale> **0.001 0.001 0.001**`（本 STL 原生单位 = **毫米**，bbox 29150×16050×1730；不缩放就是 1000 倍大）、姿态 **`0 0 1.6413436 0 0 0`（无旋转、z 抬 1.6413436 m）**、模型 pose **`0 0 0 0 0 0`**（让 **mesh 米制坐标 == world 坐标**，地图/PCD/出生点全部同一个系）；一条命令复跑：`tools/scripts/world/stl_to_world.py`（`worlds.md` §4.4） |
| **过程（判决性测量）** | 真实建图产物 vs STL 几何：把 `/map_save` 存的 22,947 点真实点云平移到 world 系后 bbox `x[−14.64,14.45] y[−6.19,9.56]`（STL 是 `x[−14.575,14.575] y[−6.401,9.649]`）；0.05 m 栅格上 **87.5% 的"竖直结构"格落在 STL 障碍格 5 cm 之内**（mean chamfer **0.019 m**）⇒ **毫米缩放、z 抬升、模型 pose 三件事都被实测证明是对的**；四种形态实测：**建图 ✅ / 2D+AMCL 导航 ✅ PASS / 3D+GICP 导航 ✅ PASS**（含一条"定位仍在缓慢收敛"的打折项） |
| **原理 / 场地对本车的坑** | ① **地图 free 237.8 m²，但 `clearance ≥ 0.22 m` 的只有 178.6 m²（同一连通域 176.3 m²）⇒ 真正能走的 ≈ 176 m²（占 free 的 74%）**；② **两个半场之间最窄处通道宽 0.45 m**（车直径 0.44 m）⇒ 余量 5 mm/边、**实际过不去**；A\* 给出的过场路线长 **41.7 m（绕场边）**；③ 台阶 **0.20 m 与 0.30 m**（轮半径 0.06 m ⇒ **物理上上不去**）；④ 目标点应当选在**出生点所在的半场**，跨半场的目标请当作"已知不可达" |
| **回退** | 世界/地图都是文件级资产：`map/attic/`、`PCD/attic/` 里留着历史版本；重跑一条命令即可再生成 |
| **证据** | `worlds.md` §0–§6；提交 `7c0006b` `280888e` `eb7b811` `7e7b823` |

## §2.6 工具与方法论（**本期可复用的部分**）

### §2.6.1 四条已经制度化的做法

| 做法 | 具体是什么 | 本期最有说服力的例子 |
|---|---|---|
| **① 判决性测量（先给判据，再造仪器）** | 面对"是不是 X 造成的"这类问题，先问"**如果 X 成立，我应该量到什么**"，然后造一个**只读**仪器去量那个量；测量结果必须能**否掉**候选 | "缺外参导致 odom 被转 30°"这条假设（H1）：判据 = "把 `/cloud_registered` 按那条 TF **反变换**回去，如果 odom 本来斜，反变换后会变成 60°"⇒ 实测 **1.041°** ⇒ **假设被否**（`Fidelity` §I.5.1） |
| **② 绕过被怀疑的那一层** | 当"控制器/规划器/代价图"都可能是原因时，**把 nav2 整个摘掉**只留物理 | `--push 0.25`：robot11 只能走 **0.4199 m** ⇒ 根因是实体障碍，不是控制器（§2.4.2） |
| **③ 反向验证（防止"修好了但修坏了别的"）** | 每个"改好一个指标"的改动，都要问"**它有没有让另一件事变坏**" | `update_rate 1000 Hz` 让角速度 1.03% → 7.8%，**但**盲推 0.408 → **2.565 m**（车爬上障碍）⇒ 否掉（§2.1.7）；限速开了以后**真台阶仍然限速**（`why=step` 28/75 帧）才算没把功能关掉（§2.1.6） |
| **④ 未验证清单制度** | 每份报告末尾**必须**有"未验证/诚实清单"，写明"哪些是单跑、哪些是无头、哪些只是推断"；**禁止**把推断写成事实 | 本文 §5 就是把各文档的未验证项去重合并的结果 |

### §2.6.2 "默认路径逐字节不变 + 一键回退"是怎么做到的（三种机制）

| 机制 | 怎么做 | 验证方式（不是"我说没变"） |
|---|---|---|
| **槽位覆盖文件**（`robot:=` / `ground:=` / `local_obstacle:=`） | 只为该槽位生成一份**逐键副本**或**只含几何键的覆盖层**，launch 深合并后传给节点；其它槽位**原样返回公共段路径**（不合并、不写临时文件） | ① 单元级：同一份代码三种槽位下"传给 nav2 的路径/值"逐项列出（默认/hzmirm 仍 0.22，robot11 = 0.3565）；② 运行期：`ros2 param get` 回读（`robot_radius` 真的是 0.3565） |
| **开关缺省 = 旧语义** | 插件 `<tilt_rpy>` 缺省单位阵、`<cloud_frame>` 缺省 `parent`、`<range_from_origin>` 缺省"修好的语义"、`self_mask_enable` 默认 false、`obstacle_near_ground_m` 默认 0.0、`gate_enable/smoothing_enable` 默认 true… | ① **结构性论证**（"缺省分支就是 2026-10-07 起逐字节相同的代码路径"）；② **实测对照**（默认模型跑一遍，逐项与改动前同量级：如 6256/2739 点/帧 vs 6270/2707）；③ **渲染 diff**（`xacro` 两档去注释后逐字节相同、35410 B） |
| **生成物由生成器拥有** | `sentry_robot_robot11_sim.xacro` 头部写"本文件是生成的，不要手改"；生成器 `robot11_make_sim_xacro.py` 可**逐字节重算** | `python3 … --out .tmp/…xacro && diff` **为空** ⇒ "生成器与入库的 xacro 逐字节一致"；清单缺失或 `all_ok=false` ⇒ 生成器**直接报错**（不生成"看起来对但没人验证过"的模型） |

### §2.6.3 跑法纪律（每一条都是踩过坑换来的）

1. **一条 bash 调用 = 一个 PID namespace**（本仓 `bwrap --unshare-pid --die-with-parent`）⇒ **启动 / 驱动 / 收尾必须塞进同一次调用**，否则进程全清、量到的是残局。
2. **隔离四件套**：`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、按 tag 哈希占用的 `GAZEBO_MASTER_URI`、`unset DISPLAY`（无头）。
3. **只杀自己的进程**：按 `/proc/<pid>/environ` 核对 master URI 后再 kill；**绝不做全机 `pkill -9 -x gzclient`**（它可能杀掉别人正在跑的 run；⚠️ 用户那两次 `exit code -9` **不能**归因于它 —— **用户 2026-10-10 回忆是自己手动 `pkill -9`（因为卡）**，发送者始终未指认，见 §1.2 第 16 条）。
4. **publisher 必须等机器人稳定后再注入**：在"spawn + 插件加载"期间出现 publisher 时，实测 `gzserver` segfault（exit −11）/ `gzclient` abort（exit −6）+ spawn 服务超时（`Fidelity` §H.6）。
5. **采样不能饿死**：单线程 `rclpy.spin_once()` 在 ~80 Hz `/tf` 洪水下会被饿死 ⇒ 造出"LIO 位移是真值 2.6 倍"的**假结论**；正确做法 = `MultiThreadedExecutor` + `ReentrantCallbackGroup` + 回调只做 O(1) 赋值 + **独立落盘定时器** + 每行带 `*_age_sim`（`lio_divergence_no_impact.md` §1.1）。
6. **单位与口径要写死**：① **"12 s"是墙钟还是仿真秒**（RTF 0.35 vs 1.00 会差 3 倍 ⇒ 执行率一律用**仿真时间**归一）；② **RTF 不可比**（同一批里可比、跨批不可比：0.23/0.36/0.69 排序不可复现）；③ **度/弧度**（`gt_yaw_delta_deg` 多乘 57.2958 那个 bug）；④ **分带百分比的分母**（`/scan` 用"有限波束 833 条"，不是全部 1462 个 bin）；⑤ **同一帧**（`ground`/`obstacle` 两朵云是异步到的、帧号不同 ⇒ 离线分析只用**唯一自洽**的那朵 `raw`）。
7. **破坏性实验放最后**：会**不可逆带坏状态**的段（高角速度甩转）排到路线末尾，否则前面所有"干净段"的结论全部作废（`lio_drift_diagnosis.md` §2.2）。
8. **一次只改一项**：把 5 项一起改的那次（`missing_data_ray_length` + `hit/miss` + `num_range_data` + `optimize_every_n_nodes`）留存率反而从 49% 掉到 30%，指向我多加的两项 ⇒ **回滚、一次一改**（`issues_and_findings.md` #26 七次结论）。

### §2.6.4 本期的工具地图（只列"能被别人复用"的）

| 目录 | 工具（举例） | 干什么 |
|---|---|---|
| `tools/scripts/regress/` | `robot_model_probe.py`、`run_robot_model_probe.sh`、`robot11_{mesh_inventory,geometry,gz_collision_bench,livox_fov,self_mask,make_sim_xacro,weld_chassis,decimate_visuals}.py`、`check_robot11_visual_slot.py`、`verify_low_terrain.py`、`check_slope_speed_table.py`、`check_traversability_criteria.py`、`analyze_slope_speed_ab.py`、`linefit_{offline,replay}_probe.py`、`map_prior_ab.py` | 模型资产量测 / FK 复算 / 断言 / 离线判据台架 / 低地形验收 / 限速真源对账 / A-B 指标提取 |
| `tools/scripts/tiltmount/` | `tilt_mount_probe.py`、`run_tilt_mount_probe.sh`、`tilt_mount_compare.py`、`tilt_chain_check.py`、`tilt_ab_table.py`、`tf2_compose_order_test.cpp`、`costmap_input_dump.py`、`near_ground_replay_probe.py`、`ground_leak_probe.py`、`p2l_forecast.py`、`nav_goal_forensics.py`、`nav_goal_report.py`、`run_nav_goal_forensics.sh`、`run_chassis_yaw_bench.sh`、`chassis_yaw_probe.py` | 斜装"帧/射线/外参/外方位"取证、逐帧点账本、代价图圆/分带、**绕过 nav2 的盲推**、可达性前置判据、底盘角速度台架 |
| `tools/scripts/diag/` | `nav_lio_timeline.py`、`run_nav_lio_timeline.sh`、`analyze_noimpact_divergence.py`、`map_odom_jump_gate.py`、`segment_drive.py`、`lio_drift_analyze.py`、`run_drift_diag.sh` | 50 Hz 只读时间轴 / 归因分析 / `map→odom` 跳变哨兵 / 分段驱动 + 逐段 ATE |
| `tools/scripts/mapping/` | `coverage_drive.py`、`map_archive.sh`、`map_asset_guard.py`、`map_resume_check`、`pcd_to_nav2_map.py`、`mapping_ab_run.sh`、`ramp_section_probe.py` | 跑图 / 存档与守卫 / 续建一致性检查 / 点云→2D / 建图 A-B |
| `tools/scripts/localization/` | `run_nav_smoke_regress.sh`、`nav_smoke_regression.py`、`compare_regress_snapshots.py`、`run_gicp_nav_ab.sh`、`gicp_nav_watch.py` | **P0 冒烟回归**（改任何东西之后都要跑）/ GICP A-B |
| `tools/scripts/world/` | `stl_to_world.py` | 场地 STL ⇒ world + 2D 图 + PCD 一条命令 |
| `tools/scripts/control/` | `improved_teleop.sh` | 键盘接管（⚠️ 松手必须显式发零速） |
| `bench/`（包内） | `slope_speed_offline.cpp`、`ground_seg_ab`、`gicp_selfsim_probe` | **不需要 Gazebo** 的离线台架（调参/验收的第一道闸） |

## §2.7 `tdt-nav-kit` 研究结论（**研究，未落地**）

> 触发 = 用户原话：「这个自己研制怎么自己研制呢，因为这个纯 c++ 吧，我们这个又算强依赖 nav2 吧，
> 做成 nav2 的插件吗，还是有办法保持纯 c++ 的形态…会不会影响原来纯 c++ 的运行效率呢」。
> 结论落在 `docs/nav2_vs_custom_planner.md`（**纯新增文档 + CMake 骨架，没有一行代码落地**，提交 `d4965bd`）。

### §2.7.1 三种接法与"该选谁"

| 维度 | **A. nav2 插件** | **B. 独立节点** | **C. 独立进程 + 共享内存/零拷贝 IPC** |
|---|---|---|---|
| 要写多少代码 | 一个薄适配类（Humble 的 `GlobalPlanner` 只有 **5 个纯虚函数**，其中 4 个可留空）+ pluginlib xml + 参数段 | 一个完整节点：TF/位姿、costmap 源、控制律、到达判据、恢复策略、生命周期 | B 的全部 + 共享内存/IPC 协议 + 进程编排 |
| 库还是纯 C++ 吗 | **是**（库一行不改，ROS 依赖只在适配层） | **是**（库编进 ROS 节点，但库本身无 ROS 头文件） | **是** |
| 每次规划的额外开销 | 一次虚函数调用 + action 收发（**都不是主要成本**）；costmap 是**指针原地读** | 取决于 costmap 从哪来：订 `costmap_raw` 会**序列化 + 拷贝**；自己跑 costmap 层则等价于 A | 一次 IPC 传输。**同进程**才能真零拷贝；跨进程只能靠 SHM/loaned message |
| 必须自己重实现 | 只在插件语义范围内（planner 只需 `createPlan`） | nav2 帮你做的一切（BT/恢复、目标管理、costmap 图层与 TF、`/cmd_vel` 独占、进度检查） | 同 B + IPC 编解码/一致性 |
| 与 nav2 生态 | 最好（RViz/`/plan`/BT/waypoint 全在） | 差（都要重新接线） | 中 |
| 何时选 | **默认选它** | 当 nav2 的**抽象与问题不匹配**（地形不是"空地/障碍"二值、底盘动力学不是差速/全向）时 | 只有在**已证明**规划耗时/抖动是瓶颈、且同进程不可行时（**本次没有找到任何论文/仓库用它跑规划器**） |

**推荐的一句话版**：把库保持原样（纯 CMake、无 ROS），在它旁边加一个 ament 包做薄适配；
**先做 `GlobalPlanner` 插件**（A\* 前端 + min-snap 后端，输出 `nav_msgs::Path`），
把"带时标的 min-snap 轨迹真正被执行"这件事留给后续的 Controller 插件或独立节点。

### §2.7.2 效率账（谁才是瓶颈）

| 项 | 量级 | 结论 |
|---|---|---|
| pluginlib `dlopen` + `configure()` | 生命周期期**一次** | 一次性开销，可忽略 |
| 每次规划的虚函数调用 + action 收发 | 纳秒~微秒级 | **不可能成为瓶颈** |
| **库自己的耗时**（上游公布） | A\* **平均 5.4 ms / 最大 14.8 ms**；min-snap **平均 3.9 ms / 最大 126.7 ms** | ⚠️ **min-snap 的最坏 126.7 ms 落在 20 Hz 控制环里会顶掉 2–3 拍** —— 这才是要担心的 |
| costmap 取用方式 | planner/controller 形态 = **指针原地读**；smoother 走 `CostmapSubscriber`（`nav2_msgs/Costmap` 的 `uint8[] data` = 560×300 = **168 000 B**，**每次更新都要序列化/反序列化一遍**） | 优先选"指针"那条路 |
| RMW/IPC、执行器抖动 | nav2 自己测过（文档给了来源） | 要 settle 必须在**我们机器上**测（文档 §3.6 直接给了实验设计清单） |

### §2.7.3 三条"接口语义级"的坑（**这条最值得记**）

1. **nav2 原生没有任何"带时间的轨迹"接口**（逐条证据）：① 消息层 `nav_msgs/Path` **只有 `header` + `poses[]`**，`nav2_msgs` 的 14 个消息里**没有 Trajectory 类**；② 动作层 `FollowPath` 只收 `nav_msgs/Path`；③ 接口层 `Controller` 只吐**一拍** `TwistStamped`，`Smoother` 是 `Path→Path`；④ **nav2 自己还会主动丢弃 Path 上的时间戳**：MPPI 的 path handler 把每个 pose 的 `header.stamp` **覆盖成当前位姿的时间戳**（`path_handler.cpp:71,86`）⇒ "把时标塞进 `poses[i].header.stamp`"这条路**在 nav2 自带控制器上会被无声抹掉**；⑤ 唯一"带时间"的东西是 MPPI **内部**的候选轨迹（对外只输出第一拍）；⑥ 别家怎么做：无人机栈干脆**不用 `Path`**（EGO-Planner 用自定义 `quadrotor_msgs/PositionCommand`，时间参数化直接进消息定义）。⇒ 本库的 `dt = 0.1 s` 是**隐式时标**（相邻点间距），一进 `Path` 就只剩"约定"。
2. **`0 = FREE` vs `0 = 障碍` 的值域陷阱**：nav2 的 master grid 是 `0 = FREE_SPACE / 253 = INSCRIBED / 254 = LETHAL / 255 = NO_INFORMATION`（`cost_values.hpp:42-46`），而本库的 `YAstar::setMap(int w, int h, u_char* mapData)` 把数据映射成 `x == 0 ? 1.f : 0.f` ⇒ **0 = 障碍、非 0 = 自由**（`yastar.cpp:404-417`，阈值 `occThs = 0.5f`）。**直接把 `getCharMap()` 喂进去 = 整张图取反**（自由变障碍）；另一个重载 `setMap(..., std::vector<int8_t>&)` 用 `x == 0 ? 0 : 1`，符合直觉但**会把 `−1`（unknown）当成障碍** ⇒ **必须自己写显式转换函数 + 一个单元测试**。
3. **`initCostMap()` 是硬前提**：库自己写着"调用 search 之前必须调用 `initCostMap()` 或其它设置 costmap 的函数"（`yastar.hpp:174-175`）—— **忘了不会报错，只会得到奇怪路径**（这类"静默失效"在本仓已多次出现，见 §3 的"静默失效"原则）。

---

# §3 原理与契约总纲（把"为什么这么改"提升成可复用的原则）

| # | 原则 | 一句话 | 本期最硬的例证 |
|---|---|---|---|
| **P1** | **帧是一等公民** | 同一份数据配上错误的 `frame_id`，会让"物理正确的模型"在 RViz/nav2 里看起来全错 | `urdf` 档：点云逐点相同（0.0004°）而 `/cloud_registered` 地面 30.970°（§2.3.2） |
| **P2** | **`frame_id` 是承诺，不是标签** | 写 `odom` 就必须真在 odom 里；写 `livox_frame` 就必须真在那个帧里 | 三个 `livox_frame` 的分辨（§0.1）；bug ② 的修复（§2.3.2） |
| **P3** | **契约 = 每条 TF 边 / 每个话题恰好一个发布者** | 多发布者 = 抖动 + 结论不可信；"拦一下"这类需求也不能靠抢发布权实现 | `map→odom` 单发布者、`base_link→base_link_fake` 双发布者被消除（P4）、跳变哨兵**只报警不拦**的理由（§2.5.1 坑 4） |
| **P4** | **参数归属 = 参数属于某个帧、某个原点、某一层** | "高度/半径/距离"这类参数**必须**说清"它属于哪个帧"；说不清就会调错对象 | `obstacle_layer` 高度带的真实语义（§2.2.5）；`p2l` 高度带跨帧换算（§2.2.6）；`robot_radius` 是内切还是外接（§2.2.2） |
| **P5** | **表示分离：视觉 / 碰撞 / 传感器 / 代价图是四种用途** | 同一个物体在不同用途下**应该**用不同几何/不同网格；把它们混成一个才是错 | CAD 原网格：视觉可以（但要抽稀）、碰撞绝对不行（§2.1.1）；视觉抽稀**不可能**改变点云（射线走 ODE、只与 `<collision>` 求交） |
| **P6** | **降维发生在哪一层是设计选择，但"信息丢失点"必须知道** | 一旦跨过某条边界（`p2l`）信息就永久丢失 ⇒ 想用那个信息就必须在边界之前动手 | 2D 链路上唯一能用"高度"的位置 = 近地剔除（§2.2.1）；`min_obstacle_height` 不是障碍物高度闸（§2.2.5） |
| **P7** | **刚体假设是物理引擎的前提，不是可选项** | `planar_move` 的 `Set*Vel` 只对单刚体自洽；"没有执行机构的自由度"是纯负担 | 底盘塌成单刚体：角速度 1.03% → 16.2%（§2.1.7） |
| **P8** | **先量后改；每个"修"前面要有一次判决性测量** | 判据先行（"如果 X 成立，我应该量到什么"），测量必须能**否掉**候选 | H1/H2/H3 三条假设各自被一次测量判决（§2.1.5）；六候选逐条判决（§2.4.2） |
| **P9** | **反向验证：改好一个指标不算完，要问"它有没有让别的变坏"** | 这正是"能跑通"与"没造假"的分界线 | `update_rate` 1000 Hz 让车爬上障碍（§2.1.7）；限速后**真台阶仍限速**才算没关功能（§2.1.6） |
| **P10** | **单变量 + 破坏性实验放最后 + 一次只改一项** | 多变量同改会让结论无法归因；不可逆状态必须先排到末尾 | 11 次跑 / 9 个配置的主表（§2.4.4）；甩转段放最后（§2.3.6）；五次同改那次的反例（§2.6.3 第 8 条） |
| **P11** | **唯一真源（single source of truth）** | 同一件事只在一处维护；"横幅/日志"必须**运行时读生效值**，不能写"意图" | `traversability_criteria.yaml` 的 `speed_limit_*` + `check_slope_speed_table.py` 断言两处逐键一致（§2.4.1）；横幅改成 `_YamlKeysReadout`（`robot_models.md` §12.4） |
| **P12** | **默认路径逐字节不变 + 一键回退 + 生成物有生成器** | 任何改动都要能回答"默认路径变了吗""怎么退回去" | 三种机制（§2.6.2）；`model://` 自动化里"包侧 export 是全局 env 增量"这条偏离被明确登记（§2.1.4） |
| **P13** | **仿真 ↔ 实车：先分三类再动手** | "改感知输出内容"（❌ 拒绝）／"消除仿真失真"（✅ 该做）／"世界本身的差异"（✅ 参数分家） | `sim_real_contract.md` §四/§五；"不许为了让仿真跑得动而改感知输出内容"（算力账要在仿真侧还） |
| **P14** | **优先怀疑"静默失效"，而不是"算错了"** | 本期的多数疑难都是"没报错但什么也没做/做错了" | 忘了 `initCostMap()` 只会得到奇怪路径（§2.7.3）；`gravity_aligned_frame` 一开就 0 点（§2.2.4）；`pure_localization` 写错层级 ⇒ 永不生效 + 析构 FATAL（`issues_and_findings.md` #15）；队列深度只是把"等待"吸收掉（§2.3.1） |
| **P15** | **证据分级与诚实边界** | 实测 / 推断 / 未验证要分开写；未验证清单是交付物的一部分 | 本文 §5；`Fidelity` §G/§I.10/§J.8/§K.8/§L.10/§M.10 |
| **P16** | **"观感"要翻译成可判定的量** | 用户说"斜了/飘了/走得慢"，都要先翻译成"哪个量、在哪个帧、什么口径" | "像平放扫到的东西被倾斜" → 刚性旋转 vs 覆盖图案（§0.1）；"慢" → 其实是"走不到"（0/4 到点，§2.4.1） |

---

# §4 总账表

## §4.1 所有 before → after 的实测数字（按层）

> 口径：除非注明，均为**无头隔离跑**、`lio:=small_point_lio`、同一世界/出生点；
> "证据"列的 `§` 指原文档章节（本文相应条目在 §2 里）。

| # | 指标 | 口径 / 条件 | 修前 | 修后 | 证据 |
|---|---|---|---|---|---|
| 1 | `base_link.STL` 碰撞的接触工况 RTF | ODE quick/50、1 ms 步长、有接触 | **0.200**（最低 0.04，窗口内只推进 0.49 s） | **1.002**（4 box） | `robot_models.md` §9.3 |
| 2 | 峰值 RSS（同一工况） | 同上 | **760 MB** | **174 MB** | 同上 |
| 3 | 视觉面数（12 个 mesh 合计） | 磁盘/清单 | **2,821,320** | **163,995（−94.2%）** | §13.3.3 |
| 4 | 视觉磁盘 | 同上 | **141.07 MB** | **8.20 MB** | 同上 |
| 5 | 抽稀误差（最差） | 包围盒 / `out→src` p99 / 剪影 IoU | — | **0.297 mm / 0.18 mm / 0.9919** | 同上 |
| 6 | 自击 `r<0.12 m` 占比 | robot11 原始云 | **75.5%**（Phase 2 实心 4 box） | **29.0%**（A 方案挖视锥；剩下的是物理真实云台回波） | §11.2 |
| 7 | `/segmentation/ground` 点/帧 | 同上 | **529** | **5637**（默认模型 2739） | 同上 |
| 8 | `/odom` 消息数 | 同一窗口 | **0**（有发布者） | **188** | 同上 |
| 9 | `/scan` >4 m 波束/帧 | 同上 | **0** | **≈81** | 同上 |
| 10 | `/scan` 有限束数 | 同帧 A/B（改 `max_height`） | **856.8**（linefit）/ 1113.8（patchwork） | **934.7（+9.1%）** / **1163.5（+4.5%）** | `ground_segmentation_slots.md` §11 |
| 11 | 0.40–1.00 m 高度带覆盖率 | 同一朵 obstacle 云 | **0.0%** | **100%** | 同上 |
| 12 | `/scan` 波束在 odom 里超出 `[0,2] m` 的比例 | `sensor` 档 | 0.404 | **0.3960**（❌ 未达成：2D 链路结构上定不了基） | `Fidelity` §K.1.3 |
| 13 | 车那格代价 | robot11 静止、近地剔除 | **42（非 free）** | **0（FREE）** | §K.2.3 |
| 14 | 车半径圆内 `≥99` / free | 同上（半径 0.3565） | **156 / 239** | **0 / 999** | 同上 |
| 15 | 到最近 lethal（车心） | 同上 | **0.531 m** | **1.534 m** | 同上 |
| 16 | **全局** 车那格 | 静止、三档几何 A/B | **99**（0.3565 外接） | **0**（0.300 内切） | §K.3.1 |
| 17 | 到最近 lethal（全局） | 同上 | **0.135 m** | **0.804 m** | 同上 |
| 18 | 原始云"地面峰" | robot11、插件 A/B | **−0.280 m**（随距离变，像斜面） | **−0.155 m**（尖峰） | §K.4.2 |
| 19 | 车半径圆内 `≥99`（插件修复后） | 同上 | **13** | **0** | 同上 |
| 20 | `gravity_aligned_frame: base_link` 的 ground 点/帧 | 同一帧、活的节点 | **0** | **3969**（默认路径 3969 → 3969 不变） | `robot_models.md` §11.4 |
| 21 | `/scan` 丢帧 | 同一条路线、`scan_queue_size` 保持 1 | **116（4.3%）**，2.5 s 节律 | **10（0.35%）**，节律消失 | `slam_toolbox_scan_drops.md` §9.4 |
| 22 | 等待 TF 的 p90 | 同上 | **136 ms** | **13 ms** | 同上 |
| 23 | `/cloud_registered` 地面倾角 | `urdf` 档 | **30.970°** | **1.054°** | `Fidelity` §J.1.1 |
| 24 | `odom→base_link` 假俯仰 | `urdf` 档（真值 0.003°） | **4.890°** | **0.317°** | §J.1.2 |
| 25 | `map→odom` 逐次更新步长 p95 | GICP 静止 A/B（同一二进制） | **0.0087 m** | **0.0005 m（17×）** | `gicp_divergence_and_jitter.md` §4.2 |
| 26 | `map→odom` 累计修正路程（150 s） | 同上 | **4.085 m** | **0.247 m（16.5×）** | 同上 |
| 27 | 融合位姿逐样本跳变 p95 | 同上 | **0.0048 m** | **0.0020 m（2.4×）** | 同上 |
| 28 | 发目标前"等定位稳定"判据 | P0 回归 | 旧文档 **3.4 cm / 30 s** | **0.0020 m / 5 s** | 同上 §4.4 |
| 29 | `/initialpose` 生效延迟 | 饱和（PCL 400~490 ms/帧 @10 Hz） | **22.7 s**（且只在停流后被处理） | **0.24 s**（= 一个 align 周期内） | `gicp_initialpose_latency.md` §4.2 |
| 30 | 位姿图节点间距 | 同路线/速度/3 分钟 | **0.797 m** | **0.313 m** | `slam_toolbox_tuning.md` §4.1 |
| 31 | 环秩 / 回环修正事件 | 同上 | **1 / 0~1** | **8 / 6** | 同上 |
| 32 | 受控实验：TF 晚发对丢帧 | 完美里程计（ATE 0.000 m） | 按时发 ⇒ **0** | 晚 130 ms ⇒ **62（4.26%）** | `slam_drops_loopclosure_odom.md` §0 |
| 33 | IMU 全程峰值 | 整栈 A/B（同目标） | **110.8 m/s²** | **50.8（−54%）** | `slope_speed_limiting.md` §3.1 |
| 34 | 结局（到点） | 同一张图 / 同一目标 | **0/4 到点**（1 ABORTED + 3 超时） | **2/2 到点**（~27 s / 22.6 m） | `lio_divergence_no_impact.md` §3.3 |
| 35 | `|v|` p50（只比没发散的那一对） | 同上 | **0.05 m/s** | **1.16/1.19 m/s** | 同上 |
| 36 | 平地回归用时 | 2.9 m 路线 | **3.2 s**（限速关） | **3.6 s**（限速开，+0.4 s，单次差值） | `slope_speed_limiting.md` §3.3 |
| 37 | 自击掩膜剔出的点/帧 | robot11 静止 | **0** | **3374（≈29.4%）** | `robot_models.md` §12.3.3 |
| 38 | 走廊最近格 `data_min_d` | 同上 | **0.00 m**（37/37 帧） | **0.20 m**（46/46 帧） | 同上 |
| 39 | 近场 max 台阶残差 | 同上 | **0.023~0.032 m**（日志里出现过 0.128~0.203） | **0.002~0.005 m** | 同上 |
| 40 | "参考坡度" | 同上 | **0.0°**（被自击格钉住） | **2.9~3.1°** | 同上 |
| 41 | 静止时限速落在地板的帧数 | 同上 | **9/129**（日志行） | **0/46** | 同上 |
| 42 | 计划路径到内切带距离 p50 / 执行余量 p50 | smac2d 平滑器 | **0.25 m / 0.57 m** | **0.335 m / 0.85 m**（健康段 0.75 → **1.03**） | `path_clearance_and_contact.md` §2.1/§3.1 |
| 43 | 角速度执行率（台架，`wz=1.0`×12 仿真秒） | 同一世界/出生点/命令 | **1.03%（+7.09°）** | **16.2~16.5%（+111.7…113.3°）** | `Fidelity` §M.3 |
| 44 | 角速度执行率（整栈） | 盲发 `wz=1.0`×12 s（请求 687.5°） | **0.55%（真值 +3.79° / LIO +3.86°）** | **5.15%（+35.42° / +33.75°）**（优于默认模型 3.98%） | §M.7 |
| 45 | 60 s 无指令自由漂移 yaw | 整栈 | **+8.20°** | **+0.006°**（LIO −0.091°；默认模型 −0.154°） | 同上 |
| 46 | 反向 **1.2 m** 目标 | 在可达区内、闸门放行 | 未测 | 真值位移 **1.2102 m**、残余 **0.0794 m**、**`Goal succeeded`** | 同上 |
| 47 | 纯物理盲推极限（正前 / 正后） | 绕过 nav2，25 s 腿 | **0.4199 / 1.5753 m** | **0.3992 / 1.4370 m**（可达性判据取值） | §M.7/§M.8 |
| 48 | `--goal-forward 0.5` 真值位移 / 残余 | `slam_nav` | 0.313 / 0.170 m（§K 口径） | **0.2977 / 0.1902 m**（同协议复跑） | §L.1.3 |
| 49 | 缓坡被判障碍的比例 | STL 真值、22~35° | linefit **35.7~44.3%** | patchwork **2.5~3.0%** | `ground_segmentation_slots.md` §5.3/§6 |
| 50 | `/scan` <1.5 m 近距假回波 | 整栈 smoke | **91.32**（linefit） | **34.43**（patchwork，−2.65×） | 同上 §6.1 |
| 51 | 分割器 CPU | 同上 | **0.0836 核** | **0.0562（−33%）** | 同上 |
| 52 | 0.15 m 薄墙覆盖（`th_dist`） | 合成台架 | **11.8%**（0.125） | **100%**（0.08） | 同上 §3.1/§5.3 |
| 53 | 2D 先验：全场台阶边沿被表示 | `verify_low_terrain.py` | **49.44%** | **52.71%**（同窗口对照 54.42%） | `map_assets.md` §3.1/§3.2 |
| 54 | 2D 先验：撞击窗 `map(−3.6,3.1)±0.6` | 同上 | **28/33 = 85%** | **31/31 = 100%** | 同上 |
| 55 | 2D 先验：可行驶斜面被标占用 | 同上 | **3.29%** | **5.21%**（其中"离真值边沿 >0.20 m 的真障碍" 0.57% → 0.90% = **+0.05 m²**） | 同上 |
| 56 | 3D 先验点数 / 足迹 | GICP 自相似探针 | **178,543 / 415.6 m²** | **203,352 / 449.3 m²** | 同上 §3.3 |
| 57 | 累积点云垂向拖影 → 重投影误占 | 真值平坦处 `map(−5,2)` | `/scan` 图 22.7 m² | 累积点云图 **122.6 m²**（94.7% 的占用格真值局部高差 ≤0.10 m） | `traversability_plan.md` §9.1 |
| 58 | 建图 `/map` 占用格 / 已知格 | 整栈 smoke | linefit 6544 / 184 537 | patchwork **7756（+18.5%）/ 216 322（+17.2%）** | `ground_segmentation_slots.md` §6.1 |
| 59 | LIO 正常工况漂移 | 分段路线（51.4 m / 408 仿真秒） | — | 直线 **0.0105 m/m**、窄沟 **0.0031 m/m**、整程 ATE RMSE **0.036 m** | `lio_drift_diagnosis.md` §0 |
| 60 | 高角速度甩转 | 6 次同档 | — | **6 次坏 3 次**（最坏位置跑飞 10.68 m、`map→odom` 13.34 m） | 同上 |
| 61 | PCL GICP 单帧 align | 构建口径修复（`colcon_defaults.yaml`） | **369.9 ms** | **11.3 ms（32.7×）** | `build_optimization.md`；提交 `6ae2ada` |
| 62 | 可通行面积（RMUC2026） | clearance ≥ 0.22 m | free 237.8 m² | **≈176 m²（74%）**（最窄通道 0.45 m，车直径 0.44 m） | `worlds.md` §6 |

## §4.2 ★ 被否掉的候选修法（连同否掉它的数字）

| # | 候选修法 | 为什么看起来合理 | **否掉它的数字** | 证据 |
|---|---|---|---|---|
| 1 | 把插件 `update_rate` 提到 **1000 Hz** | 每 10 ms 才重设速度，接触/约束有 10 个物理步去掰回来 | 角速度确实 → **7.8%**，**但**纯物理正前方盲推 **0.4077 → 2.5652 m**、底盘 z **0.1522 → 0.4005 m（爬上障碍）**、60 s 漂移 0.308 → 0.297°/s（**没解决**） | `Fidelity` §M.5 |
| 2 | GICP **紧门限**（0.8 m/s、cap 0.12 m、20°/s） | "按健康 p99 × 2 定阈值"是常见做法 | 拒绝率 **18%**，拒的是**合法修正**（创新 0.1226 m / 0.538°、fitness 0.0024）；发布值被冻住 ⇒ 创新 0.31→0.50→**0.95→2.24 m** ⇒ 连续 25 帧判失效（**死锁**） | `gicp_divergence_and_jitter.md` §5.1 |
| 3 | **只平滑输出**、初值仍取上一帧测量 | "输出平滑了就不抖了" | 静止 120 s 漂 40 cm、逐样本跳 16 cm **照旧**（测量链本身仍是"增益 1 的积分器"） | 同上 §5.2 |
| 4 | 用**真足印多边形**替代 `robot_radius` | 几何上最正确 | `rcl` 把嵌套序列当 key ⇒ `Couldn't parse params file … Sequences cannot be key at line 127` ⇒ **所有** nav2 节点一起 exit、整条导航链起不来（正确写法必须转成**字符串**）；且量到的收益只剩"转弯角不扫障碍" | `Fidelity` §K.3.4 |
| 5 | 调大 **`scan_queue_size`** 消丢帧日志 | 日志里就是"queue is full" | 队列 2 ⇒ `queue is full` **62 → 0**，但同一跑出现 **70 条另一种丢帧**，位姿图节点 **58 → 35** ⇒ **总量没减少，只是换了名字** | `slam_drops_loopclosure_odom.md` §0 |
| 6 | 只放松回环 **`chain_size`/半径/阈值** | "阈值太严所以不回环" | 只放松 chain 10→4 ⇒ **环秩全程 0**；chain4 + 半径 4 + 阈值放松（节点间距不变）⇒ 环秩只到 **2** | `slam_toolbox_tuning.md` §0/§4.1 |
| 7 | **收紧**回环阈值治"地图跟着车转" | "门槛太松 ⇒ 误回环" | 收紧后 `Score histogram: **Count: 0**`（一条回环都没命中）⇒ 位姿图完全没有全局锚定 ⇒ **方向错**；真因是时间基混用 + 先验给错源 | `issues_and_findings.md` #20/#21 |
| 8 | **角速度限幅**（`wz_max 2.5→1.4` 等三处） | `lio_drift_diagnosis` 说高角速度甩转会飘 | 因果方向相反：3 次分歧起步时指令 `|ω| ≤ 1.15/1.29/1.40 rad/s`，而 2.8~3.0 rad/s 的 recovery spin 出现在 LIO 已经飞到 **4.9~9.2 m/s 之后**；唯一一跑 `y1` 没发散且到点，但**比新表慢 4.5 s、p50 低 0.27 m/s**，且**一次跑分不开"限幅有效"与"这次本来就没磕重"** | `lio_divergence_no_impact.md` §4.2 |
| 9 | 用**标障碍**代替限速 | "把坡面标成障碍规划器就绕开了" | 坡面**本身可行驶**（标它是错的）；实测只把第一击 **86.4 → 68.2 m/s²** | `slope_speed_limiting.md` §5.1 |
| 10 | **只朝前**的走廊 | 车正常是往前开的 | 两次整栈跑里**最重的一击都发生在倒车时**（`v = −1.04/−1.00 m/s`）；第一版峰值 66.5/67.7/86.5 ⇒ 改成**双向窗口**才降到 50.8 | 同上 §5.2 |
| 11 | 用 `/odom` 的 `twist.linear.x` **符号判方向** | 物理上最直接的信号 | 机器人确实在倒车（`/cmd_vel` 与真值都是负的），但 `[slope_speed]` **一直打"前进"** ⇒ 判据没生效、**根因未定位** ⇒ 默认关闭、改用双向窗口 | 同上 §5.2/§7 第 1b 条 |
| 12 | 把 **`velocity_smoother.max_velocity 2.0→1.0`** 当限速替代 | 一行参数就能"开慢点" | 它**不看地形**，平地也慢 —— 而本机制的意义正是"只在低地形前慢" | 同上 §5.6 |
| 13 | bug ③ 的**平移也改成物理真值** | "修就修全" | `/scan` 盘面 z p50 **+0.063 → −0.091 m**；落在高度带之外的波束 **0/948 → 950/950**；局部代价图 lethal/inscribed **461/16937 → 0/0**（robot11 与**默认模型**都整张变空） | `Fidelity` §J.5 |
| 14 | C 方案：`body` 用**抽稀网格**当碰撞 | 比 4 个 box 更贴形 | **ODE 的 ray-vs-trimesh 求交代价是 O(三角面数)**：30000 条射线 × 3000 面 ≈ **9×10⁷ 次/帧 @10 Hz** ⇒ `spawn_entity` 服务 **60 s 超时**、90 s 内 cloud/imu/scan **全 0**。**不是参数问题** | `robot_models.md` §11.2.2 |
| 15 | MeshLab `meshlabserver` 的 QECD | 最"标准"的抽稀工具 | 本机**无 GL**：`MLException: GLEW initialization failed: Missing GL version`（连列过滤器都走不到） | `robot_models.md` §13.3.1 |
| 16 | 用**精确**误差口径（`vtkImplicitPolyDataDistance`） | 精度更高 | 在 dst 建 cell locator，`base_link` 那种 **208 万面**的 dst **10 分钟都出不来**；只在 `l12` 上校准过（精确 p99 0.204 mm vs 采样 0.000 mm ⇒ 采样**偏低 ≤0.22 mm**） | 同上 §13.3.3 注 1/§13.10 第 6 条 |
| 17 | `beluga` 槽位沿用 `laser_min_range: −1.0` | 别的槽位就这么写的 | beluga 对该键声明的是 `[0, DBL_MAX]` ⇒ **启动即崩** | `localization_slots.md`；提交 `c655d77` |
| 18 | cartographer：`use_odometry=false`（"干脆关掉先验"） | 先验同源冗余、晚到会撞 CHECK | 关掉后**近处地面/内部小墙留存曲线**坏、自由格=0（"栅格一点点出来很艰难/全灰"）；且**回环 Count: 0** 说明被调的门槛属于"已经不工作的东西" | `issues_and_findings.md` #20/#26 五次结论 |
| 19 | slam_toolbox `mode: lifelong`（在旧图里定位 + 顺便精修） | 名字就是"终身建图" | ① 没有任何代码读它；② 本机**没有 `lifelong_slam_toolbox_node` 可执行文件**（上游 `CMakeLists.txt` 故意没装）；③ 上游自己标 *highly experimental* ⇒ COD 队的 `mode: lifelong` **实际跑的就是普通 async 建图** | `slam_toolbox_tuning.md` §0 第 2 条 |
| 20 | 把 99.1 MiB 的大文件**排除**在仓库外 | GitHub 单文件硬上限 100 MiB，它只差 0.9 MiB | 实测**入库并 push 成功**（只有 >50 MiB 的 warning）；代价 = 以后每次 clone 都要拖这 104 MB（**没用 LFS**） | `robot_models.md` §9.1/§10.8 第 8 条 |

---

# §5 诚实边界（合并各文档的未验证清单，去重 + 按风险排序）

## §5.1 未验证 / 没做完（**不要当成已知**；按"影响下一步决策的程度"排序）

| # | 未验证项 | 风险 / 影响 | 出处 |
|---|---|---|---|
| 1 | **`robot:=robot11` 没有重跑"修前"的整栈基线**（§L 的 +3.79° / +8.20° 是直接引用；台架复现过、整栈"修后"有） | 严格同期对照缺失；要补：`--chassis articulated` 回退后再跑同一条命令 | `Fidelity` §M.10 1 |
| 2 | **反向 2.0 m 目标没有"修前 vs 修后"的同协议对照** | 只知道"修后"的形状（Δyaw −179.7°、0 条 plan） | §M.10 2 |
| 3 | **`wz` 剂量-响应没做完整扫描**（只有台架单点 `wz=1.0` 与整栈 `wz=1.0`；`wz=−0.75` 只由 RPP 给出） | "角速度通道好不好"缺一条曲线 | §M.10 3 |
| 4 | **单刚体丢掉了 12 个不受控自由度**（`j2…j11`）：将来若要给轮子/云台加真控制器，**必须**先回退到 `--chassis articulated` 再改造 | 影响未来扩展（生成器留了开关） | §M.10 4 |
| 5 | 塌陷后的接触变成"4 个 cylinder 刚体接触"，**平动的摩擦特性也变了**（正前方 0.4077 → 0.3992 m，−2%；反向 1.4370 与 §L 的 1.5753 同量级，差异来自 25 s 腿的时间上限 + 起始点不同） | **没有做系统标定** | §M.10 5 |
| 6 | **`robot11_mount:=sensor` 的"整条链端到端好用"没有任何意义被证明**；**代价图那一层仍然坏**（odom 斜 30° ⇒ `obstacle_layer` 实测 **41.2%** 波束被高度带丢掉） | 该档只能当"帧↔数据自洽"的示范，不能当"可用链" | §J.7 第 3 条、§J.4.2 第 (3) 条 |
| 7 | `sensor` 档**只测了 `lio:=small_point_lio`**；`fastlio`/`pointlio` 走 `lio_tf_adapter` 的杆臂只做了几何/单元级验证；`ground:=patchwork` + `sensor` 在 launch 里**直接报错**（设计选择，不是验证过的组合） | 组合覆盖不全 | §J.8 5/6 |
| 8 | **bug ③ 的"平移"与"代价图高度带重新定基"没实现**（只给了两个候选与实测代价） | 现状态是"歪打正着"（靠 0.2044 m 的巧合补回 odom z） | §J.5/§J.8 10 |
| 9 | **`sensor_height` 的 4.3 cm 疑问**：几何预测 **0.2595 m** vs 由原始云反推 **~0.302 m**（要查的是"轮 collision mesh 最低点 vs 关节 origin 的 FK"）；影响面 = `linefit.sensor_height`（别的任务的槽位文件）与 `p2l` 高度带的解释 | 若真偏 4 cm，地面线整体抬高 4 cm，与 `max_dist_to_line 0.05` 同量级 | §K.1 末尾/§K.8 6 |
| 10 | **`sensor` 档插件 A/B 没跑完**（`k2_sen_legacy` 那一跑 `probe.json` 的 `frames = {}`，而 linefit/限速都正常、TF 也 −30.000° ⇒ **取数时机/订阅侧的问题**，不是该档坏了）；要补：`bash tools/scripts/tiltmount/_batch_k2.sh`（≈15 min） | 该档的插件修复数字**不主张** | §K.8 3 |
| 11 | **全局代价图在静止时仍然偏脏**（0.300 档车那格 0、圆内 123 free，但整张图 unknown 仍占大头、`map` 还在建）；**没做**"跑几分钟之后再看"的对照 | 影响"长跑能不能干净" | §K.8 2 |
| 12 | `sensor` 档 `/scan` 出带比例仍是 **0.396**（2D 链路结构上定不了基；真修要改代价图帧或 LIO 的 odom 重力对齐） | 已知且**不打算在本期修** | §K.8 1 |
| 13 | **没测** `ground:=patchwork` + 近地剔除；**没测** `urdf` 档 + 近地剔除（那一档"半圈波束被高度带丢掉"仍在） | 组合覆盖 | §K.8 7/8 |
| 14 | **RTF / 内存没有做"同时间窗交替"的严格 A/B**（RTF 0.339~0.416 只能读作"同一批里可比"） | 凡是"哪个更慢"的结论都不成立 | §G 3、§K.8 9 |
| 15 | **真足印多边形只做到"知道怎么写"**（nav2 要**字符串**格式；没跑通那一版） | 转弯角扫障碍的折中仍在 | §K.8 4 |
| 16 | **`--goal-forward 2.0` 本身没修**：参数层面**没有可修的**（减小 `inflation_radius`、放大 `xy_goal_tolerance`、改 `regulated_linear_scaling_min_radius` 都**只会**让车更靠近障碍或更早宣布成功） | 要真做长距离验收必须换 spawn/世界，或先修角速度通道（已修） | §L.9 4、§L.0 |
| 17 | 先验图里正前方障碍位置（x≈0.5）与实测物理极限（0.776 m）**差 ~0.28 m 未归因**（候选：口径/尺度差，或两个不同物体）；反向那一侧是吻合的 ⇒ **不是整体平移** | 地图与物理的一致性有未解释残差 | §L.10 2 |
| 18 | **没做**"`slam_nav` 跑 5 分钟后再发同一个目标"这一格（本轮所有目标固定在 launch 后 60 s 发） | 建图成熟度的影响只有 `mode:=nav` 这个"图已建全"的对照 | §L.10 4 |
| 19 | **可达区形状只有两个方向的读数**（前 0.40 / 后 1.44 m），没有做"同一距离、不同方位角"的扫描 | 闸门只能用在这两个方向 | §L.10 6 |
| 20 | **"抽稀省 GUI 内存"在本机没有被证实**（`gzclient` 473 / `gzserver` 3202 / `rviz2` 314~322 MiB 两档都一样，且与 Phase 4 的 1381↔243 MiB **不一致**）；渲染端足迹（~240 B/面）是**推断**；**真 GPU 帧率未知**（只有 llvmpipe 软件 GL） | 用户机器上的实测才是裁决 | §13.4.3/§13.4.4/§13.10 2 |
| 21 | **`base_link` 的精确误差口径跑不出来**（208 万面 dst 上 10 分钟未返回）⇒ 它的 p99/max 只有采样口径的数；`--calibrate` 只在 `l12` 上对过 | 方法学缺口（已登记） | §13.10 5/6 |
| 22 | **抽稀后的视觉在真 GPU 上"看起来够不够"没看过**（没有一张真渲染截图做视觉确认） | "好不好看"未验 | §13.10 7 |
| 23 | `decimated` 与 `full` 两档**没有做端到端逐帧对照**（只在结构上排除了差异来源：A1/G5 断言） | 成本高 | §13.10 9 |
| 24 | **`/scan` 里那圈"自身障碍"没有修**，也没有验证"它是不是 nav 不走的根因" | 候选修法（在 p2l 侧加自击掩膜 / 在插件侧修内移）**未做** | §13.9/§13.10 11 |
| 25 | `model://` 解析：**真 GPU / 真显示器没跑过**（"mesh 加载"是用 `/proc/<gzclient>/io` 的 `rchar` + fd 扫描证的）；沙箱里 gzclient 对"运行期插入的模型"处理不稳定；`robot11_visual:=full` 档的解析链**没有单独复测** | "车在真机屏幕上好不好看"仍未验 | `robot_models.md` §14.6 |
| 26 | 用户机上"`gzclient` 还会不会被 `-9` 杀"**仍未复现**（本机 31.9 GB 两档都活满窗口）；两个等价解析根会让 `InsertModelWidget` 多报 3 行 `Missing model.config` 噪音（**没有去消**） | 需要在同类内存受限环境里跑 `full` 才能证实 | `gazebo_gui_troubleshooting.md` §9 |
| 27 | `hzmirm` 槽位**没有在新代码下重跑**（§11.4 的更正只是"读数据 + 复现 bug"）；`<1 m` 的近场点里"云台自击"与"真障碍"**仍未分离**（只有离线射线求交的归因）；B 方案（抬高雷达）与 `linefit.sensor_height` 的同步是**手工**的（忘记改就会出现"地面线整体偏 h"的静默错误） | 三个槽位对比的公平性 | `robot_models.md` §11.8 3/4/5 |
| 28 | 限速：**"第一击消除"没做到**（110.8 → 50.8，目标 ≲30；最可能原因 = 走廊里坡度/台阶估计仍稀（单帧 7~20 格）、4°/0.04 m 死区为压噪而设）；**承诺前推用"上限"当速度**（实车/打滑/被挡时**低估剩余距离、提前释放**）；**只验证了 `nav:=mppi`**（RPP/DWB/TEB 的 `setSpeedLimit()` 没测）；`ground:=linefit` 槽位只跑了默认槽；**只在"一张先验图 + 一个目标"上各一次跑**；`mode:=mapping` 没跑；真车未验；`/odom` 方向源在 LIO 发散时会跟着错；负障碍（坑/落空）只有部分覆盖 | 这一串直接决定"能不能上真车" | `slope_speed_limiting.md` §7 |
| 29 | 建图/资产：全场台阶边沿覆盖率（35.98%~54.42%）**够不到 80% 判据**，这是 2D 先验的**固有上限**（不是本次提升引入）；累积点云的垂向拖影会让 3D→2D 投影大面积误占；全局图"局部图 raytrace 还没清到"的残留仍在 | 换场地/换图前必读 | `map_assets.md` §3.1、`traversability_plan.md` §9.1 |
| 30 | LIO：唯一复现的失效模式（高角速度甩转）**门槛未定**、机理（"抛硬币"抛在 GICP 位姿门/内点上）只是**判据**不是**修复**；续建污染的 PCD 资产**没修**（只证明不影响实时位姿）；所有数字来自**单个世界 + 单出生点** | 不能外推 | `lio_drift_diagnosis.md` §11 |
| 31 | `tdt-nav-kit`：效率账里"必须在**我们机器上**测"的那几项（§3.6 的清单）**一项都没测**；`docs.nav2.org` 的 `/humble/` 目录 404，凡是契约都以本机 1.1.20 头文件为准 | 研究结论**未落地** | `nav2_vs_custom_planner.md` §0/§7 |
| 32 | 时间戳：`point_lio` 的同一 bug **判定但没能实测**（该槽位在仿真里第 100 帧就被自己的 `pcd_save` 打死）；两个 LIO **子模块的 bug 一行未动**（只报告 + 给补丁） | 上游 fork 里仍存在 | `timestamp_construction_audit.md` 结论先行/§4.4 |
| 33 | GICP：`localization:=small_gicp` 槽**没做行驶 A/B**（两条判据同样生效是结构性结论）；更紧的 `max_correspondence_distance`（1.5→0.8~1.0 m）**未测**；"沿墙方向不可观测"是**地图资产 + 场地几何**的性质，调参救不了 | 不要指望调参 | `gicp_divergence_and_jitter.md` §5.3/§7 |
| 34 | 3D 先验 + GICP 的"定位仍在缓慢收敛"是一条**打折项**（`worlds.md` §5）；`stvl_local_costmap.md` 的四次跑数字本文**未复算**（见 §5.3） | 读表要打折 | `worlds.md` §5、`stvl_local_costmap.md` §3 |

## §5.2 "这些结论是靠什么支持的"（单跑 / 单出生点 / 无头）

| 支持力度 | 结论 |
|---|---|
| **可复现三次以上 / 结构性论证 + 实测** | 车那格 inscribed 团（**513/997 → 435/1001 → 453/1000** 三次独立跑）；`plugin` 档点云/平面拟合**逐位复现**（`(−0.01376, 0.0122, 0.99983)`、`d=0.26038`、1.054°、自击 3294）；`a281636` 的截断修复（丢帧 116→10 单跑，但机理链四步都可单独验证） |
| **单变量 A/B（较强）** | GICP 静止 A/B（**同一二进制**、只切两个开关）；近地剔除同帧回放 A/B（同一帧点云）；插件 `range_from_origin` A/B（同一次编译只差那一行）；限速开/关（同一真源临时改、跑完还原 + diff 核对） |
| **单跑 / 单出生点 / 单目标** | 限速整栈 A/B（各一次跑，撞击量级本身是**概率性**的）；`robot11` 的绝大多数数字（**一个出生点**：RMUL2026 出生点）；`--goal-forward` 系列（各一次）；`sensor` 档三方 A/B（各 1 次静止 + 1 次目标 + 1 次固定动作） |
| **无头 / 软件 GL（不代表真机观感）** | 所有 GUI 数字（Xvfb 3200×1200×24 + llvmpipe/Mesa 23.2.1）；所有"mesh 加载"证据（文件级 `rchar` + fd 扫描）；**没有一次真 RViz 观察** |
| **纯推断（明确登记）** | 渲染端网格常驻足迹 ~240 B/面；用户机 `gzclient` SIGKILL 的内存压力解释；"包侧 export 会让默认模型的 env 多一项"的影响（无头量不出来，只有 GUI 噪音 3→6 行） |
| **概率性结论（不要当确定量）** | "撞击 ⇒ LIO 发散"（11 跑 / 9 配置里 9 次硬撞、3 次仍成功）；"高角速度甩转 ⇒ 飘"（6 次坏 3 次） |

## §5.3 源可核性：这份汇总"读了什么、没读什么"

**逐字/整节读完的源**（本文引用最密）：`tf_interface_contract.md`（全文）、
`slope_speed_limiting.md`（§0–§7）、`tilted_lidar_fidelity.md`（§A/§B/§C/§D.3–§D.4/§G/§H/§I/§J/§K/§L/§M 的关键节与全部表格）、
`robot_models.md`（§1.3/§3.2、§9–§15）、`gazebo_gui_troubleshooting.md`（§0/§1 + `robot_models.md` §14 的转述）、
`slam_toolbox_scan_drops.md`、`mapping_2d_from_cloud.md`、`slam_toolbox_tuning.md`、`slam_drops_loopclosure_odom.md`、
`map_assets.md`、`continue_mapping.md`、`lio_divergence_no_impact.md`、`gicp_divergence_and_jitter.md`、
`gicp_initialpose_latency.md`、`lio_drift_diagnosis.md`、`timestamp_construction_audit.md`、`path_clearance_and_contact.md`、
`ground_segmentation_slots.md`、`traversability_plan.md`、`nav2_vs_custom_planner.md`、`params_ownership_checklist.md`、
`sim_real_contract.md`、`worlds.md`、`README.md` 的**结论节 + 关键表**（各文档的 §0/速查 + 上表列到的具体表）。

**只读到"结论/摘要"层面、本文据此转述的源**（要逐行细节请回原文档，本文**没有**独立复算它们的数字）：
`stvl_local_costmap.md`（只读 §0 摘要 + 标题）、`mapping_small_point_lio.md`（只读 README 摘要 + 提交信息）、
`lio_slots.md` / `localization_slots.md`（只读 README 摘要 + 提交信息）、`algorithm_matrix.md`（只读 README 摘要）、
`smoke_test_runbook.md`（只读 README 摘要）、`issues_and_findings.md`（只读前 40 行 + #15/#18–#26 的摘录）、
`architecture.md`（只读标题结构与 §3.2.x 目录）、`glossary.md`（未读，术语表见本文 §8）、
`debug_fastlio_cartographer.md` / `cartographer_2d_occupancy_semantics.md`（只通过 `issues_and_findings.md` 与 README 间接引用）。

**明确不在本工作期范围、本文未读**：`cod_nav_*.md`（8 份，2026-10-04 以前的移植规划）、
`rm_algolab_plan.md` / `rm_algorithm_overview.md` / `rm_bench_refactor_plan.md` / `rm_algorithm_catalog.md`、
`scan_context_*.md`（3 份）、`mapping_relocalization_survey.md`、`nav_strategy_taxonomy.md`、
`slam_nav_goal_strategies.md`、`research_p2l_scan_stall.md`、`kinodynamic_esdf_mpc_notes.md`、
`build_optimization.md`、`3d_to_2d_survey.md`、`mapping/` 与 `package/` 两个子目录。
（它们的存在与定位见 `docs/README.md`；本文只在"P0 阶段"里提了它们对应的提交。）

## §5.4 文档之间的矛盾，以及本文采信了哪一版（**逐条给理由**）

| # | 矛盾 | 采信 | 理由 |
|---|---|---|---|
| 1 | `tilted_lidar_fidelity.md` 把 §I/§J 标为 2026-10-09、§K/§L/§M 标为 2026-10-10；但对应提交在 `git log` 里是 **2026-10-07 / 10-08** | **git 日期**（本文时间线），引用时保留原 `§` 编号 | 提交是"改了什么/什么时候"的唯一真值；文档内的"会话日"标签不稳定 |
| 2 | §K.5 表记"`detected collision ahead` = **0 条**"；§L.1.3 记长目标是 **135 条** | **§L**（0 条只对**短目标**成立） | §L 是同协议复跑 + 明确写了"口径更正" |
| 3 | §J.5 记 `sensor` 档局部代价图 **0 / 0（整张全 free）**；§K.1.3 复跑得 **495 / 16511** | **§K.1.3**（`0/0` 是"**平移也改成物理真值**"那一版的后果） | §K 明确把口径钉死，并给了独立复跑的数字 |
| 4 | §I.9 的单测把"正确合成"写成 `T_ol·T_bl⁻¹`；§J.1.2 用链式法则推出 `T_ol·T_bl` | **§J**（并保留 §I.8(f) 那句"会给出 rpy (30°,0,yaw)"是对的） | §J 是后一轮的**记法更正**，且给了单测断言 |
| 5 | `robot_models.md` §9.2 生成物文件头写整车面数 2,870,498；逐 mesh 求和是 **2,821,320** | **2,821,320** | 12 个 STL 逐项求和（提交 `15175ea` 就是修这个笔误） |
| 6 | §13.9 的 `/scan` 分带百分比分母用 693；实际有限波束 **833 条** | **833** | 提交 `e060a98` 的更正 |
| 7 | Phase 4 §12.2 报 rviz2 **1381 ↔ 243 MiB**；Phase 5 §13.4.3 同脚本重跑是 **314 / 322 MiB** | **两版都保留**，以 Phase 5 为"测量记录"，并把不一致登记为**未定论** | 两者是同脚本同环境的两次测量 ⇒ 属"测量冲突"，不能单方面宣布哪一个对（三条可能原因已列） |
| 8 | Phase 3 §11.2 说近场点"100% 落在云台 `l10/l11`，中位 0.14 m"；Phase 4 §12.3.1 复核为中位 **0.021 m**、AABB 只命中 3 点 | **Phase 4** | 更晚、更细的逐点复核，并给出了独立证据（地面 z 随距离单调变化） |
| 9 | `lio_drift_diagnosis.md` §2.3 的"巨大死区"表（要 1.9 rad/s 得给 6）；`lio_divergence_no_impact.md` §2 同批配对重测 **≈1:1** | **≈1:1**（旧表作废） | 旧表自认"大概率混进了闭环里 cmd 与被控量不同口径的问题"，且新测量给了 n 与符号一致率 |
| 10 | `hzmirm` 的 `/segmentation/ground = 0` 归因：§3.3 只讲几何；§11.4 追加"C++ bug" | **几何 + bug 并列** | 两条证据都成立（几何 6.32 m 盲环 + 该键一开 = 0 点） |
| 11 | `robot11_mount` 的取值：§15 说"只能是 `plugin\|urdf`"；§J 新增 `sensor` | **三档** | §J 是后一轮施工，且给了"选错直接报错"的语义 |
| 12 | §11 说"视觉 = 原始 mesh、视觉零损失"；Phase 5 把默认 `decimated` 指向"专用视觉 LOD" | **Phase 5**（"零损失"只在 `robot11_visual:=full` 时成立） | Phase 4 引入档位、Phase 5 改变了 `decimated` 的指向，文档自己也标了"这句话现在只在 full 档成立" |
| 13 | §C.4/§F 与 §K.5 对 `--goal-forward 0.5` 的读数（0.313/0.170 m）与 §L 的 0.2977/0.1902 m | **§L**（同协议复跑） | §L.1.3 明确写"复现，差异来自版本/时间" |
| 14 | "回退档逐字节相同"（多处）vs `d3034aa` 的精确表述 | **"模型体（去注释）逐字节相同"** | 后者是逐字节比对后的精确说法 |
| 15 | `path_clearance_and_contact.md` §3.2 说"没有任何单变量能可靠救这条链"，而 §4.1 仍然应用了 `w_smooth 0.4→0.0` | **两条都保留**：`w_smooth` 只治"贴边"（有计划/执行余量的量化收益），**不治链**（`v_mul12` 是反例） | 文档自己写明了"只治贴边不治链" |
| 16 | README 索引把 `robot_models.md` 的状态写成"（2026-10-07 新增）"，与其 §14/§15 的 2026-10-08 内容 | 以**文档正文**为准（README 只是索引） | 索引是"定位"不是"事实源" |

---

# §6 命令与回退速查

## §6.1 现在能用的启动命令（复制即可）

```bash
# ① 默认模型（本期没有改它的任何默认路径）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=nav \
    lio:=small_point_lio localization:=gicp nav:=mppi planner:=smac2d spin_speed:=0.0

# ② robot11 的"今天的行为"（帧正、点云正、射线真的斜）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav \
    lio:=small_point_lio robot:=robot11 spin_speed:=0.0

# ③ robot11 + 物理斜装 + 账也对（点云表达在真·传感器系；下游做重力对齐）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav \
    lio:=small_point_lio robot:=robot11 robot11_mount:=sensor spin_speed:=0.0

# ④ robot11 + 看"倾角记在关节上"的诊断档（RViz 里会看到 30° 斜坡 —— 那就是"帧与数据不一致"）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav \
    lio:=small_point_lio robot:=robot11 robot11_mount:=urdf spin_speed:=0.0

# ⑤ 带 GUI（不需要任何环境变量前缀；model:// 解析已自动化）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=nav \
    lio:=small_point_lio localization:=gicp nav:=mppi planner:=smac2d \
    spin_speed:=0.0 robot:=robot11 nav_rviz:=True

# ⑥ 无头 / 不再等在线模型库（两个都是 opt-in，默认 = 改造前行为）
… gui:=False              # 只起 gzserver，RViz 照旧
… gui:=False gazebo_offline:=True

# ⑦ 建图 / 续建（同名存档 = 续建；换场地续建默认拒绝）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping \
    lio:=small_point_lio mapper:=slam_toolbox map_name:=RMUC2026_home nav_rviz:=True spin_speed:=0.0

# ⑧ 只起 Gazebo + 模型（不含感知/LIO；注意：这条入口**不传** robot11_mount ⇒ 只能跑 plugin 档）
ros2 launch hzmi_rm_simulation rm_simulation.launch.py robot:=robot11 world:=RMUC2026

# ⑨ 换地面分割器 / 换定位槽 / 换局部障碍层（槽位，默认不变）
… ground:=patchwork      … localization:=small_gicp      … local_obstacle:=stvl
```

**构建（install/ 是逐文件符号链接 ⇒ 新文件必须 build 才可见）**：

```bash
# 改过插件/判据层/两个分割节点/launch/参数（最全的一条）
colcon build --symlink-install --packages-select ros2_livox_simulation rm_ground_traversability \
    linefit_ground_segmentation_ros patchwork_ground_segmentation rm_nav_bringup small_point_lio \
    pointcloud_to_laserscan robot11
```

## §6.2 验收命令（改完必须跑的那几条）

```bash
# ① robot11 槽位的 30 秒自检（跑起来之后，另开一个终端）
ros2 param get /ground_segmentation sensor_height              # 期望 0.2595（不是 0.226）
ros2 param get /ground_segmentation gravity_aligned_frame      # 期望 空串（sensor 档期望 base_link）
ros2 param get /ground_segmentation obstacle_near_ground_m     # 期望 0.05（robot11）/ 0.0（默认）
ros2 param get /ground_segmentation self_mask_enable           # 期望 True（robot11）/ False（默认）
ros2 param get /local_costmap/local_costmap robot_radius       # 期望 0.3（robot11）/ 0.22（默认）
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.min_obstacle_height   # 期望 0.0
ros2 param get /pointcloud_to_laserscan target_frame           # plugin 档期望 空串；sensor 档期望 base_link
ros2 run tf2_ros tf2_echo base_link livox_frame                # plugin: rpy 0；urdf/sensor: −30 0 0
ros2 topic echo /segmentation/obstacle --field width -n 1      # 期望 ~5500-5800（不是 ~6100-6900）
ros2 topic echo /ground_segmentation/traversability_stats -n 1 | grep -o '"near_ground":[0-9]*'

# ② P0 平地冒烟回归（改任何东西之后都要跑）
bash tools/scripts/localization/run_nav_smoke_regress.sh --tag regress_x --domain 151 --port 11821 \
     --localization gicp --nav mppi --planner smac2d --goal -1.0 2.0

# ③ 一条命令量齐 robot11（隔离无头 + 参数/TF 回读 + 契约 + 代价图圆 + 短目标）
tools/scripts/tiltmount/run_tilt_mount_probe.sh my_goal --variant plugin --settle 30 \
    --duration 30 --frames 3 --goal-forward 0.5 --goal-wait 45 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False

# ④ 三层坐标系自检（离线，秒级）
python3 tools/scripts/tiltmount/tilt_chain_check.py --dir sensor:.tmp_tiltmount/j2_sensor
#   期望："地面法向 vs 自己 frame_id z" ≈ 30°；"经 TF 转到 base_link 后" ≈ 1°；自击掩膜命中 ≈ 29.2%

# ⑤ 只量近地剔除（同一帧回放，秒级，不用 Gazebo）
bash tools/scripts/tiltmount/_replay_ng.sh <一帧 raw_*.csv> 0.2595

# ⑥ 视觉档位 / 生成物 / 限速真源 的断言（都不用 Gazebo）
python3 tools/scripts/regress/check_robot11_visual_slot.py          # 21 项断言，期望全绿
python3 tools/scripts/regress/check_slope_speed_table.py            # 真源 + 表↔模型 + 与 nav2 对账
python3 tools/scripts/regress/check_traversability_criteria.py
python3 tools/scripts/regress/robot11_make_sim_xacro.py --out /tmp/x.xacro && diff /tmp/x.xacro \
    src/bringup/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro          # 期望 diff 为空（生成物 = 生成器输出）

# ⑦ 可达性前置判据 + 目标闸门（新验收口径）
tools/scripts/tiltmount/run_nav_goal_forensics.sh m10_r11_reach25 --settle 25 --duration 6 \
    --goal-wait 0 --preflight-reach 0.25 --preflight-time 25 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
tools/scripts/tiltmount/run_nav_goal_forensics.sh m8_r11_rev12 --settle 30 --duration 30 \
    --goal-forward -1.2 --goal-wait 60 --reach-gate .tmp_tiltmount/m10_r11_reach25/forensics.json -- \
    world:=RMUL2026 mode:=nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False

# ⑧ 底盘角速度台架（变体矩阵，一次 <40 s）
#    先备好模型 SDF：xacro <模型>.xacro <args> | gz sdf -p /dev/stdin > /tmp/<模型>.sdf
tools/scripts/tiltmount/run_chassis_yaw_bench.sh t_r11_weld --model /tmp/robot11_welded.sdf \
    --variant baseline --wz 1.0 --push-wall 12 --settle 8 --add-jsp
```

## §6.3 回退速查表（每个改动**一键怎么退**）

| 想退掉什么 | 怎么做（一键） | 影响面 |
|---|---|---|
| 整套 `robot:=robot11` | 去掉 `robot:=robot11`（**默认路径从未改动**） | 零风险 |
| `robot11_mount` 任意档 | 去掉该参数（默认 `plugin`）；`sensor` 档还可只删 xacro 里的 `<cloud_frame>` | 只影响 robot11 |
| 单刚体底盘（模型行为） | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --chassis articulated` | 生成物**模型体去注释后逐字节等于** 2026-10-09 的 HEAD（两份 35025 B） |
| 视觉抽稀（回上游原始 STL） | `robot:=robot11 robot11_visual:=full` | 回到 Phase 1~3 的逐字节行为 |
| 视觉档位的默认值 | 生成器 `--visual-inventory` 指向旧清单重跑 | 默认 `decimated` 依赖 `meshes/decimated/` |
| 自击掩膜 | `config/traversability_self_mask_robot11.yaml` 的 `self_mask_enable: false`（或撤 launch 里 `self_mask_params`） | 回到"静止被压到地板 0.60" |
| 近地剔除 | `config/traversability_near_ground_robot11.yaml` 的 `obstacle_near_ground_m: 0.0`（或撤 `near_ground_params`） | 回到"车那格 42、圆内 156 个 ≥99" |
| 半径/膨胀（0.300/0.60/0.65 → 0.3565/0.70/0.75） | 改 `nav2_params_sim_robot11_costmap.yaml` 的两个键（或 `git revert 1c89473` 的该文件） | 只影响 robot11 槽位 |
| 插件 0.1 m 起点修复 | 在对应模型 xacro 的 `<plugin>` 里加 `<range_from_origin>false</range_from_origin>` | **逐字节**回旧行为（所有模型） |
| 两个 LIO bug 修复（②③） | `git revert 2a5a8b6`（默认档**逐位不变**；revert 后与 §I 那一轮等价） | 影响 `urdf`/`sensor` 档的账 |
| 时间戳四舍五入 | `git revert a281636` | 丢帧日志（116 条 / 4.3%）会回来 |
| linefit 的 `Identity()` 修复 | 改回 `Eigen::Affine3d tf;`（或 `git revert 0142bd9`） | `gravity_aligned_frame` 一开又变 0 点 |
| p2l 高度带 1.0 | 改回 `max_height: 0.1` | >0.326 m 的几何又从 `/scan` 消失 |
| 前瞻限速（只关限速） | `speed_limit_enable: false`（**构造期读参数**：`ros2 param set` 运行时不生效，要重启节点或改文件） | 速度回到 nav2 自己的 `vx_max`/smoother |
| 前瞻限速（连判据一起关） | `traversability_enable: false` | 回到"没有坡度/台阶判据"的旧链路 |
| 限速提速档（peak 45 / floor 0.60 / 死区 5.5°+0.06 m） | 六个键改回 `30 / 0.45 / 4.0 / 0.040 / 旧两张表`（或 `git revert 4adf0d8`） | 回到"0/4 到点" |
| MPPI 里残留的速度上限 | `ros2 topic pub --once /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.0}"` | 节点被 SIGKILL 时的兜底 |
| smac2d 的 `w_smooth 0.4→0.0` | 改回 `0.4`（或 `git revert 4a7996c`） | 计划路径又会贴内切带 |
| GICP 三道判据 + 低增益滤波 | `gate_enable:=false` + `smoothing_enable:=false`（= 改动前行为） | 回到"每帧采纳 + 用测量当初值" |
| slam_toolbox 节点加密 | 三个键改回 `0.5/0.5/0.5`（或 `git revert 5cb93cb`） | 回环又基本不触发（环秩 1） |
| 地图默认对（2D/3D 先验） | 用 `*.bak-20261007` 就地覆盖回 `map/RMUC2026.pgm\|yaml` 与 `PCD/RMUC2026.pcd` | 见 `map_assets.md` §4.4 的逐条命令 |
| 会话存档的某一代 | `map_archive.sh restore`（走 `.prev-<时间戳>` 那条链） | ⚠️ 不要手工搬 `.prev-*` |
| `save` 的严格会话名解析 | `git revert 02306fb`（**不推荐**：会恢复"信任 `.session.yaml`"的事故面） | 存档安全 |
| `model://` 解析自动化（启动侧） | 删 launch 里 `ld.add_action(OpaqueFunction(function=_gazebo_model_path_setup))` | 回"必须手工前缀环境变量" |
| `model://` 解析自动化（包侧） | 删 `package.xml` 里 `<gazebo_ros gazebo_model_path="${prefix}/.."/>` 并重建 robot11 | ⚠️ 裸 `gzserver`/`gz sim` 入口又会回落在线模型库 |
| `gui` / `gazebo_offline` 开关 | 不传即可（默认 = 改造前行为） | 零风险 |
| 地面分割槽位 `ground` | `ground:=linefit`（默认） | 等于没改 |
| 局部 STVL | `local_obstacle` 槽位改回默认 | 零风险 |
| 本期的**工具**（`tools/scripts/tiltmount/` 等） | `rm -rf tools/scripts/tiltmount/`（**不进任何 launch/节点**） | 只会让 §6.2 的命令失效 |

---

# §7 提交索引（本工作期全部相关提交）

> 口径：`git log` 按**作者日期**取 `2026-10-05` 及以后（含 10-08），共 **161** 个提交，按时间倒序。
> "层"一列是由提交标题的关键字机械映射出来的**粗分类**（便于检索，不是严格归属）。
> ⚠️ 若用 `git log --since=2026-10-05 --until=2026-10-09`，你只会看到 **128** 个 ——
> `--since/--until` 按提交日期过滤，而本仓这一批提交的作者/提交日期虽然相同（核过 0 处不一致），
> 边界时刻的时区解释会让窗口比"作者日期 ≥ 2026-10-05"少一批。**以本文这张表为准。**

| hash | 日期 | 一句话（提交原文） | 层 |
|---|---|---|---|
| `d3034aa` | 2026-10-08 | docs(robot11): 把"回退档逐字节相同"说准 —— 是**模型体（去注释）**逐字节相同 | 模型/斜装 |
| `d7c4da0` | 2026-10-08 | docs(fidelity): §M —— robot11 角速度通道的归因与修复（逐候选判决 + 前后数字 + 新验收口径） | 模型/斜装 |
| `9de6b72` | 2026-10-08 | feat(tools): 可达性前置判据（--preflight-reach / --reach-gate）+ 原地转目标 + 角速度台架 | 工具 |
| `ef927e2` | 2026-10-08 | fix(robot11): 底盘塌成单刚体 —— planar_move 的 Set*Vel 只对单刚体自洽，角速度通道 1.03%→16.2% | 模型/斜装 |
| `de3b78c` | 2026-10-08 | docs(fidelity): §L —— `robot:=robot11` 的 `--goal-forward 2.0` 走不到：归因（实体障碍 + 角速度通道 0.55%），§K.8 第 5 项收口 | 模型/斜装 |
| `339e205` | 2026-10-08 | fix(tools): tilt_mount_probe 的 gt_yaw_delta_deg 把"度"又 degrees() 了一次（多乘 57.2958） | 模型/斜装 |
| `94f0d6b` | 2026-10-08 | feat(tools): §L 取证链 —— nav_goal_forensics 探针（plan/代价图/action 状态机/BT/速度 + 绕过 nav2 的 --push） | 导航/限速 |
| `6f6a014` | 2026-10-08 | chore(tools): _batch_k2.sh 每跑后立即校验 frames 非空并把 probe.json 另存 <tag>.done.json（批次中断也知道哪几跑是好的） | 工具 |
| `4964361` | 2026-10-08 | docs(fidelity): §K.8 补记 sensor 档插件 A/B 的数据缺口现象（probe frames={} 但 linefit/限速正常） | 模型/斜装 |
| `8e73c8b` | 2026-10-08 | docs(fidelity): §K.4.2 补上 robot11 的插件偏移 A/B（旧 vs 新：地面峰 -0.280 到 -0.155、圆内 >=99 13 到 0） | 模型/斜装 |
| `18f2738` | 2026-10-08 | docs(fidelity): §K 代价图侧收尾（高度带定基 / 近地剔除 / 半径与足印 / 插件 0.1 m 偏移）+ 取证工具 | 模型/斜装 |
| `1403f42` | 2026-10-08 | fix(livox_plugin): 点 = range*axis 漏了射线起点（minDist*axis + offset.Pos()），默认修好 + 回退开关 | 仿真/插件 |
| `1c89473` | 2026-10-08 | fix(nav2): robot11 槽位代价图几何 外接0.3565/0.70 改 内切0.300/0.60（+ 高度带"为什么不动"的注释） | 模型/斜装 |
| `360d493` | 2026-10-08 | feat(perception): 近地剔除（obstacle_near_ground_m，按槽位）—— 代价图 2D 链路唯一能用上高度的那一级 | 感知/代价图 |
| `d4965bd` | 2026-10-08 | docs(nav2): 纯 C++ 规划库接入 nav2 的三形态决策/契约/效率账 + CMake 骨架（纯新增） | 导航/限速 |
| `7542973` | 2026-10-08 | docs(fidelity): §J 两个 LIO bug 的修复 + robot11_mount:=sensor 的 A/B 实测（纯追加） | 模型/斜装 |
| `8375963` | 2026-10-08 | feat(tools): tiltmount 探针扩展（--drive/--goal/RTF/代价图圆/分带/契约）+ 两个新离线工具 | 模型/斜装 |
| `1116b05` | 2026-10-08 | feat(sim): 插件新增 <cloud_frame> 开关 + robot11_mount:=sensor（"物理斜装 + 账也对"，纯增量） | 模型/斜装 |
| `2a5a8b6` | 2026-10-08 | fix(small_point_lio): 修 bug②「点云多乘一次 T(base←livox)」+ bug③「odom→base 姿态用共轭」 | 仿真/插件 |
| `527fc10` | 2026-10-07 | feat(tools): 新增 tools/scripts/tiltmount/ —— 斜装雷达「帧/射线/外参/外方位」取证工具（纯增量） | 模型/斜装 |
| `a7497f0` | 2026-10-07 | docs(fidelity): §I 斜装雷达物理对不对/帧与外参账逐项判决（H1/H2/H3）+ RViz 配方 + 精确改法（中文） | 模型/斜装 |
| `c7f57d8` | 2026-10-07 | docs(gazebo): §5.1.6 无头二次独立复核 + §14/§15 两处更正与交叉链接 | 仿真/插件 |
| `6c9ae96` | 2026-10-07 | docs(fidelity): 新增《雷达"斜 30°"到底斜在哪儿》逐帧点账本 + 代价图归因 + mount 开关（中文） | 模型/斜装 |
| `e2bcf3d` | 2026-10-07 | feat(gazebo): robot:=robot11 的 model:// 解析自动化（launch 侧 + 包 export）+ 无头复核工具 | 模型/斜装 |
| `b7d625b` | 2026-10-07 | feat(robot11): 新增 robot11_mount:=plugin|urdf 安装方式开关（默认不变）+ 取证工具 | 模型/斜装 |
| `c12cdb8` | 2026-10-07 | docs(gazebo): 更正 §2.2 标题里的时间戳格式（是 `(秒 纳秒)`，不是微秒） | 仿真/插件 |
| `3737c23` | 2026-10-07 | docs(robot_models): §12.7 未验证项 ① 追加交叉链接（指向 gazebo GUI 另案结清） | 模型/斜装 |
| `96974db` | 2026-10-07 | docs(gazebo): 新增《gzclient 加载不出来》取证/机理/实测/修法（中文） | 仿真/插件 |
| `3b48c7c` | 2026-10-07 | feat(launch): 新增 `gui:=True|False` 与 `gazebo_offline:=True|False` 两个开关（纯增量） | 仿真/插件 |
| `e060a98` | 2026-10-07 | docs(robot11): §13.9 更正 `/scan` 分带百分比的分母（用"有限波束 833 条"，不是 693） | 模型/斜装 |
| `15175ea` | 2026-10-07 | fix(robot11): 生成物文件头里的整车面数笔误 2,870,498 → **2,821,320**（= 12 个 STL 求和，§9.2） | 模型/斜装 |
| `bf0353b` | 2026-10-07 | docs(robot11): §13 Phase 5 —— 上网查到的标准做法（带 URL）+ 抽稀流水线 + 三档代价实测 + 开关 + 验收 + 未验证 | 模型/斜装 |
| `4ec728e` | 2026-10-07 | feat(robot11): `decimated` 档改指**专用视觉 LOD** + 开关契约检查工具 + 三处不再骗人的横幅（Phase 5） | 模型/斜装 |
| `646277f` | 2026-10-07 | feat(robot11): `<visual>` 的 QEM 视觉 LOD —— 抽稀工具 + 12 个抽稀件 + 量化验证清单（Phase 5） | 模型/斜装 |
| `d2ac1cf` | 2026-10-07 | docs(robot11): §12 Phase 4 —— GUI 代价表（full vs decimated，实测）+ 自击掩膜前后数字 + 接线运行期证据 + 诚实更正 + 未验证清单 | 模型/斜装 |
| `cf60b84` | 2026-10-07 | fix(bringup): robot11 —— 更正骗人的横幅、补运行期接线证据、视觉档位开关（默认抽稀）、GUI 代价实测工具 | 模型/斜装 |
| `1ca09eb` | 2026-10-07 | feat(perception): 自击掩膜 —— 把近场自身回波剔出「坡度/台阶/限速」判据（默认关，robot11 槽位开） | 模型/斜装 |
| `23c1dc8` | 2026-10-07 | docs(robot11): §11.2.1 补一条共享代码回归证据（默认模型在插件+linefit 修复后与 Phase 2 数字一致） | 模型/斜装 |
| `a1d9c15` | 2026-10-07 | docs(robot11): §11.9 补 nav2 覆盖层的**运行时**证据（robot_radius 0.3565 生效 + controller/planner active + 目标被接受，但车未走 ⇒ 列入未验证） | 模型/斜装 |
| `d4e8232` | 2026-10-07 | feat(robot11): Phase 3 —— 让雷达真的看得见（A 方案：碰撞挖视锥）+ 斜置感知链槽位化 + nav2 几何覆盖 | 模型/斜装 |
| `82e4946` | 2026-10-07 | feat(sim): livox 射线插件支持"安装倾角"<tilt_rpy>（缺省单位阵 ⇒ 其它模型逐字节不变） | 模型/斜装 |
| `0142bd9` | 2026-10-07 | fix(perception): linefit 的 gravity_aligned_frame 会把点云塌到原点（Eigen::Affine3d 未初始化） | 感知/代价图 |
| `599fc09` | 2026-10-07 | feat(robot11): `robot:=robot11` opt-in 槽位（生成式 xacro）+ 雷达 30° 的实测判定 + 实测评估 | 模型/斜装 |
| `f361f00` | 2026-10-07 | feat(robot11): 用户哨兵 robot11 的 mesh 资产包 + 逐 mesh 量测 + base_link 碰撞策略（实测） | 模型/斜装 |
| `712741f` | 2026-10-07 | docs(robot): docs/robot_models.md —— 模型槽位 + 用户哨兵 URDF 的实测评估（+ 索引与 3 处交叉引用） | 模型/斜装 |
| `a2c6349` | 2026-10-07 | feat(tools): 机器人模型槽位的实测/复算/断言工具（无头跑·只读探针·离线 linefit·provenance） | 工具 |
| `d3940b5` | 2026-10-07 | feat(bringup): robot:=<模型> opt-in 槽位（默认 ''= 现行模型，逐字节行为不变）+ 按槽位切标定 | 模型/斜装 |
| `d6232ea` | 2026-10-07 | feat(robot): robot:=hzmirm 的机器人模型 —— 用户哨兵 URDF 逐字接入 + 我们补的 inertial/IMU/雷达/底盘 | 模型/斜装 |
| `95846c3` | 2026-10-07 | docs(map_assets): 地图/先验资产规范 + 逐个清单 + 实测对照 + 换图/回滚流程（+ 索引与 5 处文档同步） | 建图/资产 |
| `c8d65f9` | 2026-10-07 | chore(map): 退役的合成资产搬进 map/attic 与 PCD/attic（只搬不删）+ 两处 README | 建图/资产 |
| `2192024` | 2026-10-07 | feat(tools): map_prior_ab.py —— 两张 2D 先验候选的公共窗口/逐格/边沿对照（离线，无 ROS） | 建图/资产 |
| `7f9e354` | 2026-10-07 | feat(map): RMUC2026 默认先验对按实测提升到 v3 会话产物（旧份留 *.bak-20261007） | 建图/资产 |
| `6ca978e` | 2026-10-07 | docs(lio): 「没撞击却飘里程计」归因报告 + 限速/角速度 A/B + §2.3 更正 | 定位/LIO |
| `4adf0d8` | 2026-10-07 | feat(bringup,perception): 前瞻限速提速档（peak 45 / floor 0.60 / 死区 5.5°+0.06 m） | 感知/代价图 |
| `dad39c5` | 2026-10-07 | feat(tools,diag): 「没撞击却飘里程计」专用仪器 —— 只读时间轴 + 归因分析器 + 无头跑法 | 工具 |
| `72155e4` | 2026-10-07 | docs(slope_speed): §5 补一条没走通的路 —— 用 /odom 速度符号判方向没触发（根因未定位，默认改双向窗口） | 感知/代价图 |
| `fc226bb` | 2026-10-07 | docs(slope_speed): 前瞻坡度/台阶限速报告 + traversability_plan §9 状态更新 | 感知/代价图 |
| `71f6c87` | 2026-10-07 | feat(tools,regress): 限速真源/物理自检 + A/B 指标提取 + 离线合成扫掠台架 | 工具 |
| `bb0fe03` | 2026-10-07 | feat(bringup): 可通行性真源新增 speed_limit_* 一组键（前瞻限速的表与物理锚点） | 其它 |
| `ca89b69` | 2026-10-07 | feat(perception): 前瞻坡度/台阶限速 —— 连续量 → 速度上限（发 nav2 SpeedLimit，命令链零新增节点） | 感知/代价图 |
| `f78a685` | 2026-10-07 | docs(slots): §12 补一条更稳的重跑路线 —— 直接存跑图那次的 slam_toolbox /map（对点云垂向拖影免疫，本次实测 10.1 m² 占用 / 可行驶斜面误占 1.82%） | 建图/资产 |
| `819160c` | 2026-10-07 | docs: 缺陷①②③的修复报告 —— 实测前后、整栈 nav 复跑、重跑建图 recipe、回退 | 文档 |
| `97d0dbc` | 2026-10-07 | feat(tools,regress): 低矮地形实测/验收装置（同帧 A/B + 建图捕获 + 重跑建图 + 图侧验收） | 工具 |
| `cb3a756` | 2026-10-07 | fix(perception): p2l 高度带 max_height 0.1→1.0（离地 0.326→1.226 m）+ 离线判据改走真源 YAML | 感知/代价图 |
| `3ea5dd4` | 2026-10-07 | feat(perception): 实时「坡度/台阶」判据（rm_ground_traversability）+ 两个地面分割槽位接入 | 感知/代价图 |
| `4da000e` | 2026-10-06 | docs(clearance): §2.2 补一条独立佐证（map_opt p05 −0.61 m：雷达看见了图上更近的几何） | 感知/代价图 |
| `2e7cf61` | 2026-10-06 | docs(clearance): 跑次口径统一为 11 次跑 / 9 个配置（base 与 v_ginfl 各跑两次） | 感知/代价图 |
| `acb93c1` | 2026-10-06 | docs(clearance): §3 表两处数字校正（base 健康段 IMU 80.8/5、v_rpp 28.2/0）+ 按撞击量级重写读法 | 感知/代价图 |
| `4a7996c` | 2026-10-06 | fix(nav2): smac2d 平滑器 w_smooth 0.4→0.0（不再把 A* 的余量拉回内切带） | 导航/限速 |
| `6f0c27a` | 2026-10-06 | docs(clearance): 贴膨胀边/撞击/定位失效的实测归因报告（12 次跑 + 场地网格对账） | 感知/代价图 |
| `1811149` | 2026-10-06 | feat(tools,regress): 余量/接触/定位 三合一实测装置（贴膨胀边与撞击的量化基线） | 工具 |
| `25e33ad` | 2026-10-06 | docs(gicp): 发散与抖动复现/机制/修复/A-B 报告（+ 槽位与算法矩阵指针） | 定位/LIO |
| `642c174` | 2026-10-06 | feat(tools,gicp_bench): 发散/抖动 A/B 装置 + 场地自相似性探针 | 定位/LIO |
| `4f29d35` | 2026-10-06 | fix(gicp_registration): 接受判据三道 + 环路内低增益滤波 —— 治 map→odom 自传播发散与"到点后四处抖动" | 定位/LIO |
| `42370d7` | 2026-10-06 | docs(continue_mapping): §11 事故复盘②「save 把位姿图写到别人那套会话的名字上」 | 建图/资产 |
| `02306fb` | 2026-10-06 | fix(mapping): save 严格解析会话名（活栈优先 / 多会话拒绝 / --check），不再信任 .session.yaml | 建图/资产 |
| `9d180ff` | 2026-10-06 | feat(bringup): 会话播报器 —— 把「本次 launch 是谁」挂到 ROS 图上（存档事故修复之一） | 装配/参数 |
| `c7260da` | 2026-10-06 | docs(slam-toolbox): 丢帧 × 回环 × 里程计两假设的实测裁决（A 不成立 / B 买的是 TF 时延不是精度） | 建图/资产 |
| `d0fe0a0` | 2026-10-06 | feat(diag): 丢帧 × 回环 × 里程计 A/B 装置（只读仪器 + 真值里程计桥 + 归因器） | 工具 |
| `7d03f16` | 2026-10-06 | docs(diag): LIO 漂移分段诊断报告 —— 四个候选（正常累积/卡死退化/转弯/续建污染）逐条对账 | 工具 |
| `e866cfa` | 2026-10-06 | feat(diag): LIO 漂移分段诊断工具 —— 分段脚本化路线 + 50Hz 同步时间轴 + 逐段 ATE | 工具 |
| `3a2b54d` | 2026-10-06 | docs(slam-toolbox): §5 补三条实测反例（只放松 chain / 放宽 chain+半径+阈值 / 指望纠 LIO 大跳变），§7 修正路线复盘 | 建图/资产 |
| `5cb93cb` | 2026-10-06 | fix(slam-toolbox): 建图节点加密（min_travel 0.5→0.2 等），让回环真的能触发 | 建图/资产 |
| `d3dde82` | 2026-10-06 | docs(slam-toolbox): 建图漂移/回环调参考证 + 节点密度 A/B + sltune 实验工具 | 建图/资产 |
| `db457e1` | 2026-10-06 | docs(mapping): continue_mapping 补 数据卫生/覆盖前备份/续建一致性检查 + 事故复盘（含 2026-10-06 实测） | 建图/资产 |
| `133742d` | 2026-10-06 | feat(mapping): 续建后一次性一致性检查 map_resume_check（对不上就喊，strict 直接收栈） | 建图/资产 |
| `59eb110` | 2026-10-06 | feat(perception): cloud_accumulator 数据卫生（高度带/跳变速度闸/TF退化帧跳过）+ 写前体检与备份 | 感知/代价图 |
| `15d5c57` | 2026-10-06 | feat(mapping): 存档覆盖前自动备份 + restore/backups（+ save 后独立 PCD 体检） | 建图/资产 |
| `c8d8776` | 2026-10-06 | docs(mapping): 建图与重定位调研 —— 术语考据、会/不会用重定位的系统清单、本栈现状与缺口 | 建图/资产 |
| `adc2622` | 2026-10-06 | docs(mapping): 新增 docs/continue_mapping.md（续建/存档/场地隔离全流程）+ 三处指针 | 建图/资产 |
| `0a9c5b5` | 2026-10-06 | feat(mapping): mode:=mapping 支持「上次基础上继续建图」+ 换场地续建默认拒绝 | 建图/资产 |
| `8b14aec` | 2026-10-06 | feat(perception): cloud_accumulator —— 3D 先验也能跨会话续建（map 系累积 + 同名存档 + 同一份场地守卫） | 感知/代价图 |
| `10f2a84` | 2026-10-06 | feat(mapping): 地图存档守卫 map_asset_guard.py + map_archive.sh（save/info/list/adopt） | 建图/资产 |
| `5c3d7af` | 2026-10-06 | docs(timestamp): 全仓时间戳构造审计 —— 1 ns 截断类 bug 与戳/队列/QoS 风险 | 定位/LIO |
| `5410412` | 2026-10-06 | feat(diag): 逐纳秒戳差仪器 —— 探针加整数 ns 指标 + 离线复算器 | 工具 |
| `a281636` | 2026-10-06 | fix(small_point_lio): TF/odom 的戳由"截断"改"四舍五入"，修掉 /scan 被 tf2 QueueFull 顶掉 | 定位/LIO |
| `c73e9d0` | 2026-10-06 | docs(mapping): /scan 丢帧根因考证（1 ns 戳错位）+ 时间戳链实测 + scan_queue_size A/B | 建图/资产 |
| `fc3382d` | 2026-10-06 | docs(traversability): 新增 3D 派生可通行性设计计划（坡度/落空/净空，plan only） | 感知/代价图 |
| `b079175` | 2026-10-06 | docs(mapping): 补 §10「为什么 /scan 看不见坡道」——给 bench 使用者的链路解释 | 建图/资产 |
| `7afbe01` | 2026-10-06 | feat(mapping): 2D 先验图改从 3D 点云投影（pcd_to_nav2_map）+ 建图稳健性 A/B（丢帧/跳变哨兵/坡道段判定） | 导航/限速 |
| `1a9ce8c` | 2026-10-06 | feat(ground): 新增地面分割槽位 ground（linefit|patchwork，默认 linefit）+ vendored Patchwork++ | 感知/代价图 |
| `8170abe` | 2026-10-05 | docs(mapping): 明确 --pose-source gt 的含义/理由/代价（跑图用仿真真值闭环，LIO 质量另用 ATE 单独报） | 建图/资产 |
| `5bee8fb` | 2026-10-05 | feat(mapping): lio:=small_point_lio 建图打通 + RMUC2026 先验产物（2D+PCD，gicp 验证 PASS） | 定位/LIO |
| `f3e4c0b` | 2026-10-05 | docs(readme): docs/ 索引表补一行 worlds.md | 文档 |
| `eb7b811` | 2026-10-05 | feat(tools): 场地 STL ⇒ world + 2D 图 + PCD 一条命令（tools/scripts/world/stl_to_world.py） | 工具 |
| `7e7b823` | 2026-10-05 | docs(worlds): RMUC2026 世界说明 —— 资产/转换/出生点/地图来源/实测结论/坑 | 文档 |
| `280888e` | 2026-10-05 | feat(nav): RMUC2026 的 2D 栅格图与先验 PCD（从 STL 几何生成，非实跑建图） | 导航/限速 |
| `7c0006b` | 2026-10-05 | feat(sim): 新场地 world:=RMUC2026 —— 由附件 STL 生成的世界/模型/mesh + 出生点 | 模型/斜装 |
| `b2c8e1a` | 2026-10-05 | docs(stvl_local): 局部 STVL 的 A/B 实测（四次跑）+ 结论 + 回退 + 未验证清单 | 文档 |
| `4798ea2` | 2026-10-05 | feat(nav): local_obstacle 新增 stvl / stvl_both —— 局部代价地图也上 STVL | 导航/限速 |
| `d42a413` | 2026-10-05 | docs(small_point_lio): 全栈闭环实测（A/B）+ 调参结论 + launch 缺包修复，替换"调参中/全栈未验证" | 定位/LIO |
| `607dcab` | 2026-10-05 | fix(launch): 槽位缺包不再挡住所有组合 —— 包查询改惰性解析（small_point_lio/icp/gicp） | 定位/LIO |
| `830c5fb` | 2026-10-05 | docs(lio_slots): 扫参示例命令改用随仓库提交的 tools/lio_sweep_configs/（.tmp_sweep 是临时目录，不该写进文档） | 定位/LIO |
| `680a380` | 2026-10-05 | docs(lio_slots): 调参后重复性补第 4 次实测（6.88/6.90/6.92/6.93 m，末位置逐位一致） | 定位/LIO |
| `2ff42d4` | 2026-10-05 | tune(small_point_lio)+docs: 仿真参数调优 —— 只改 4 键，轨迹 30.2 m → 6.9 m（≈ FAST-LIO 参照） | 定位/LIO |
| `b3995b5` | 2026-10-05 | feat(lio)+tools: small_point_lio 离线扫参驱动（并行、多窗口）+ 判读脚本补 JSON/轨迹输出 | 定位/LIO |
| `2ba6625` | 2026-10-05 | docs(algorithm_matrix): §9.2.3 表改用 localization:=small_gicp 命名，§9.2.2 补历史口径说明 | 定位/LIO |
| `30b4e8e` | 2026-10-05 | docs(algorithm_matrix): 五路对照改用 localization:=small_gicp，删掉"临时改 YAML + git checkout"旧流程 | 定位/LIO |
| `2c42339` | 2026-10-05 | feat(localization)+docs: 新增 localization:=small_gicp 槽 —— backend 由 launch 注入，槽位层面与 gicp 并列 | 定位/LIO |
| `6bfea21` | 2026-10-05 | docs(lio): 登记 lio:=pointlio 的节点级复核事实 + lio:=small_point_lio 的真实状态（精度未达标） | 定位/LIO |
| `680a834` | 2026-10-05 | docs(localization): beluga SIGINT abort 的 respawn 审计（结论：不改代码）+ gicp 两后端并列 | 定位/LIO |
| `0a2e74d` | 2026-10-05 | feat(lio)+docs: 新增 lio:=small_point_lio 槽 + 节点级验证工具与实测记录 | 定位/LIO |
| `98baa55` | 2026-10-05 | vendor(small_point_lio): 引入 Yancey2023/small_point_lio@688d75c（MIT）+ timebase 兜底补丁 + 仿真参数 | 定位/LIO |
| `c2cb002` | 2026-10-05 | docs(algorithm_matrix)+§9.2: 新构建口径下的五路对照 —— 五路全 PASS、missed-rate 全 0、small_gicp align 快 5.4× | 定位/LIO |
| `20d1fb1` | 2026-10-05 | feat(regress): nav_smoke_regression.py 加 --localization <NAME> —— 让快照自证是哪一路定位 | 定位/LIO |
| `c7bcd6d` | 2026-10-05 | fix(gicp_registration)+docs: /initialpose 被"点云 + align"饿死 —— handoff/apply 拆分 + 第三个回调组（22.7 s → 0.24 s） | 定位/LIO |
| `6ae2ada` | 2026-10-05 | build+docs: 修构建口径 —— colcon 默认不设 CMAKE_BUILD_TYPE（6 个包整个 -O0），提交 colcon_defaults.yaml；实测 PCL GICP 369.9→11.3 ms（32.7×） | 文档 |
| `c655d77` | 2026-10-05 | fix(localization)+docs: localization:=beluga 启动即崩 —— 删掉 laser_min_range(-1.0)，beluga 对该键声明的是 [0, DBL_MAX] | 定位/LIO |
| `d60c21e` | 2026-10-05 | feat(gicp_registration)+docs: 可切换配准后端 backend=small_gicp（vendored koide3/small_gicp@v1.0.1）—— 默认仍是 pcl | 定位/LIO |
| `21fc301` | 2026-10-05 | feat(localization)+docs: 新增 localization:=beluga 槽（beluga_amcl 2.1.1 = AMCL 的现代实现） | 定位/LIO |
| `5c9c0a0` | 2026-10-05 | feat(analysis)+docs: Scan Context 同源性硬门（离线 bag 对拍）—— 同源性 PASS，严格判据 INCONCLUSIVE(偏 PASS) | 工具 |
| `b626f37` | 2026-10-05 | feat(analysis)+docs: Scan Context A1 资产探针与报告 —— 判定 GO（有条件） | 工具 |
| `413415d` | 2026-10-05 | docs: 新增 Scan Context 全局重定位方案（738 行，A~F） | 文档 |
| `34c1eb3` | 2026-10-05 | docs(algorithm_matrix §九): 记录本轮重定位线结果 + §9.1 三方对比协议 | 文档 |
| `4445fe3` | 2026-10-05 | feat(regress): 新增 compare_regress_snapshots.py —— 回归快照一键对比表 | 工具 |
| `708b35d` | 2026-10-05 | docs(localization_slots §7): 记录 gicp 验收结论（/tf 满速、fitness 2.3e-3 m²、PASS 3.2s/0 恢复）+ 五条遗留（settle 阈值偏严、总算力不足、iterations 不降 CPU、资产退化、三方对比暂缓） | 定位/LIO |
| `8b47ff9` | 2026-10-05 | fix(gicp): TF 定时器搬进独立回调组 + MultiThreadedExecutor（治 350 ms align 把 50 Hz TF 饿到 2.3 Hz ⇒ nav2 extrapolation） | 导航/限速 |
| `098078d` | 2026-10-05 | tune(gicp): 两级下采样 leaf 0.10/0.05 + max_corr 1.5（照抄 COD 2025 small_gicp_relocalization，治 map→odom 30 s 漂 8 cm） | 定位/LIO |
| `c8863f1` | 2026-10-05 | tune(gicp): tf_lookahead_sec 0.3 -> 0.45（覆盖 CPU 过载时 TF 定时器被饿死的最坏情况） | 定位/LIO |
| `629c971` | 2026-10-05 | feat(regress): nav_smoke_regression 发目标语义对齐人类 RViz（yaw auto=车当前朝向 / 等定位稳定 / 目标可用性预检） | 工具 |
| `36a71cf` | 2026-10-05 | fix(gicp_registration): map→odom/~/pose 盖 now+tf_lookahead_sec（≈AMCL transform_tolerance），修 nav2 "extrapolation into the future" | 导航/限速 |
| `28deaf1` | 2026-10-05 | feat(gicp_registration): 新增 GICP 重定位槽 localization:=gicp（只发 map→odom + 健康信号） | 定位/LIO |
| `47d7411` | 2026-10-05 | feat(params): goal checker 改 PositionGoalChecker（全向车位置-only） | 装配/参数 |
| `f033d96` | 2026-10-05 | fix(icp_registration): 点云订阅改用 SensorDataQoS(BEST_EFFORT)，/initialpose 保持 RELIABLE | 定位/LIO |
| `53f7a58` | 2026-10-05 | docs(定位槽位): 决策记录 —— 先试 icp；slam_toolbox 暂缓（保留入口，附客观补充：更新率不低、短板在资产维护与全局重定位弱）；gicp 提上日程 | 定位/LIO |
| `b467990` | 2026-10-05 | docs: 新增重定位槽位登记表（localization_slots.md） | 定位/LIO |
| `48703e6` | 2026-10-05 | fix(amcl): 高速跟踪参数 —— transform_tolerance 1.0→0.3 / update_min_d 0.25→0.05 / update_min_a 0.2→0.05 / recovery_alpha_slow 0.0→0.001 / recovery_alpha_fast 0.0→0.1 | 定位/LIO |
| `7097602` | 2026-10-05 | docs: 新增 Kinodynamic_esdf_mpc 调研笔记（270 行，含与 MPPI 的本质差异、引入成本与许可风险） | 文档 |
| `bbe7d43` | 2026-10-05 | refactor(params): nav2 参数从「一个组合一份 full copy」改为 base + 槽位文件（8/8 组合逐键 0 差异） | 导航/限速 |
| `419df04` | 2026-10-05 | feat(params): 补齐 planner×nav 组合文件（dwb/teb/mppi × smac2d） | 导航/限速 |
| `09578ed` | 2026-10-05 | docs: 补入 6 份调研文档（此前只写在磁盘、未入库） | 文档 |
| `effb443` | 2026-10-05 | fix(params): smac2d 变体同步 robot_radius 0.40 -> 0.22（与 dc03e11 对齐） | 导航/限速 |
| `dc116c0` | 2026-10-05 | feat(nav2): 新增 nav:=mppi 槽位（默认仍是 rpp ⇒ 现有已验证路径零影响）—— MPPI 控制器 + 逐键核实 1.1.20 + model_dt=1/controller_frequency | 导航/限速 |
| `50b51e9` | 2026-10-05 | docs(工单 §I): 卡住恢复链核查（只读，不改代码）—— 内置 BT/behavior_server/1.1.20 插件逐项核实 + 用户可执行的验证步骤 + 6 条缺口 | 文档 |
| `dc03e11` | 2026-10-05 | fix(nav2): robot_radius 0.40 → 0.22（回到真实车体半径）—— 消除 costmap 几何 ≠ 真实几何 导致的「一进带就卡死」 | 感知/代价图 |
| `2017f3a` | 2026-10-05 | docs(工单 §H): 记录 planner:=smac2d 实施（commit d4e662b、12 个已核实键、剔除 refinement_num 等）+ 新坑『新增 params 文件必须先 colcon build』+ 验收清单 | 导航/限速 |
| `d4e662b` | 2026-10-05 | feat(nav2): 新增 planner 槽位（默认 navfn ⇒ 现有已验证路径零影响）—— A/B COD 2026 风格 SmacPlanner2D | 导航/限速 |
| `c8f8868` | 2026-10-05 | docs(工单 §G): small_point_lio 排期约定 —— 可后面补且不阻塞 D3/D4；三条风险均为可控工程量；零成本预研动作=独立进程验证'喂得进去' | 定位/LIO |
| `2aca994` | 2026-10-05 | docs(工单 §F): 记录候选实验「两层 STVL」（全局+局部）—— 含他们参数、五个连带效应（时间维/吃原生点云/是删地面分割的前提/必须配裁剪盒/清除来源增加）、我们的现状与分两步尝试方案+判据+回退 | 文档 |
| `a4d7f40` | 2026-10-05 | docs(工单 §E): 决策记录 —— 静态 map→odom 不采用 / linefit 保留 / small_point_lio 提升为 lio 同级槽位（含落地清单与硬门槛） | 定位/LIO |
| `c3b0c96` | 2026-10-05 | docs: COD_NAV 思路整理（重点定位层特殊性）—— 逐层做法/理由/代价 + 定位层为何特殊 + 内在自洽性 + 对 D3/D4 的启示 | 文档 |

---

# §8 术语表（一页）与"接下来该做什么"

## §8.1 术语表（本仓口径）

| 术语 | 一句话定义 |
|---|---|
| **槽位（slot）** | launch 参数化的**可选实现**：`world:=` / `mode:=` / `lio:=` / `mapper:=` / `localization:=` / `nav:=` / `planner:=` / `ground:=` / `local_obstacle:=` / `robot:=` / `robot11_mount:=` / `robot11_visual:=`。**槽位的默认值 = 现行行为**，新增槽位不许影响别的槽位 |
| **`mode:=`** | `mapping`（建图/续建）／`slam_nav`（边建边导）／`nav`（用先验图导航） |
| **真值（gt）** | Gazebo 的世界位姿（`/odom_ground_truth`，**不含 TF**）；跑图时可用 `--pose-source gt` 闭环，把"建图质量"与"LIO 质量"分开量 |
| **RTF** | Real-Time Factor（`/clock` 口径）。⚠️ **只能同批比较**，跨批排序不可复现 |
| **ATE** | Absolute Trajectory Error（对齐后 vs 真值），本仓报 RMSE / max / 每米漂移 |
| **创新** | `本帧配准结果 ⊖ 本帧初值` = GICP 这一帧**想施加的修正量**（平移模长 + 等效 yaw） |
| **inscribed / circumscribed radius** | 内切半径（凸包边到原点最近距离，"能走多窄"）／外接半径（凸包顶点最远，"保证不撞"）。nav2 的 `robot_radius` 语义是**内切** |
| **lethal / inscribed / 膨胀** | nav2 `OccupancyGrid` 取值：**100 = lethal、99 = inscribed、0 = free、−1/255 = unknown**；1~98 是**膨胀层经过翻译的值**，**不要当 cost 读** |
| **高度带** | `p2l.min/max_height`（**点云自带帧**）／`obstacle_layer.min/max_obstacle_height`（**代价图帧**）。两者**帧不同、语义不同**（§0.3） |
| **近地剔除** | `obstacle_near_ground_m`：`dz = z − 局部地面 g(x,y)` ≤ 阈值的点从 obstacle **改判** ground（投影成 2D **之前**） |
| **自击（self-hit）** | 雷达打到自身结构的真实回波（robot11 约 **29%** 的点，中位半径 0.021 m）。**它是物理真实的**，掩膜只把它剔出**判据**，不动 `/segmentation/*` 标签 |
| **自击掩膜** | `self_mask_*` = 113 个 collision AABB ∪ 近场死区（`r_xy ≤ 0.2416` 且 `z ≥ −0.2295`） |
| **走廊 / 承诺特征 / 距离前推** | 限速器的前瞻区间（车前/车后 1~3 m，半宽 0.28 m + 张开 8°）；"承诺"= 记住某个特征的 `v_req` 与距离；"前推"= 每帧按 `上一帧上限 × hold_decay_factor × dt` 扣减剩余距离 |
| **单刚体 / articulated** | 底盘结构两档：`rigid`（把 `j2…j11` 塌成一个刚体，**`planar_move` 自洽**）／`articulated`（回退档，模型体去注释后逐字节等于旧版） |
| **QEM / 剪影 IoU / 单向表面误差** | 二次误差边折叠抽稀；三视剪影交并比（≥0.97 判据）；`out→src` 的表面误差 p99/max（抽稀件偏离原表面的量） |
| **判决性测量** | "如果假设 X 成立，我应该量到什么"——能**否掉**候选的那种测量 |
| **逐字节不变** | 默认路径的行为承诺（三种实现机制见 §2.6.2）；⚠️ 精确表述常常是"**模型体（去注释）**逐字节相同" |
| **未验证清单** | 每份报告末尾的诚实清单（本文 §5 是合并版） |
| **`.tmp_*`** | 跑出来的原始数据目录（**不入库**，已在 `.gitignore`）；引用文档时只能当"可复算的凭据"，不能当仓库资产 |
| **资产命名** | `map/<world>.pgm\|yaml`（当前 2D 先验）／`PCD/<world>.pcd`（当前 3D 先验）／`<world>_<tag>.{posegraph,data,meta.yaml}`（会话存档，一个 tag = 一次会话）／`.prev-<YYYYmmdd-HHMMSS>`（工具自动备份）／`.bak-<YYYYMMDD>`（人工提升备份）／`attic/`（只搬不删） |
| **哨兵 / 云台 / 小陀螺** | RM 哨兵机器人的云台（`j10/j11`）与"边转边打"行为；本仓用 `base_link_fake` + `fake_vel_transform`（`spin_speed`）实现。⚠️ robot11 的云台**没有控制器**（`spin_speed` 对它设 0.0） |
| **坡脚 / 台阶残差** | 坡度突变处（撞击来源）／"格内抬升(p90) − 该格坡度能解释的部分"（窄尺度尖台阶） |
| **SFC / min-snap** | 方形走廊（Safe Flight Corridor）／最小 snap 多项式轨迹（`tdt-nav-kit` 的后端，**研究未落地**） |

## §8.2 接下来该做什么（候选清单，按"阻塞程度"排序）

| 优先级 | 要做什么 | 为什么 | 从哪开始 |
|---|---|---|---|
| **P0** | **把"pitch 还是 roll"用实物/装配图彻底定死** | 目前默认 roll 的依据是"CSV 更可信 + 用户按实物确认"，但**证据仍是单点的**；若实物是 pitch，射线/盲区等价但"最朝下方位"完全不同（±x vs ±y） | `robot_models.md` §10.5 的方位直方图判据 + `livox_tilt_axis:=pitch` 对照跑 |
| **P0** | **换 spawn / 换世界做真正的"长距离 + 需要转向"验收** | 当前出生点**正前方 0.42 m 就堵死**，`--goal-forward 2.0` 永远红叉；角速度通道刚修好，还没有一次真实验收 | `--preflight-reach` 先量可达区，再选目标；或换 `world:=RMUC2026` |
| **P1** | **`sensor` 档链路可用化**：把代价图帧/odom 的重力对齐问题解决（或在代价图侧换重力对齐帧） | 现在 `sensor` 档"帧↔数据自洽"但**41.2% 波束被高度带丢掉**，代价图这一层仍坏 | `Fidelity` §J.5 的两个候选（LIO 侧 z 基准 / 代价图侧高度带或 `global_frame`），**必须单独 A/B 且会动别人的参数目录** |
| **P1** | **补三类缺失的对照跑**：① `bash tools/scripts/tiltmount/_batch_k2.sh`（sensor 档插件 A/B）② `--chassis articulated` 的整栈"修前"基线 ③ `wz` 剂量-响应扫描（0.3/0.6/1.0/1.5） | 这三个直接决定"§M 的修复到底值多少" | `Fidelity` §K.8 3 / §M.10 1、3 |
| **P1** | **`sensor_height` 的 4.3 cm 疑问** | 若真偏 4 cm，`linefit` 的地面线整体抬高 4 cm（与 `max_dist_to_line 0.05` 同量级） | §K.1 末尾/§K.8 6（要查"轮 collision mesh 最低点 vs 关节 origin 的 FK"） |
| **P2** | **真足印多边形**（字符串格式）落地 | 消掉"内切圆 ⇒ 转弯角扫障碍"这个已知折中；`rcl` 那条坑已知道怎么绕（必须写成**字符串**） | `Fidelity` §K.3.3/§K.3.4 |
| **P2** | **`/scan` 里那圈"自身障碍"的处置**（p2l 侧加自击掩膜，或修插件内移） | 它可能正是"局部图被自身障碍填满 ⇒ 规划器认为车被围住"的原因之一 | `robot_models.md` §13.9 的候选修法 |
| **P2** | **限速器的两处已知缺陷**：① 承诺前推改用**真实行程**（订阅 `/odom` 速度而不是用"上限"）② 在 RPP/DWB/TEB 上验证 `setSpeedLimit()` 语义 | 前者在实车/打滑时会**提前释放**；后者决定这套限速能不能换控制器 | `slope_speed_limiting.md` §7 第 2/3 条 |
| **P2** | **全局代价图"静止也偏脏"**：raytrace/原点附近清理；"跑几分钟再看"的对照 | 影响长跑可靠性 | `Fidelity` §K.8 2 |
| **P3** | **视觉/GPU 的真机复测**：真 GPU 上看抽稀够不够、GUI 内存到底省不省 | 本机只有 llvmpipe，"省内存"这条**没测出来** | `robot_models.md` §13.4.3/§13.10 |
| **P3** | **把 patchwork 变成可选默认**：补一次 `mode:=nav` 整栈回归 | 现在它只在离线 + 建图 smoke 上验证过 | `ground_segmentation_slots.md` §7.1/§9 |
| **P3** | **两个 LIO 子模块的时间戳补丁**要不要合（`FAST_LIO` 实测同一 bug；`point_lio` 判定未实测） | 现在只报告未改 | `timestamp_construction_audit.md` §4.4/§9 |
| **P4** | **`tdt-nav-kit` 落地第一版**：纯库 + 薄适配 + `GlobalPlanner` 插件（`planner:=tdt` 槽） | 研究已完成，但 §3.6 的自测清单**一项没测** | `nav2_vs_custom_planner.md` §6.1/§7.3 |
| **P4** | **负障碍（坑/落空）与净空（限高）判据** | `p2l` 高度带抬到 1.226 m 之后"头顶结构"会成为假障碍；换场地/真机必须面对 | `traversability_plan.md` §9 的 B/C 两行 |

---

## 附：这份汇总的自我说明

* **它是"汇总"，不是"新事实"**：本文没有跑任何仿真、没有改任何代码。
  所有数字都来自 §0 开头列的源文档与 `git log`；引用时给了 `§` 或提交 hash。
* **引用优先级**：① 提交（改了什么）→ ② 原文档的**表格与结论节**（数字）→ ③ 本文 §2 的转述。
  三者若冲突，以 ① 与 ② 为准，并按 §5.4 的方式记录口径。
* **维护约定**：以后每有一条新改动，**先在原文档里写全**（现象/原因/过程/改动/原理/效果/回退/证据），
  再在本文 §2 追加一条、§4 的总账表补一行、§7 的提交索引补一行。
  **本文永远保持"一份能读完的总入口"这个定位**，不复制原文档的长篇推导。
* **建议的阅读顺序**：§0（5 分钟）→ §1.2（推翻过什么）→ §3（原则）→ 你要找的那一层（§2.x）→ §5（别把这些当成已知）。


---


## 来源 7/7：`research/system_status.md`


## 1. 四个东西各自的定位

| | 是什么 | 我们怎么用 |
|---|---|---|
| **zlio**（GPL-2.0） | 纯定位栈：PCD 建图 + 先验定位 + 重定位 | 只取**位姿**和**世界系点云**，不取语义 |
| **T-DT**（MIT） | 纯规划算法库：YAstar(A*) + SFC + MinSnap + Kastar | **算法内核，一行未改** |
| **ROGMap**（LGPL-3.0-or-later，vendor） | 3D 概率占据 + ProjectionLayer + ESDF | **障碍层**，进程内链进规划器 |
| **BIT**（Apache-2.0 根） | 整栈参考（Point-LIO + ROGMap + MINCO + MPC + BT） | **只学设计与判据**，不搬代码 |

**关键认知**：zlio 不做感知，T-DT 不做感知，中间那层（障碍语义）**两边都没有**，必须我们定义。

## 2. 现在的数据流

```
zlio: /localization/odom + /localization/lio_world_cloud（同源，map 系）
   │
   ├─► ROGMap（进程内，Planner 持有）
   │     3D 0.05~0.10 体素 + hit/miss + raycast + decay
   │     → ProjectionLayer：FREE / PASSABLE / OCCUPIED / UNKNOWN
   │
   ▼
TdtPlanner（我们适配，519 行）
   ├ L1 骨架：/map 上 A*，缓存（目标动 0.3 m 或 5 s 重算）
   ├ L2 局部：8 m，A*(势场) → DP → 梯形 → SFC(2.5) → MinSnap(order6)
   ├ 零容差门控：Bresenham 逐格，穿墙 ⇒ 拒绝/回退 A* 栅格
   └ 恢复 FSM：T1 无进度 / T2 连续失败 ⇒ 1.5 m 逃逸 ⇒ 作废骨架
   │  /plan（局部+骨架尾段）  /trajectory_plan（x/y/vx/vy/t）
   ▼
TdtLocalController（我们适配，482 行）
   整图栅格(/map∪mask) → 弧长 v(s) → 前视 4 m → Kastar(30 Hz)
   ▼
velocity_smoother[3,3,3] → fake_vel_transform → 底盘（上限 2.32 m/s）
```

## 3. 参数速查

| 层 | 参数 | 值 |
|---|---|---|
| 规划 | max_speed / max_acc | 3.0 / 3.0 |
| 规划 | corridor / shrink / order / maxDx | 2.5 / 0 / 6 / 3 |
| 规划 | TL / iter / simplify | 10 s / 6 / 0.1 |
| 规划 | two_level / global_window | true / 30.0 m（≥场地尺寸 ⇒ 全图吃动态层）|
| 规划 | use_rog_map | **false（默认，需手动开）** |
| 规划 | recovery_enable / escape | true / 1.5 m @ 0.3 m/s |
| 控制 | lookahead / vmax / amax / max_w | 4.0 / 3.0 / 4.0 / 3.0 |
| 控制 | 频率 | 30 Hz |
| 底盘 | 上限 | 2.32 m/s（40 rad/s × 0.058 m） |

## 4. 已经解决的问题（都验证过）

| 现象 | 根因 | 修法 |
|---|---|---|
| `snap=0` 每次 | T-DT origin 三处约定冲突 | 全程局部坐标 |
| 整图变障碍 | OccupancyGrid `0=自由` vs PGM `0=障碍` | `>=65` |
| 路径穿墙 | 容 2 格 × 我们薄墙 | 墙加粗 + 零容差门控 |
| 车背离目标 | 速度被旋转两次 | 删冗余旋转 |
| 画龙 | 纯 P 朝向 + 延迟 | 控制律修正 |
| 抖动 | 单级规划被动态障碍改全路径 | 两级规划 |
| mask 只喂 A* | 走廊/平滑/门控看不见 | 六处同一份 |
| terrain 274 ms | TF 查询阻塞（50 ms×3） | 改最新 TF + 5 ms |
| 可钻误判 218 格 | 遮挡（远处墙只扫到顶） | 下方需 ≥2 层被清空 |

## 5. 已知盲区（按严重度）

| # | 盲区 | 后果 | 来源 |
|---|---|---|---|
| **1** | **负障碍**（高台边缘/断崖/无支撑） | **车会开下去**，规划器认为正常 | BIT §13.2 明说 |
| **2** | 定位重定位跳变（实测 10 m / 172°） | 起点落墙内 ⇒ 门控全拒 ⇒ snap=0 | 我们日志 |
| 3 | ProjectionLayer 只用 2 个判据 | 分不清实心墙 vs 稀疏结构 | BIT 用 4 个 |
| 4 | 恢复只有 T1/T2 | 缺跟踪误差、定位不可信两条触发 | 设计文档 |
| 5 | 尾段 A* 对角切角 | 5 cm 级擦角 | 实测 4 px |
| 6 | 动态障碍无显式分割 | 只知道"有东西"，不知道是车还是墙 | BIT §13.3 |

## 6. 优先级（建议）

| 顺序 | 事项 | 理由 |
|---|---|---|
| **1** | **负障碍判断**（地面支撑/高度连续性） | 掉下去不可恢复 |
| **2** | 定位跳变门限（重定位加距离约束） | 规划器失效的根因 |
| 3 | ProjectionLayer 第 3/4 判据（垂直占据率、占据数） | 语义更准 |
| 4 | 恢复 T3/T4 | 补完已有骨架 |
| 5 | 动态障碍分割 | 需要聚类，工程量大 |
| 6 | LLM 战术 | **与导航无关**，是东大的战术层，价值另算 |

## 7. 一句话结论

> **算法内核用 T-DT，障碍层用 ROGMap，安全靠我们自己的零容差门控 + 两级规划 + 恢复 FSM；
> 现在最大的风险不是"规划不够聪明"，而是"负障碍看不见"和"定位会跳"。**


---
