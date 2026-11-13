RMUC2027 仿真场地 + 当前使用的地图  (导出时间 2026-11-03)
================================================================

目录结构（故意保持 Gazebo 的 model:// 解析布局）:

  world/
    RMUC2027_world/          ← 场地本体（world + model + STL）
      RMUC2027_world.world   ← 启动用: ros2 launch ... world:=RMUC2027
      model.config
      model.sdf
      meshes/RMUC2027.stl    ← 18.9 MB / 378,666 三角面（单位 mm，scale 0.001）
    RMUC2027_fence/          ← 围栏（纯 SDF box，无网格）
      model.config
      model.sdf
  map/
    RMUC2027_v2.yaml         ← 当前在用的地图（resolution 0.05, origin -19.9/-8.89）
    RMUC2027_v2.pgm          ← 560 x 313

怎么用:
  1) 让 Gazebo 能找到两个 model 目录（父目录加入 GAZEBO_MODEL_PATH）:
       export GAZEBO_MODEL_PATH=$PWD/world:$GAZEBO_MODEL_PATH
  2) 放到本仓库对应位置（等价于现在的布局）:
       world/RMUC2027_world/   → src/rm_simulation/hzmi_rm_simulation/world/RMUC2027_world/
       world/RMUC2027_fence/   → src/rm_simulation/hzmi_rm_simulation/world/RMUC2027_fence/
       map/RMUC2027_v2.*       → src/rm_nav_bringup/map/
     ⚠ 原仓库里 world/RMUC2027_world/meshes/RMUC2027.stl 是**符号链接**，
       指向 src/rm_simulation/hzmi_rm_simulation/meshes/RMUC2027_world/meshes/RMUC2027.stl；
       本包里已改为**实体文件**，直接覆盖即可（或把链接重新指过去）。
  3) 重建（如果是拷贝进仓库）:
       colcon build --symlink-install --packages-select hzmi_rm_simulation rm_nav_bringup

已知特征（与本次排查相关，供接手的人参考）:
  · 场地中部在离地约 0.25~0.46 m 处有一层**大面积水平/微斜面**（门洞处 z≈0.294 m），
    实测来自 STL 三角面；它在雷达(0.188 m)上方 ⇒ 地面分割必然判为 nonground，
    也是 LIO 扫描退化的主要来源。
  · 地面本身不是平面：沿 y 从 2.0→7.2 m，地面 z 从 +0.049 漂到 −0.082（≈1.4°）。
