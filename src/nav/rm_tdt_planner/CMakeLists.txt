cmake_minimum_required(VERSION 3.16)
project(rm_tdt_planner)

# tdt 源码需要 C++20（std::span / <ranges>）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 本地依赖前缀（不需要 sudo）----
# ★ 2026-11-13：src 目录重排后（src/nav/rm_tdt_planner），到仓库根从 2 层变成 3 层 ⇒ 多退一级。
set(OSQP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/mpc_proto/.deps/tdt/install" CACHE PATH "OSQP 前缀（T-DT 配方构建）")
set(OSQPEIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/mpc_proto/.deps/tdt/install" CACHE PATH "OsqpEigen 前缀（T-DT 配方构建）")
if(NOT EXISTS "${OSQP_ROOT}/lib/libosqp.so")
  message(FATAL_ERROR "缺少 OSQP：先跑 tools/mpc_proto/build.sh")
endif()
if(NOT EXISTS "${OSQPEIGEN_ROOT}/lib/libOsqpEigen.so")
  message(FATAL_ERROR "缺少 OsqpEigen：见 docs/tdt_plan_v3.md §依赖")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rm_plan_msgs REQUIRED)
find_package(rm_nav_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rog_map REQUIRED)
get_filename_component(ROGMAP_PREFIX "${rog_map_DIR}/../../.." ABSOLUTE)
set(ROGMAP_LIB "${ROGMAP_PREFIX}/lib/librog_map.a")

# tdt 内核（原样，MIT）
add_library(tdt_core SHARED
  src/tdt/yastar.cpp src/tdt/kinodynamicAstar.cpp
  src/tdt/sfcSquare.cpp src/tdt/minimumSnap.cpp)
target_include_directories(tdt_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/tdt> $<BUILD_INTERFACE:${OSQP_ROOT}/include> $<BUILD_INTERFACE:${OSQP_ROOT}/include/osqp> $<BUILD_INTERFACE:${OSQPEIGEN_ROOT}/include> $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>)
target_link_libraries(tdt_core ${OSQP_ROOT}/lib/libosqp.so ${OSQPEIGEN_ROOT}/lib/libOsqpEigen.so)
# ⚠️ 上游 yastar.cpp 用了 std::cout 但没 include <iostream>；不改上游，强制包含
set_source_files_properties(
  src/tdt/yastar.cpp src/tdt/kinodynamicAstar.cpp src/tdt/sfcSquare.cpp src/tdt/minimumSnap.cpp
  PROPERTIES COMPILE_FLAGS "-include ${CMAKE_CURRENT_SOURCE_DIR}/src/tdt/prelude.hpp")
set_target_properties(tdt_core PROPERTIES BUILD_RPATH "${OSQP_ROOT}/lib;${OSQPEIGEN_ROOT}/lib")

# planner 插件
add_library(${PROJECT_NAME} SHARED src/planner.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  ROOT_DIR="${rog_map_DIR}/../rog_map/"
  ORIGIN_AT_CORNER
  USE_ROS2)

target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${rog_map_INCLUDE_DIRS}
  ${nav2_core_INCLUDE_DIRS} ${nav2_costmap_2d_INCLUDE_DIRS} ${nav2_util_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} ${rclcpp_lifecycle_INCLUDE_DIRS} ${rcl_lifecycle_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} ${pluginlib_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
  ${rm_nav_interfaces_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${rog_map_LIBRARIES}
  "${ROGMAP_LIB}"
  tdt_core
  ${nav2_core_TARGETS} ${nav2_costmap_2d_TARGETS} ${nav2_util_TARGETS} ${nav2_msgs_TARGETS}
  rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle
  nav2_costmap_2d::nav2_costmap_2d_core
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_msgs_TARGETS} ${sensor_msgs_TARGETS}
  pluginlib::pluginlib
  ${rm_plan_msgs_TARGETS}
  rm_plan_msgs::rm_plan_msgs__rosidl_typesupport_cpp
  rm_plan_msgs::rm_plan_msgs__rosidl_generator_cpp
  ${rm_nav_interfaces_TARGETS}
  rm_nav_interfaces::rm_nav_interfaces__rosidl_typesupport_cpp
  rm_nav_interfaces::rm_nav_interfaces__rosidl_generator_cpp)

# L0 感知层（独立节点，可脱离 nav2 单独跑）
add_executable(nav_l0_perception src/nav_l0_perception.cpp)
target_compile_options(nav_l0_perception PRIVATE -fopenmp)
target_compile_definitions(nav_l0_perception PRIVATE
  ROOT_DIR="${rog_map_DIR}/../rog_map/" ORIGIN_AT_CORNER USE_ROS2)
target_include_directories(nav_l0_perception PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${rog_map_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} ${visualization_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS} ${rm_nav_interfaces_INCLUDE_DIRS})
target_link_libraries(nav_l0_perception
  ${rog_map_LIBRARIES} "${ROGMAP_LIB}" rclcpp::rclcpp gomp
  ${nav_msgs_TARGETS} ${std_msgs_TARGETS} ${visualization_msgs_TARGETS}
  tf2_ros::tf2_ros
  rm_nav_interfaces::rm_nav_interfaces__rosidl_typesupport_cpp
  rm_nav_interfaces::rm_nav_interfaces__rosidl_generator_cpp)

# Mask 生产者（patchwork 点云 → OccupancyGrid）
add_executable(mask_builder src/mask_builder.cpp)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
target_link_libraries(mask_builder rclcpp::rclcpp ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_msgs_TARGETS} ${geometry_msgs_TARGETS}
  tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2_geometry_msgs::tf2_geometry_msgs)
target_include_directories(mask_builder PRIVATE ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

pluginlib_export_plugin_description_file(nav2_core rm_tdt_planner_plugin.xml)

install(TARGETS nav_l0_perception RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/nav_stage_monitor.py DESTINATION lib/${PROJECT_NAME})
# 2026-11-12：把 STVL 的 costmap 二值化成 T-DT 消费的 /plan_mask
install(PROGRAMS scripts/plan_mask_from_costmap.py DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} tdt_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS mask_builder RUNTIME DESTINATION lib/${PROJECT_NAME})

# ★★★ 2026-11-13：**terrain_layer 已停止编译**（用户决定退役）。
#   原因：3D 体素+逐柱语义投影从未实测通过；launch 里的节点已移除，
#   障碍层统一走 costmap（STVL→costmap→plan_mask 桥）或 mask（z 带一刀切）。
#   源码保留在 `src/terrain_layer.cpp` 备查；要恢复就把下面几行取消注释即可：
#     add_executable(terrain_layer src/terrain_layer.cpp)
#     target_link_libraries(terrain_layer rclcpp::rclcpp ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_msgs_TARGETS} ${geometry_msgs_TARGETS}
#       tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs)
#     target_include_directories(terrain_layer PRIVATE ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
#     install(TARGETS terrain_layer RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES rm_tdt_planner_plugin.xml DESTINATION share/${PROJECT_NAME})
# ★★★ 2026-11-12：**离线的 T-DT 地图探针**（用 T-DT 自己的 YAstar 检查一张 PGM）。
#   用途：把"0.02 CAD 点云 → 栅格图"的产物直接喂给 T-DT，验证
#   ①取值约定（0=障碍）对不对 ②里面 A* 能不能走出来 ③路径形状是否合理。
if(BUILD_TESTING)
  add_executable(minisnap_points_probe test/minisnap_points_probe.cpp)
  target_include_directories(minisnap_points_probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/tdt)
  target_link_libraries(minisnap_points_probe tdt_core ${EIGEN3_INCLUDE_DIR})

  add_executable(tdt_map_probe test/tdt_map_probe.cpp)
  target_include_directories(tdt_map_probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/tdt)
  target_link_libraries(tdt_map_probe tdt_core ${EIGEN3_INCLUDE_DIR})
endif()

install(TARGETS tdt_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/tdt ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/mpc_proto/.deps/tdt/install/include ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/mpc_proto/.deps/tdt/install/include/osqp)
ament_export_libraries(tdt_core osqp OsqpEigen)
ament_package()
