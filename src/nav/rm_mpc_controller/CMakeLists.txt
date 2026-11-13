cmake_minimum_required(VERSION 3.16)
project(rm_mpc_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- OSQP：默认用仓内解出来的 vendor 版本（不需要 sudo，见 tools/mpc_proto/build.sh）----
# ★ 2026-11-13：src 目录重排后（src/nav/rm_mpc_controller），到仓库根从 2 层变成 3 层 ⇒ 多退一级。
set(OSQP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/mpc_proto/.deps/opt/ros/humble" CACHE PATH "OSQP 安装前缀")
if(NOT EXISTS "${OSQP_ROOT}/lib/libosqp.so")
  message(FATAL_ERROR "找不到 ${OSQP_ROOT}/lib/libosqp.so；先跑 tools/mpc_proto/build.sh")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rm_plan_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/controller.cpp
  src/mpc_core/mpc_core.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${OSQP_ROOT}/include
  ${EIGEN3_INCLUDE_DIR}
  ${rm_plan_msgs_INCLUDE_DIRS}
  ${nav2_core_INCLUDE_DIRS} ${nav2_costmap_2d_INCLUDE_DIRS} ${nav2_util_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} ${rclcpp_lifecycle_INCLUDE_DIRS} ${pluginlib_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
  ${OSQP_ROOT}/lib/libosqp.so
  ${nav2_core_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  rm_plan_msgs::rm_plan_msgs__rosidl_typesupport_cpp
  rm_plan_msgs::rm_plan_msgs__rosidl_generator_cpp
  ${std_msgs_TARGETS})

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${OSQP_ROOT}/lib"
  INSTALL_RPATH "${OSQP_ROOT}/lib")

pluginlib_export_plugin_description_file(nav2_core rm_mpc_controller_plugin.xml)

# 插件加载冒烟测试（不经 controller_server）
add_executable(load_test test/load_test.cpp)
target_include_directories(load_test PRIVATE
  ${rm_plan_msgs_INCLUDE_DIRS}
  ${nav2_core_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${pluginlib_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS} ${rcl_lifecycle_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS})
target_link_libraries(load_test
  rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle pluginlib::pluginlib
  nav2_costmap_2d::nav2_costmap_2d_core ${nav2_util_TARGETS}
  ${nav2_core_TARGETS} ${nav2_costmap_2d_TARGETS} ${nav2_msgs_TARGETS}
  ${tf2_ros_TARGETS} ${tf2_TARGETS})
install(TARGETS load_test DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_libraries(${PROJECT_NAME})
ament_package()
