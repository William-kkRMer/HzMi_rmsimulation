cmake_minimum_required(VERSION 3.16)
project(rm_tdt_local_controller)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_nav_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rm_tdt_planner REQUIRED)
find_package(rm_plan_msgs REQUIRED)
set(TDT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../rm_tdt_planner/src/tdt")
set(OSQP_TDT "${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/mpc_proto/.deps/tdt/install")
find_library(TDT_CORE_LIB NAMES tdt_core PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../../install/rm_tdt_planner/lib" NO_DEFAULT_PATH)

add_library(${PROJECT_NAME} SHARED src/controller.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${nav2_core_INCLUDE_DIRS} ${nav2_costmap_2d_INCLUDE_DIRS} ${nav2_util_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} ${rclcpp_lifecycle_INCLUDE_DIRS} ${rcl_lifecycle_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${TDT_DIR}
  ${OSQP_TDT}/include ${OSQP_TDT}/include/osqp ${rm_plan_msgs_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${TDT_CORE_LIB}
  ${OSQP_TDT}/lib/libosqp.so ${OSQP_TDT}/lib/libOsqpEigen.so
  ${nav2_core_TARGETS} ${nav2_costmap_2d_TARGETS} ${nav2_util_TARGETS} ${nav2_msgs_TARGETS}
  rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle
  nav2_costmap_2d::nav2_costmap_2d_core
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_msgs_TARGETS}
  pluginlib::pluginlib
  rm_plan_msgs::rm_plan_msgs__rosidl_typesupport_cpp
  rm_plan_msgs::rm_plan_msgs__rosidl_generator_cpp
  rm_nav_interfaces::rm_nav_interfaces__rosidl_typesupport_cpp
  rm_nav_interfaces::rm_nav_interfaces__rosidl_generator_cpp)

pluginlib_export_plugin_description_file(nav2_core rm_tdt_local_controller_plugin.xml)
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES rm_tdt_local_controller_plugin.xml DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rm_tdt_planner)
ament_package()
