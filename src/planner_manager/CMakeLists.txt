cmake_minimum_required(VERSION 3.8)

project(planner_manager)


if(CMAKE_COMPILER_IS_GNUCXX OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )

endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(planner_manager_interfaces REQUIRED)
find_package(astar_planner REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(trajectory_optimizer REQUIRED)


add_executable(
  planner_manager_node

  src/planner_manager_node.cpp
)


ament_target_dependencies(
  planner_manager_node

  rclcpp
  rclcpp_action

  geometry_msgs

  planner_manager_interfaces
  astar_planner

  px4_msgs
  trajectory_optimizer
)


install(
  TARGETS

  planner_manager_node

  DESTINATION
  lib/${PROJECT_NAME}
)


install(
  DIRECTORY

  launch
  config

  DESTINATION
  share/${PROJECT_NAME}
)


ament_package()
