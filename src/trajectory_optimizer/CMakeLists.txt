cmake_minimum_required(VERSION 3.8)
project(trajectory_optimizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(uav_mapping REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/BSplineTrajectory.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
ament_export_dependencies(
  rosidl_default_runtime
)

add_library(
  trajectory_optimizer_core

  src/trajectory_optimizer.cpp
)
target_include_directories(
  trajectory_optimizer_core

  PUBLIC

  $<BUILD_INTERFACE:
    ${CMAKE_CURRENT_SOURCE_DIR}/include>

  $<INSTALL_INTERFACE:
    include>
)
ament_target_dependencies(
  trajectory_optimizer_core

  uav_mapping
)
target_link_libraries(
  trajectory_optimizer_core

  Eigen3::Eigen

  OsqpEigen::OsqpEigen
)

add_executable(
  trajectory_optimizer_node

  src/trajectory_optimizer_node.cpp
)
target_include_directories(
  trajectory_optimizer_node

  PRIVATE

  include
)
ament_target_dependencies(
  trajectory_optimizer_node

  rclcpp

  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs

  uav_mapping
)
target_link_libraries(
  trajectory_optimizer_node

  trajectory_optimizer_core

  Eigen3::Eigen
)
# 当前package使用自己生成的msg
rosidl_get_typesupport_target(
  cpp_typesupport_target

  ${PROJECT_NAME}

  "rosidl_typesupport_cpp"
)
target_link_libraries(
  trajectory_optimizer_node

  "${cpp_typesupport_target}"
)
# ============================================================
# Install
# ============================================================
install(
  TARGETS

  trajectory_optimizer_core
  trajectory_optimizer_node

  DESTINATION
  lib/${PROJECT_NAME}
)
install(
  DIRECTORY

  include/

  DESTINATION
  include
)
install(
  DIRECTORY

  launch
  config

  DESTINATION
  share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
