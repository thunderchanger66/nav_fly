cmake_minimum_required(VERSION 3.8)
project(astar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(uav_mapping REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}

  "srv/PlanPath.srv"

  DEPENDENCIES

  geometry_msgs
  nav_msgs
)
ament_export_dependencies(
  rosidl_default_runtime
)

add_library(
  astar_core
  src/astar_3d.cpp
)
target_include_directories(
  astar_core
  PUBLIC
  $<BUILD_INTERFACE:
    ${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:
    include>
)
ament_target_dependencies(
  astar_core
  uav_mapping
)

add_executable(
  astar_node
  src/astar_node.cpp
)
target_include_directories(
  astar_node
  PRIVATE
  include
)
ament_target_dependencies(
  astar_node
  rclcpp
  geometry_msgs
  nav_msgs
  uav_mapping
)
target_link_libraries(
  astar_node
  astar_core
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(
  astar_node
  "${cpp_typesupport_target}"
)

install(
  TARGETS
  astar_core
  astar_node
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
