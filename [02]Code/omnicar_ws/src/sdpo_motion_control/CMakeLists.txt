cmake_minimum_required(VERSION 3.8)
project(sdpo_motion_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionState.msg"
  "msg/GoToPointDebug.msg"
  DEPENDENCIES std_msgs
)

add_executable(motion_state_adapter_node
  src/motion_state_adapter_node.cpp
)

add_executable(go_to_point_controller_node
  src/go_to_point_controller_node.cpp
)

ament_target_dependencies(motion_state_adapter_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(go_to_point_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motion_state_adapter_node "${cpp_typesupport_target}")
target_link_libraries(go_to_point_controller_node "${cpp_typesupport_target}")

install(TARGETS motion_state_adapter_node go_to_point_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
