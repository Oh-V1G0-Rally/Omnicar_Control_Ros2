cmake_minimum_required(VERSION 3.8)
project(ldlidar_stl_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ldlidar_driver/include
  ldlidar_driver/include/core
  ldlidar_driver/include/dataprocess
  ldlidar_driver/include/filter
  ldlidar_driver/include/logger
  ldlidar_driver/include/networkcom
  ldlidar_driver/include/serialcom
)

set(SDK_SOURCES
  ldlidar_driver/src/core/ldlidar_driver.cpp
  ldlidar_driver/src/dataprocess/lipkg.cpp
  ldlidar_driver/src/filter/tofbf.cpp
  ldlidar_driver/src/logger/log_module.cpp
  ldlidar_driver/src/networkcom/network_socket_interface_linux.cpp
  ldlidar_driver/src/serialcom/serial_interface_linux.cpp
)

add_executable(${PROJECT_NAME}_node
  src/publish_node/main.cpp
  ${SDK_SOURCES}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}_node
  Threads::Threads
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz rules
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/create_udev_rules.sh
  scripts/delete_udev_rules.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
