cmake_minimum_required(VERSION 3.8)
project(sdpo_ros_odom)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sdpo_drivers_interfaces REQUIRED)

function(_fix_sdpo_imported_location target_name library_name)
  if(TARGET ${target_name})
    get_target_property(_imported_location ${target_name} IMPORTED_LOCATION)
    get_target_property(_imported_location_noconfig ${target_name} IMPORTED_LOCATION_NOCONFIG)
    if((NOT _imported_location OR _imported_location STREQUAL "_imported_location-NOTFOUND") AND
       (NOT _imported_location_noconfig OR _imported_location_noconfig STREQUAL "_imported_location_noconfig-NOTFOUND"))
      set(_candidate "${sdpo_drivers_interfaces_DIR}/../../../lib/${library_name}")
      if(EXISTS "${_candidate}")
        set_target_properties(${target_name} PROPERTIES IMPORTED_LOCATION "${_candidate}")
      endif()
    endif()
  endif()
endfunction()

_fix_sdpo_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_generator_c libsdpo_drivers_interfaces__rosidl_generator_c.so)
_fix_sdpo_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_fastrtps_c libsdpo_drivers_interfaces__rosidl_typesupport_fastrtps_c.so)
_fix_sdpo_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_fastrtps_cpp libsdpo_drivers_interfaces__rosidl_typesupport_fastrtps_cpp.so)
_fix_sdpo_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_introspection_c libsdpo_drivers_interfaces__rosidl_typesupport_introspection_c.so)
_fix_sdpo_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_c libsdpo_drivers_interfaces__rosidl_typesupport_c.so)
_fix_sdpo_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_introspection_cpp libsdpo_drivers_interfaces__rosidl_typesupport_introspection_cpp.so)
_fix_sdpo_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_cpp libsdpo_drivers_interfaces__rosidl_typesupport_cpp.so)
_fix_sdpo_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_generator_py libsdpo_drivers_interfaces__rosidl_generator_py.so)

add_executable(${PROJECT_NAME}_cmd_vel_node
  src/cmd_vel_to_motors_ref_node.cpp
)

target_include_directories(${PROJECT_NAME}_cmd_vel_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_cmd_vel_node
  rclcpp
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_cmd_vel_node
  sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_cpp
)

install(TARGETS ${PROJECT_NAME}_cmd_vel_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
