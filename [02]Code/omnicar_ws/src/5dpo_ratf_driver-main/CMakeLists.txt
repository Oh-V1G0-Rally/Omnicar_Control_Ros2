cmake_minimum_required(VERSION 3.5)
project(sdpo_ratf_driver)



# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 2 <<< #####
###################################################

if("$ENV{ROS_VERSION}" STREQUAL "2")

  # Dependencies
  find_package(ament_cmake REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system thread)
  find_package(rclcpp REQUIRED)
  find_package(sdpo_drivers_interfaces REQUIRED)
  find_package(sdpo_serial_port REQUIRED)
  find_package(serial_communication_channels REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  function(_fix_imported_location target_name package_dir library_name)
    if(TARGET ${target_name})
      get_target_property(_imported_location ${target_name} IMPORTED_LOCATION)
      get_target_property(_imported_location_noconfig ${target_name} IMPORTED_LOCATION_NOCONFIG)
      if((NOT _imported_location OR _imported_location STREQUAL "_imported_location-NOTFOUND") AND
         (NOT _imported_location_noconfig OR _imported_location_noconfig STREQUAL "_imported_location_noconfig-NOTFOUND"))
        set(_candidate "${package_dir}/../../../lib/${library_name}")
        if(EXISTS "${_candidate}")
          set_target_properties(${target_name} PROPERTIES IMPORTED_LOCATION "${_candidate}")
        endif()
      endif()
    endif()
  endfunction()

  _fix_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_generator_c ${sdpo_drivers_interfaces_DIR} libsdpo_drivers_interfaces__rosidl_generator_c.so)
  _fix_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_fastrtps_c ${sdpo_drivers_interfaces_DIR} libsdpo_drivers_interfaces__rosidl_typesupport_fastrtps_c.so)
  _fix_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_fastrtps_cpp ${sdpo_drivers_interfaces_DIR} libsdpo_drivers_interfaces__rosidl_typesupport_fastrtps_cpp.so)
  _fix_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_introspection_c ${sdpo_drivers_interfaces_DIR} libsdpo_drivers_interfaces__rosidl_typesupport_introspection_c.so)
  _fix_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_c ${sdpo_drivers_interfaces_DIR} libsdpo_drivers_interfaces__rosidl_typesupport_c.so)
  _fix_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_introspection_cpp ${sdpo_drivers_interfaces_DIR} libsdpo_drivers_interfaces__rosidl_typesupport_introspection_cpp.so)
  _fix_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_cpp ${sdpo_drivers_interfaces_DIR} libsdpo_drivers_interfaces__rosidl_typesupport_cpp.so)
  _fix_imported_location(sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_generator_py ${sdpo_drivers_interfaces_DIR} libsdpo_drivers_interfaces__rosidl_generator_py.so)
  _fix_imported_location(sdpo_serial_port::sdpo_serial_port ${sdpo_serial_port_DIR} libsdpo_serial_port.so)
  _fix_imported_location(serial_communication_channels::serial_communication_channels ${serial_communication_channels_DIR} libserial_communication_channels.so)

  # Node Executable
  add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}/Robot5dpoRatf.cpp
    src/${PROJECT_NAME}/SerialChannelsConfig.cpp
    src/${PROJECT_NAME}/SdpoRatfDriverROS2.cpp
    src/main_ros2.cpp
  )

  add_executable(${PROJECT_NAME}_tune
    src/${PROJECT_NAME}/Robot5dpoRatf.cpp
    src/${PROJECT_NAME}/Robot5dpoRatfTune.cpp
    src/${PROJECT_NAME}/SerialChannelsConfig.cpp
    src/${PROJECT_NAME}/SdpoRatfTuneDriverROS2.cpp
    src/tune_ros2.cpp
  )

  target_include_directories(${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${sdpo_serial_port_DIR}/../../../include
    ${serial_communication_channels_DIR}/../../../include
  )

  target_include_directories(${PROJECT_NAME}_tune PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${sdpo_serial_port_DIR}/../../../include
    ${serial_communication_channels_DIR}/../../../include
  )

  ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    sdpo_serial_port
    serial_communication_channels
    std_msgs
    std_srvs
  )

  ament_target_dependencies(${PROJECT_NAME}_tune
    rclcpp
    sdpo_serial_port
    serial_communication_channels
    std_msgs
    std_srvs
  )

  target_link_libraries(${PROJECT_NAME}_node
    sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_cpp
    sdpo_serial_port::sdpo_serial_port
    serial_communication_channels::serial_communication_channels
  )
  target_link_libraries(${PROJECT_NAME}_tune
    sdpo_drivers_interfaces::sdpo_drivers_interfaces__rosidl_typesupport_cpp
    sdpo_serial_port::sdpo_serial_port
    serial_communication_channels::serial_communication_channels
  )

  # Node Executable Installation Rules
  install(
    TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_tune
    DESTINATION lib/${PROJECT_NAME}
  )

  # Launch Files Installation Rules
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  # Testing
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()



###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 1 <<< #####
###################################################

elseif("$ENV{ROS_VERSION}" STREQUAL "1")

  message(AUTHOR_WARNING "Project version in ROS 1 may be deprecated")

  # Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sdpo_drivers_interfaces
    sdpo_serial_port
    serial_communication_channels
    std_msgs
    std_srvs
  )

  # Catkin specific configuration
  catkin_package(
    INCLUDE_DIRS include
    # LIBRARIES sdpo_drivers_interfaces
    CATKIN_DEPENDS
      roscpp
      sdpo_drivers_interfaces
      sdpo_serial_port
      serial_communication_channels
      std_msgs
      std_srvs
    # DEPENDS system_lib
  )

  # Build
  # - additional header files
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  # - declare a C++ executable
  add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}/Robot5dpoRatf.cpp
    src/${PROJECT_NAME}/SerialChannelsConfig.cpp
    src/${PROJECT_NAME}/SdpoRatfDriverROS1.cpp
    src/main_ros1.cpp
  )

  add_executable(${PROJECT_NAME}_tune
    src/${PROJECT_NAME}/Robot5dpoRatf.cpp
    src/${PROJECT_NAME}/Robot5dpoRatfTune.cpp
    src/${PROJECT_NAME}/SerialChannelsConfig.cpp
    src/${PROJECT_NAME}/SdpoRatfTuneDriverROS1.cpp
    src/tune_ros1.cpp
  )

  # - target dependencies
  add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_dependencies(${PROJECT_NAME}_tune
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  # - specify linking libraries
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
  )

  target_link_libraries(${PROJECT_NAME}_tune
    ${catkin_LIBRARIES}
  )

  # - installation rules
  install(
    TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_tune
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )

  install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )


##########################
#####  !!! ERROR !!! #####
##########################

else()

  message(FATAL_ERROR "No valid version of ROS is installed or environment variables not set")

endif()
