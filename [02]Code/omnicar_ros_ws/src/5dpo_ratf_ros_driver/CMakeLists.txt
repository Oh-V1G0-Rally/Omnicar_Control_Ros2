cmake_minimum_required(VERSION 3.0.2)
project(sdpo_ratf_ros_driver)

set(${PROJECT_NAME}_CATKIN_DEPENDENCIES
  roscpp
  message_generation
  message_runtime
  sdpo_ros_interfaces_hw
  sdpo_ros_serial_port
  serial_communication_channels
  std_msgs
  std_srvs
)

# file(GLOB_RECURSE MAIN_SOURCES RELATIVE ${PROJECT_SOURCE_DIR}
#   "src/sdpo_ratf_ros_driver/Robot5dpoRatf.cpp"
#   "src/sdpo_ratf_ros_driver/SdpoRatfROSDriver.cpp"
#   "src/sdpo_ratf_ros_driver/SerialChannelsConfig.cpp"
# )
# file(GLOB_RECURSE MAIN_HEADERS RELATIVE ${PROJECT_SOURCE_DIR}
#   "include/sdpo_ratf_ros_driver/Robot5dpoRatf.h"
#   "include/sdpo_ratf_ros_driver/SdpoRatfROSDriver.h"
#   "include/sdpo_ratf_ros_driver/SerialChannelsConfig.h"
# )

file(GLOB_RECURSE TUNE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR}
  "src/sdpo_ratf_ros_driver/Robot5dpoRatf.cpp"
  "src/sdpo_ratf_ros_driver/Robot5dpoRatfTune.cpp"
  "src/sdpo_ratf_ros_driver/SdpoRatfROSDriverTunning.cpp"
  "src/sdpo_ratf_ros_driver/SerialChannelsConfig.cpp"
)
file(GLOB_RECURSE TUNE_HEADERS RELATIVE ${PROJECT_SOURCE_DIR}
  "include/sdpo_ratf_ros_driver/Robot5dpoRatf.h"
  "include/sdpo_ratf_ros_driver/Robot5dpoRatfTune.h"
  "include/sdpo_ratf_ros_driver/SdpoRatfROSDriverTunning.h"
  "include/sdpo_ratf_ros_driver/SerialChannelsConfig.h"
)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPENDENCIES})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetMotorsPWM.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES sdpo_ratf_ros_driver
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPENDENCIES}
#   DEPENDS system_lib
)

###########
## Build ##
###########

## Includi i percorsi degli header
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 1. DEFINISCI LA LIBRERIA (Il cuore del driver)
## Questo assicura che Robot5dpoRatf.cpp e gli altri vengano compilati come oggetti riutilizzabili
add_library(${PROJECT_NAME}_lib
  src/sdpo_ratf_ros_driver/Robot5dpoRatf.cpp
  src/sdpo_ratf_ros_driver/SdpoRatfROSDriver.cpp
  src/sdpo_ratf_ros_driver/SerialChannelsConfig.cpp
)

## Aggiungi le dipendenze alla libreria (per messaggi e servizi generati)
add_dependencies(${PROJECT_NAME}_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## 2. DICHIARA L'ESEGUIBILE (Il nodo ROS)
## Il nodo principale contiene solo il main.cpp
add_executable(${PROJECT_NAME}_node src/main.cpp)

## 3. LINKA LE LIBRERIE (L'unione dei file)
## Diciamo al linker di unire il codice del main con la libreria appena creata
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

## Mantieni commentata la parte del tuning per ora
# add_executable(${PROJECT_NAME}_tune src/tune.cpp)
# target_link_libraries(${PROJECT_NAME}_tune ${PROJECT_NAME}_lib ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sdpo_ratf_ros_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
