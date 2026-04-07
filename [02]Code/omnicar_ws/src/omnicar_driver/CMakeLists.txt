cmake_minimum_required(VERSION 3.8)
project(omnicar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Trova le dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(sdpo_drivers_interfaces REQUIRED)
find_package(sdpo_serial_port REQUIRED)
find_package(serial_communication_channels REQUIRED)

# 2. Crea l'eseguibile
add_executable(omnicar_driver_node src/omnicar_driver_node.cpp)

rosidl_target_interfaces(omnicar_driver_node
  sdpo_drivers_interfaces "rosidl_typesupport_cpp")
  
# 3. Aggiungi le dipendenze al target
ament_target_dependencies(omnicar_driver_node
  rclcpp
  sdpo_drivers_interfaces
  sdpo_serial_port
  serial_communication_channels
)

# 4. Installazione
install(TARGETS omnicar_driver_node
  DESTINATION lib/${PROJECT_NAME})

# 5. Configurazione Ament (QUESTO DEVE ESSERE ALLA FINE)
ament_package()
