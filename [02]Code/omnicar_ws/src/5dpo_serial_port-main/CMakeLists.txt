cmake_minimum_required(VERSION 3.5)
project(sdpo_serial_port)

# Standard C++
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trova le dipendenze
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# Definizione della Libreria
add_library(${PROJECT_NAME} SHARED
  src/AsyncSerial.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link alle librerie Boost
target_link_libraries(${PROJECT_NAME} PUBLIC
  Boost::system
  Boost::thread
)

# Regole di Installazione
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- CONFIGURAZIONE AMENT PER ROS 2 JAZZY ---
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
# Esporta le directory degli header per gli altri pacchetti
ament_export_include_directories(include)

# Esporta la libreria per permettere find_package() agli altri nodi
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Registra le dipendenze esterne (Boost)
ament_export_dependencies(Boost)

# Genera i file di setup e registra il pacchetto
ament_package()
