cmake_minimum_required(VERSION 3.5)
project(serial_communication_channels)

# Standard C/C++
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trova Ament (Fondamentale per ROS 2 Jazzy)
find_package(ament_cmake REQUIRED)

# Definizione della Libreria
# Nota: controlla se il file è .c o .cpp nella tua cartella src/
add_library(${PROJECT_NAME} SHARED
  src/serial_communication_channels.c
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Regole di Installazione (Standard ROS 2)
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

# --- QUESTA È LA PARTE CHE MANCAVA E CHE RISOLVE L'ERRORE "NOT FOUND" ---

# Esporta le directory degli header per gli altri pacchetti (es. il tuo driver)
ament_export_include_directories(include)

# Esporta la libreria in modo che find_package(serial_communication_channels) funzioni
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Registra il pacchetto nel sistema Ament
ament_package()
