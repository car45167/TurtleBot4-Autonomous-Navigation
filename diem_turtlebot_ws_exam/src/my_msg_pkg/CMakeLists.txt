cmake_minimum_required(VERSION 3.8)
project(my_msg_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Trova dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Aggiunto
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Genera messaggi custom
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cone.msg"
  "msg/ConeArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Aggiungi queste righe per i file generati
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(std_msgs)

# Installa messaggi
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg)

# Installa i file generati (aggiunto)
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()