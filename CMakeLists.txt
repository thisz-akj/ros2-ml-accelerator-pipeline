cmake_minimum_required(VERSION 3.8)
project(simaai_common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "simaai_msg/SimaaiBuffer.msg"
  "simaai_msg/SimaaiMessage.msg"
  "simaai_msg/Image.msg"
  DEPENDENCIES builtin_interfaces
)

# Memory header-only library
add_library(memory_interface INTERFACE)
target_include_directories(memory_interface INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/memory/include>
)
ament_target_dependencies(memory_interface INTERFACE rclcpp)

# Install headers
install(DIRECTORY memory/include/
  DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_include_directories(
  include
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()