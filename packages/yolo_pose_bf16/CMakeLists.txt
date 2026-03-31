cmake_minimum_required(VERSION 3.8)
project(yolo_pose_bf16)

set(CMAKE_CXX_STANDARD 20)
link_directories(/usr/lib/aarch64-linux-gnu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(ENV{PKG_CONFIG_PATH} /usr/lib/aarch64-linux-gnu/pkgconfig)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(simaai_common REQUIRED)
find_package(message_filters REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# Paths
set(NODES_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/src)
set(NODES_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/include)
set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common/include)
set(MEMORY_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common/memory/include)

# Shared node sources
set(SHARED_NODE_SOURCES
  ${NODES_SRC_DIR}/process_sima_yolopose.cpp
  ${NODES_SRC_DIR}/allegro_encoder.cpp
  ${NODES_SRC_DIR}/yolopose_render.cpp
)


# Combine
set(ALL_NODE_SOURCES ${SHARED_NODE_SOURCES})

# Build each as component
foreach(source ${ALL_NODE_SOURCES})
  get_filename_component(name ${source} NAME_WE)
  add_library(${name} SHARED ${source})
  target_include_directories(${name}
    PUBLIC ${NODES_INCLUDE_DIR}
    PUBLIC ${COMMON_INCLUDE_DIR}
    PUBLIC ${MEMORY_INCLUDE_DIR}
    PUBLIC ${JSONCPP_INCLUDE_DIRS}
    PUBLIC ${OpenCV_INCLUDE_DIRS}
    PUBLIC ${GSTREAMER_INCLUDE_DIRS}
    PUBLIC $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${name}
    simaaimem
    boxrender
    simaai_genboxdecode
    configManager
    simaaidispatcher
    simaai_cvhelpers
    ${OpenCV_LIBS}
    ${JSONCPP_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
  )
  ament_target_dependencies(${name}
    rclcpp rclcpp_components std_msgs sensor_msgs
    simaai_common message_filters
  )
  rclcpp_components_register_nodes(${name} "yolo_pose_bf16::${name}")
endforeach()

# Main executable
add_executable(yolo_pose_bf16_app src/yolo_pose.cpp)
target_link_libraries(yolo_pose_bf16_app
  simaaimem
  process_sima_yolopose 
  allegro_encoder
  yolopose_render
)
ament_target_dependencies(yolo_pose_bf16_app
  rclcpp rclcpp_components simaai_common 
)

# Install
install(TARGETS
  process_sima_yolopose 
  allegro_encoder
  yolopose_render
  LIBRARY DESTINATION lib
)

install(TARGETS yolo_pose_bf16_app DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY params launch DESTINATION share/${PROJECT_NAME})

ament_package()
