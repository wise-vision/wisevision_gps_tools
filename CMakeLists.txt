cmake_minimum_required(VERSION 3.8)
project(wisevision_gps_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
${YAML_CPP_INCLUDEDIR}
include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wisevision_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(gps_device_manager_node src/main.cpp src/GpsDeviceManager.cpp)

ament_target_dependencies(gps_device_manager_node
  rclcpp
  wisevision_msgs
  sensor_msgs
  yaml-cpp
)

target_link_libraries(gps_device_manager_node yaml-cpp)

install(TARGETS
  gps_device_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
