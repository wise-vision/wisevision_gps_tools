cmake_minimum_required(VERSION 3.8)
project(wisevision_gps_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

# Fetch jwt-cpp library
FetchContent_Declare(
    jwt-cpp 
    GIT_REPOSITORY https://github.com/Thalhammer/jwt-cpp.git
    GIT_TAG 08bcf77a687fb06e34138e9e9fa12a4ecbe12332 # v0.7.0 release
)
set(JWT_BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples" FORCE)
FetchContent_MakeAvailable(jwt-cpp)

# Fetch nlohmann/json library
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.2
)
FetchContent_MakeAvailable(nlohmann_json)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wisevision_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Create a library for shared code
add_library(gps_device_manager_lib
  src/GpsDeviceManager.cpp
)

target_include_directories(gps_device_manager_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gps_device_manager_lib
  rclcpp
  wisevision_msgs
  sensor_msgs
  yaml-cpp
)

target_link_libraries(gps_device_manager_lib
  yaml-cpp
  nlohmann_json::nlohmann_json
  jwt-cpp::jwt-cpp
)

# Create the executable
add_executable(gps_device_manager_node src/main.cpp)

target_link_libraries(gps_device_manager_node
  gps_device_manager_lib
)

ament_target_dependencies(gps_device_manager_node
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(gps_device_manager_test test/gps_device_manager_test.cpp)

  target_include_directories(gps_device_manager_test PRIVATE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(gps_device_manager_test
    gps_device_manager_lib
  )

  ament_target_dependencies(gps_device_manager_test 
    rclcpp
    sensor_msgs
    wisevision_msgs
  )
endif()

install(TARGETS
  gps_device_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()