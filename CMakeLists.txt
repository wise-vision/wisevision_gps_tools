cmake_minimum_required(VERSION 3.8)
project(wisevision_gps_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
    jwt-cpp 
    GIT_REPOSITORY https://github.com/Thalhammer/jwt-cpp.git
    GIT_TAG 08bcf77a687fb06e34138e9e9fa12a4ecbe12332 # v0.7.0 release
)
set(JWT_BUILD_EXAMPLES OFF CACHE BOOL "disable building examples" FORCE)
FetchContent_MakeAvailable(jwt-cpp)

# Fetching nlohmann/json library
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.2
)
FetchContent_MakeAvailable(nlohmann_json)

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

include_directories(include)
include_directories(${CMAKE_BINARY_DIR})

target_link_libraries(gps_device_manager_node
  yaml-cpp
  nlohmann_json::nlohmann_json
  jwt-cpp::jwt-cpp
)

install(TARGETS
  gps_device_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()