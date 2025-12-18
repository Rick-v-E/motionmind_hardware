cmake_minimum_required(VERSION 3.5)
project(motionmind_hardware LANGUAGES CXX)

include(FetchContent)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FetchContent_Declare(
  serial_cpp
  GIT_REPOSITORY https://github.com/ami-iit/serial_cpp.git
  GIT_TAG v1.3.4
)

FetchContent_GetProperties(serial_cpp)
if(NOT serial_cpp_POPULATED)
  FetchContent_Populate(serial_cpp)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_subdirectory(${serial_cpp_SOURCE_DIR} ${serial_cpp_BINARY_DIR})
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/motionmind_hardware.cpp
  src/utils.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  rclcpp_lifecycle
  lifecycle_msgs
  pluginlib
  rclcpp
)

target_link_libraries(${PROJECT_NAME} serial_cpp)

pluginlib_export_plugin_description_file(hardware_interface motionmind_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS bin/calibrate_steering bin/check_single_device bin/set_pid_values
  DESTINATION lib/${PROJECT_NAME}
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
