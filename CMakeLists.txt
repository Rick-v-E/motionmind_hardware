cmake_minimum_required(VERSION 3.5)
project(motionmind_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
find_package(serial REQUIRED)
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
  serial
  pluginlib
  rclcpp
)

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
  serial
  pluginlib
  rclcpp
)

ament_package()
