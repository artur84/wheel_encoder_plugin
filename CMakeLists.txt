cmake_minimum_required(VERSION 3.8)
project(wheel_encoder_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(
  wheel_encoder_plugin SHARED
  src/wheel_encoder_plugin.cpp
)

ament_target_dependencies(
  wheel_encoder_plugin
  rclcpp
  std_msgs
)

target_link_libraries(
  wheel_encoder_plugin
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS wheel_encoder_plugin
  LIBRARY DESTINATION lib
)

ament_package()
