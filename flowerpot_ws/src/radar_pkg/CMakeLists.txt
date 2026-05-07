cmake_minimum_required(VERSION 3.8)
project(radar_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radar_msgs REQUIRED)

add_executable(radar_node src/radar_node.cpp)

ament_target_dependencies(radar_node
  rclcpp
  sensor_msgs
  radar_msgs
)

install(TARGETS radar_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()