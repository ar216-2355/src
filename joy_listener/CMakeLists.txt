cmake_minimum_required(VERSION 3.5)
project(joy_listener)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joy_subscriber src/joy_subscriber.cpp)
ament_target_dependencies(joy_subscriber rclcpp sensor_msgs)

install(TARGETS joy_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
