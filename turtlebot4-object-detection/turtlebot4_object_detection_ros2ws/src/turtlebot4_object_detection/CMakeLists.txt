cmake_minimum_required(VERSION 3.5)
project(turtlebot4_object_detection)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()