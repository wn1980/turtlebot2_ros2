cmake_minimum_required(VERSION 3.5)
project(turtlebot2_ros2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
