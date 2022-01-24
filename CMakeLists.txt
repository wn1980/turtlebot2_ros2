cmake_minimum_required(VERSION 3.5)
project(turtlebot2_ros2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY 
  launch 
  config 
  maps
  rviz 
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/activate.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
