cmake_minimum_required(VERSION 3.5)
project(turtlebot2_ros2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY 
  launch 
  config 
  maps
  rviz 
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/activate.sh scripts/auto_docking_client.py scripts/activation_button.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
