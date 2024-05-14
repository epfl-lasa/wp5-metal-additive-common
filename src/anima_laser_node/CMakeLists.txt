cmake_minimum_required(VERSION 3.0.2)
project(anima_laser_node)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS yaml std_msgs
)

catkin_install_python(
  PROGRAMS
  scripts/laser_anima.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/anima_rokae_service
)
