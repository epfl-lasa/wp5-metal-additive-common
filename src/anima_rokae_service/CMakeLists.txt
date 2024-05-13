cmake_minimum_required(VERSION 3.0.2)
project(anima_rokae_service)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_service_files(
  DIRECTORY srv
  FILES MAMControl.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

# catkin_install_python(
#   PROGRAMS
#   scripts/client.py
#   scripts/server.py
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/anima_rokae_service
# )
