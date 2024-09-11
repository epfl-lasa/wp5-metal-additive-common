cmake_minimum_required(VERSION 3.3)
project(wp5_mam_planner VERSION 0.1.0 DESCRIPTION "Planner library to perform MAM tasks.")

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_MAM_PLANNER_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  wp5_ros_interface
  wp5_robotic_arms_drivers
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  wp5_ros_interface
  wp5_robotic_arms_drivers
  DEPENDS YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${NLopt_INCLUDE_DIRS}
)

# Create a library
add_library(${PROJECT_NAME}
  src/MAMPlanner.cpp
)

# Set lib version
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Add the main node
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/MAMPlanner.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${NLopt_LIBRARIES}
)