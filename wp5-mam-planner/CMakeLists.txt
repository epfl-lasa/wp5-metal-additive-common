cmake_minimum_required(VERSION 3.3)
project(wp5_mam_planner VERSION 0.1.0 DESCRIPTION "Planner library to perform MAM tasks.")
include(${CMAKE_SOURCE_DIR}/../common_config.cmake)

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_MAM_PLANNER_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  laser_service
  wp5_common_utilities
  wp5_ros_interface
  wp5_robotic_arms
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    laser_service
    wp5_common_utilities
    wp5_ros_interface
    wp5_robotic_arms
  DEPENDS YAML_CPP EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${NLopt_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Create a library
add_library(${PROJECT_NAME}
  src/IPlannerBase.cpp
  src/PlannerWelding.cpp
  src/PlannerCleaning.cpp
  src/ObstaclesManagement.cpp
)

# Set lib version
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Add the main node
add_library(${PROJECT_NAME}_node
  src/IPlannerBase.cpp
  src/PlannerWelding.cpp
  src/PlannerCleaning.cpp
  src/ObstaclesManagement.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${NLopt_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
