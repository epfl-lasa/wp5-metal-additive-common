cmake_minimum_required(VERSION 3.3)
project(wp5_common_utilities VERSION 0.1.0 DESCRIPTION "Common utilities for WP5")
include(${CMAKE_SOURCE_DIR}/../common_config.cmake)

# Find packages if any
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    math_tools
    yaml_tools
    conversion_tools
    debug_tools
  CATKIN_DEPENDS
    roscpp
    std_msgs
    tf
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
  DEPENDS YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# --- Convertion tools library
add_library(conversion_tools
  src/conversion_tools.cpp
)

target_link_libraries(conversion_tools
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# --- Debug tools library
add_library(debug_tools
  src/debug_tools.cpp
)

target_link_libraries(debug_tools
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  conversion_tools
)

# --- Math tools library
add_library(math_tools
  src/math_tools.cpp
)

target_link_libraries(math_tools
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  conversion_tools
  debug_tools
)

# --- YAML tools library
set(YAML_TOOLS_LIBRARY yaml_tools)

add_library(yaml_tools
  src/yaml_tools.cpp
)

target_link_libraries(yaml_tools
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
