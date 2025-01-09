cmake_minimum_required(VERSION 3.3)
project(wp5_ros_interface VERSION 0.1.0 DESCRIPTION "Ros interface library to bind with specific ros version")
include(${CMAKE_SOURCE_DIR}/../common_config.cmake)

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_ROS_INTERFACE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  wp5_common_utilities
)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    roslib
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
    wp5_common_utilities
  DEPENDS YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Create a library
add_library(${PROJECT_NAME}
  src/RosInterfaceNoetic.cpp
)

# Set lib version
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Link library to robotic_arms_drivers
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Link the shared library against required libraries
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# Add include and src to include path to make includes easier in the code
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  src
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

export(PACKAGE ${PROJECT_NAME})
