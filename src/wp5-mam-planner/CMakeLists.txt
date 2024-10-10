cmake_minimum_required(VERSION 3.3)
project(wp5_mam_planner VERSION 0.1.0 DESCRIPTION "Planner library to perform MAM tasks.")
set(CMAKE_CXX_STANDARD 17)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Set compile options for different build types
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Apply the compile options based on the build type
add_compile_options(${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}})

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_MAM_PLANNER_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  wp5_common_utilities
  wp5_ros_interface
  wp5_robotic_arms_drivers
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

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
  DEPENDS YAML_CPP EIGEN3 GTest
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
  src/MAMPlanner.cpp
  src/Subtask.cpp
  src/WaypointParser.cpp
)

# Set lib version
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Add the main node
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/MAMPlanner.cpp
  src/Subtask.cpp
  src/WaypointParser.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${NLopt_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# -------------------------
# Add the test executable
# -------------------------
set(test_exe_name test_waypoint_parser)

# Add gtest dependency
if (CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  include_directories(
    ${GTEST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
  )

  # Add the test executable only once
  add_executable(${test_exe_name}
    test/test_waypoint_parser.cpp
    src/WaypointParser.cpp
  )
  target_link_libraries(${test_exe_name}
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    pthread
  )
endif()