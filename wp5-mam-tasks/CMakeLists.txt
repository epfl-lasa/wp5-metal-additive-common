cmake_minimum_required(VERSION 3.3)
project(wp5_mam_tasks VERSION 0.1.0 DESCRIPTION "Base task library decriving the task to be performed.")
include(${CMAKE_SOURCE_DIR}/../common_config.cmake)

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_TASKS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wp5_common_utilities
  wp5_mam_planner
  wp5_robotic_arms
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  wp5_common_utilities
  wp5_mam_planner
  wp5_robotic_arms
  DEPENDS YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Add the main node
set(node_name wp5_mam_main_task_node)
add_executable(${node_name}
  src/main_task.cpp
  src/ITaskBase.cpp
  src/TaskWelding.cpp
  src/TaskCleaning.cpp
  src/Subtask.cpp
  src/ROI.cpp
  src/WaypointParser.cpp
)

target_link_libraries(${node_name} PRIVATE
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# -------------------------
# Add the python scripts
# -------------------------
catkin_install_python(PROGRAMS
  scripts/publish_waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
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
