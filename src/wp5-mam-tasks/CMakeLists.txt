cmake_minimum_required(VERSION 3.3)
project(wp5_mam_tasks VERSION 0.1.0 DESCRIPTION "Base task library decriving the task to be performed.")

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_TASKS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wp5_mam_planner
  wp5_robotic_arms_drivers
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  wp5_mam_planner
  wp5_robotic_arms_drivers
  DEPENDS YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# Add the main node
set(node_name wp5_mam_main_task_node)
add_executable(${node_name}
  src/main_task.cpp
  src/ITaskBase.cpp
  src/TaskWelding.cpp
  src/TaskCleaning.cpp
)

target_link_libraries(${node_name} PRIVATE
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
