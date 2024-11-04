cmake_minimum_required(VERSION 3.3)
project(wp5_mam_tasks VERSION 0.1.0 DESCRIPTION "Base task library decriving the task to be performed.")
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
add_definitions("-DWP5_TASKS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wp5_common_utilities
  wp5_mam_planner
  wp5_robotic_arms_drivers
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  wp5_common_utilities
  wp5_mam_planner
  wp5_robotic_arms_drivers
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
)

target_link_libraries(${node_name} PRIVATE
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
