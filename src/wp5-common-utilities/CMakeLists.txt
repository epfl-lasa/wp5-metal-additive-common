cmake_minimum_required(VERSION 3.3)
project(wp5_common_utilities VERSION 0.1.0 DESCRIPTION "Common utilities for WP5")
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

# Find packages if any
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
