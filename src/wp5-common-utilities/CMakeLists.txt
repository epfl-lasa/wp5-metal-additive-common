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
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES math_tools yaml_tools convertion_tools
  CATKIN_DEPENDS roscpp
  DEPENDS YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# --- Math tools library
set(MATH_TOOLS_LIBRARY math_tools)

add_library(${MATH_TOOLS_LIBRARY}
  src/math_tools.cpp
)

target_link_libraries(${MATH_TOOLS_LIBRARY}
  ${catkin_LIBRARIES}
)

# --- YAML tools library
set(YAML_TOOLS_LIBRARY yaml_tools)

add_library(${YAML_TOOLS_LIBRARY}
  src/yaml_tools.cpp
)

target_link_libraries(${YAML_TOOLS_LIBRARY}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# --- Convertion tools library
set(CONVERTION_TOOLS_LIBRARY convertion_tools)

add_library(${CONVERTION_TOOLS_LIBRARY}
  src/convertion_tools.cpp
)

target_link_libraries(${CONVERTION_TOOLS_LIBRARY}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
