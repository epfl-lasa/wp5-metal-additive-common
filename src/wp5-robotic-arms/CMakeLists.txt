cmake_minimum_required(VERSION 3.3)
project(wp5_robotic_arms VERSION 0.1.0 DESCRIPTION "Drivers library to manage different robotic arms")
set(CMAKE_CXX_STANDARD 17)

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Define a macro for the build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG_MODE)
endif()

# Set compile options for different build types
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Apply the compile options based on the build type
add_compile_options(${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}})

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_ROBOTIC_ARMS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

# Add the external library
add_subdirectory(libs/ik-geo-cpp)

# Find packages if any
find_package(yaml-cpp REQUIRED)
find_package(trac_ik_lib REQUIRED COMPONENTS)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  trac_ik_lib
  wp5_common_utilities
  wp5_ros_interface
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include libs/ik-geo-cpp/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    trac_ik_lib
    wp5_common_utilities
    wp5_ros_interface
  DEPENDS YAML_CPP EIGEN3
)

include_directories(
  include
  urdf
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${trac_ik_lib_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Create library
add_library(${PROJECT_NAME}
    src/IRoboticArmBase.cpp
    src/RoboticArmUr.cpp
    src/RoboticArmFactory.cpp
)

# Set lib version
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Install include files
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Add include and src to include path to make includes easier in the code
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  urdf
  src
  ${YAML_CPP_INCLUDE_DIRS}
  ${trac_ik_lib_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Link the INTERFACE library to the STATIC library
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${trac_ik_lib_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ik_geo
)

# Link library to robotic_arms_drivers
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

export(PACKAGE ${PROJECT_NAME})

# -------------------------
# Compile perform_fk tool
# -------------------------
add_executable(perform_fk
  tools/perform_fk.cpp
  src/IRoboticArmBase.cpp
  src/RoboticArmUr.cpp
  src/RoboticArmFactory.cpp
)

target_link_libraries(perform_fk
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${trac_ik_lib_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ik_geo
)

# -------------------------
# Compile perform_fk tool
# -------------------------
add_executable(perform_ik
  tools/perform_ik.cpp
  src/IRoboticArmBase.cpp
  src/RoboticArmUr.cpp
  src/RoboticArmFactory.cpp
)

target_link_libraries(perform_ik
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${trac_ik_lib_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ik_geo
)

# -------------------------
# Add the test executable
# -------------------------
set(test_exe_name test_robotic_arm)

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
    test/test_robotic_arm.cpp
    src/IRoboticArmBase.cpp
    src/RoboticArmUr.cpp
    src/RoboticArmFactory.cpp
  )

  target_link_libraries(${test_exe_name}
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ik_geo
    pthread
  )
endif()