cmake_minimum_required(VERSION 3.3)
project(wp5_robotic_arms_drivers VERSION 0.1.0 DESCRIPTION "Drivers library to manage different robotic arms")

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_ROBOTIC_ARMS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

# Add the external library
add_subdirectory(libs/ik-geo-cpp)

# Find packages if any
find_package(catkin REQUIRED COMPONENTS
  roscpp
  trac_ik_lib
)
find_package(yaml-cpp REQUIRED)
find_package(trac_ik_lib REQUIRED COMPONENTS)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include libs/ik-geo-cpp/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp trac_ik_lib
  DEPENDS YAML_CPP
)

include_directories(
  include
  urdf
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${trac_ik_lib_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/IRoboticArmBase.cpp
    src/RoboticArmCr7.cpp
    src/RoboticArmUr5.cpp
)


# Set lib version
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Add include and src to include path to make includes easier in the code
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  urdf
  src
  ${YAML_CPP_INCLUDE_DIR}
  ${trac_ik_lib_INCLUDE_DIRS}
)

# Link the INTERFACE library to the STATIC library
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${trac_ik_lib_LIBRARIES}
  ik_geo
)

# Link library to robotic_arms_drivers
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

export(PACKAGE ${PROJECT_NAME})