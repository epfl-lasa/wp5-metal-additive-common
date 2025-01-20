cmake_minimum_required(VERSION 3.21)
project(test)

set(CMAKE_CXX_STANDARD 17)

# Include the ik_geo cmake as a subdirectory
add_subdirectory(.. IK_GEO)

add_executable(test src/main.cpp)

# Link against the ik_geo object produced by the ik_geo cmake
target_link_libraries(test PRIVATE ik_geo)