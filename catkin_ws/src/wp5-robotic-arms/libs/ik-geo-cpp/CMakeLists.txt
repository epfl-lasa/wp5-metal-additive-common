# Modified from https://gitlab.com/Fraser999/Rust-From-CXX/

project(ik_geo)
cmake_minimum_required(VERSION 3.0)

# Enable ExternalProject CMake module
include(ExternalProject)

# # Set default ExternalProject root directory
set_directory_properties(PROPERTIES EP_PREFIX ${CMAKE_BINARY_DIR}/Rust)

# Add rust_example as a CMake target
ExternalProject_Add(
    ik_geo_lib
    DOWNLOAD_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND cargo build COMMAND cargo build --release
    BINARY_DIR "${CMAKE_CURRENT_LIST_DIR}/rust-wrapper"
    INSTALL_COMMAND ""
    LOG_BUILD ON)

# Build to the dist folder
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_library(ik_geo INTERFACE)
add_dependencies(ik_geo ik_geo_lib)
target_include_directories(ik_geo INTERFACE include)

if (WIN32)
    target_link_libraries(ik_geo INTERFACE
        debug "${CMAKE_CURRENT_LIST_DIR}/rust-wrapper/target/debug/libik_geo.lib"
        optimized "${CMAKE_CURRENT_LIST_DIR}/rust-wrapper/target/release/libik_geo.lib"
        ws2_32 userenv advapi32)
else ()
    target_link_libraries(ik_geo INTERFACE
        debug "${CMAKE_CURRENT_LIST_DIR}/rust-wrapper/target/debug/libik_geo.so"
        optimized "${CMAKE_CURRENT_LIST_DIR}/rust-wrapper/target/release/libik_geo.so")
endif()

