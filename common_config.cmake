# @file common_config.cmake
# @brief Here is stored the common cmake configuration for the project.
# This file is included in all the CMakeLists.txt files of the project.

# @author [Louis Munier] - lmunier@protonmail.com
# @version 0.1
# @date 2025-01-28

# @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

# Set default build type to Release if not specified
if(DEFINED ENV{CMAKE_BUILD_TYPE})
    set(CMAKE_BUILD_TYPE $ENV{CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build." FORCE)
else()
    # Set default build type to Release if not specified
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    endif()
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
