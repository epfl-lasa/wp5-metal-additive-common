# IK Geo C++

This is a wrapper for the Rust implementation for C++. The outpiut of this build is 2 files:

1. ik_geo.h
2. libik_geo.so/libik_geo.lib

## To build

```bash
mkdir build
cd build
cmake .. && make
```

## Including as a dependency

You can get a CMake object to link against by including this source as a sub_directory.

```cmake
# Include the ik_geo cmake as a subdirectory
add_subdirectory(.. IK_GEO)

add_executable(test src/main.cpp)

# Link against the ik_geo object produced by the ik_geo cmake
target_link_libraries(test PRIVATE ik_geo)
```

## To use

Include the header

```c++
#include "ik_geo.h"
```

Then you can create robots based off of kinematic specifications or select from hard coded ones

```c++
int main() {
    Robot robot = Robot::ur5();
    double rotation_matrix[9] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    double position_vector[3] = {0.0, 0.0, 0.0};

    std::vector<ik_geo::Solution> solutions;
    robot.ik(rotation_matrix, position_vector, solutions);
}
```

## To do

-   Test Windows implementation
-   Create a moveit package
