# Robetarme WP5 Deliverable

## Overview

The repository comprises multiple packages written in python first, for development purpose. Currently, it is designed to work with ROS Noetic and MoveIt.

There are two main FSM built for that project, as follow :

Error Handling FSM:

- **AllOk** This state is here to claim that the robot is safe to use and there are no errors.
- **ErrorMode**  Here an error occurs and needs to be acknowledged. It can be a safety error or simply an error in the execution of the task. After the error is acknowledged, it goes back to AllOk mode and the main FSM is resumed from where it was.

Main FSM:

- **Initializing** This is where every objects are initialized. It creates an object for the MoveIt planning scene, it initializes the known static obstacles and so on.
- **Planning** Here is the planning for the entire task. It will extract a path from the different waypoints and save it for later use.
- **Ready** As soon as the path is computed, the Robot enters in a Ready state where a Start signal is waiting to move to Executing state.
- **Executing** The path is executed with welding or cleaning execution when needed. If the robot is asked to exit, then it goes to Homing state otherwise it goes back to Planning.
- **Homing** The robot goes back to its initial position when the task is finished, it means it will just exit the FSM and be ready for another run when needed.
- **Exit** Here the task is finished, the FSM exited and all the unnecessary Cpp objects destroyed.

![Two main FSM](240830_mam_deliverable_architecture.svg)

## Requirements

This code is supposed to run on Linux, tested on both Ubuntu 20.04 and 22.04. It uses docker, side to docker compose, with the nvidia toolkit if any nvidia graphics card is installed. So you need to install :

- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [docker compose](https://docs.docker.com/compose/install/linux/#install-the-plugin-manually)

If you have an nvidia graphic card :

- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Docker

### Build the Docker containers

The docker containers are using docker compose tools. Each parameter is define inside the docker-compose.yml.

```bash
docker compose build
```

Then you can setting up the container, in detached mode, using :

```bash
docker compose up -d
```

And accessing it with a bash, in interactive mode, using :

```bash
docker exec -it <container_name> bash
```

## Build code

To build the ROS packages you can use :

```bash
catkin build
```

It is **HIGHLY** recommended to use the debug cmake flag when testing processes since it adds features like trajectory pre-vizualisation with user feedback before running them. To do so, run the following command :

```bash
catkin build -DCMAKE_BUILD_TYPE=Debug
```

## VSCode support

If using vscode with remote control extension, you can attach a vscode window to the chosen container using the bottom left green button.

## Troubleshooting

### X Server issues

If no window showed up after setting up a docker image that should output one there may be an issue with the X server access. To allow docker using the X server, run the following command :

```bash
xhost +local:docker
```

## License

This project includes software licensed under the Apache License 2.0 and the BSD 3-Clause License.
See the LICENSE file and [Credits](#credits) section for more details for the overall project and ur-package/LICENSE file for the UR licensing part.

## Credits

This repository use the work of the following repositories:

- [Moveit!](https://github.com/moveit/moveit) - *BSD-3-Clause*
- [IK-Geo-Cpp](https://github.com/Verdant-Evolution/ik-geo-cpp)
- [ROS industrial](https://github.com/ros-industrial/universal_robot)
- [Universal Robot Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) - *Apache 2.0*
- [ROS modbus device driver](https://github.com/epfl-lasa/ros-modbus-device-driver.git) - *MPL-2.0*

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Last Update - 2025-01-16
