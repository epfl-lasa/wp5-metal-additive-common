# Robetarme WP5 Deliverable

## Overview

The repository comprises multiple packages written in python first, for development purpose. Currently, it is designed to work with ROS Noetic and MoveIt, but plans are underway to transition it to ROS2.

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

![Two main FSM](docs/240830_mam_deliverable_architecture.svg)

## Clone

This repository contains submodules :

```bash
git clone --recurse-submodules git@github.com:epfl-lasa/wp5-metal-additive.git
```

## Installation

We are currently using docker and its docker compose option. Please set it up before going forward.

The easy way is to run the setup.sh script which is in the root folder of this git repository, as follow:

```bash
bash setup.sh
```

It will setup everything using default values. All the environment variables needed in the docker are set up inside the .env file. You can modify them as you need.

To go more detailed, here are the commands automatically ran, with the previous scripts, to set up the system. It will set up your git folder, to avoid sending too big files online. Then the commands update the submodules and finally build, mount and allow you to access docker container with the last docker commands.

```bash
# --- Git Setup
# Copy pre-commit into hooks folder insside .git/hooks directory
cp "scripts/hooks/pre-commit" ".git/hooks/pre-commit"
chmod +x .git/hooks/pre-commit

# --- Submodules
# Initialize the submodules
git submodule update --init --recursive
git submodule update --recursive --remote
```

## Docker

### Build the Docker containers

The docker containers are using docker compose tools with their profiles keys. Each parameter is define inside the docker-compose.yml file and can mount multiple container from the following profile list :

- **nvidia** - ros1 noetic with the tools to run the controller **with** nvidia support
- **intel** - ros1 noetic with the tools to run the controller **without** nvidia support
- **coppeliasim** - coppeliasim simulator *need to have an access to EDU license for now*
- **e-series** - run a polyscope window to interact with e-series UR robots [ur5e, ur10e, ...]
- **cb-series** - run a polyscope window to interact with cb-series UR robots [ur5, ur10, ...]

### Examples

Generic commands, autocompletion is supported for all of these commands :

```bash
# Building specific docker profile
docker compose --profile <profile_name> build

# Mounting specific docker profile in detached mode
docker compose --profile <profile_name> up -d

# Mounting specific docker profile in detached mode and build it if needed
docker compose --profile <profile_name> up -d --build

# Mounting multiple docker profile container by appending them, in detached mode with building option
docker compose --profile <profile_name> --profile <profile_name> up -d --build

# Accessing docker in interactive mode from a shell
docker exec -it <docker_container_name> bash
```

To run the nvidia support :

```bash
# Building ros1 noetic docker with nvidia support
docker compose --profile nvidia build

# Mounting ros1 noetic docker with nvidia support in detached mode
docker compose --profile nvidia up -d

# Mounting ros1 noetic docker with nvidia support in detached mode and build it if needed
docker compose --profile nvidia up -d --build

# Accessing docker in interactive mode from a shell
docker exec -it wp5-metal-additive-ros-1 bash
```

### VSCode support

If using vscode with remote control extension, you can attach a vscode window to the chosen container using the bottom left green button.

### Troubleshooting

#### X Server issues

If no window showed up after setting up a docker image that should output one there may be an issue with the X server access. To allow docker using the X server, run the following command :

```bash
xhost +local:docker
```

### Optional : Coppeliasim Simulation

For those who needs and can use the university license of coppeliasim, the following commands can be used to add the coppeliasim docker as a submodule :

```bash
git submodule add -f git@github.com:epfl-lasa/docker_ros_coppeliasim.git tools/docker_ros_coppeliasim
git submodule update --init --recursive
```

The following command using docker compose with the right profile can run the simulation's coppeliasim window in detached mode.

```bash
docker compose --profile coppeliasim up -d --build
```

The optional *--build* argument will build the image if not already done.

## Credits

This repository use the work of the following repositories:

- IK-Geo - <https://github.com/rpiRobotics/ik-geo>
- ROS industrial - <https://github.com/ros-industrial/universal_robot>
- Universal Robot - <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>
- ROS modbus device driver - <https://github.com/epfl-lasa/ros-modbus-device-driver.git>

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Last Update - 2024-10-23
