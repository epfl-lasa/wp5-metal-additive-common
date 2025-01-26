# WP5 - Robotic Arms

Implement the different robotic arms class, to be able to use different robots based on the needs.

## Table of Contents

- [Overview](#overview)
- [Specialities](#specialities)
  - [RoboticArmFactory.h](#roboticarmfactoryh)
- [Architecture](#architecture)
- [Maintainers](#maintainers)

## Overview

This package implement the robotic arms classes as well as the robotic arm factory to be able to create new robotic arm based on their name.

## Specialities

### RoboticArmFactory.h

This part is implemented using the factory mechanism. The goal here is to be able to create a new robotic arm, only based on its name. To be able to add new possibilities to this factory, please add the robotic arm as a new entry in the list.

With the use of some C++ mechanisms such as the deleted constructor, the static feature and the inline functionnality, we are able to use this class without instantiating any object. Just calling the creatRoboticArm function with the correct *robotName*, we are capable of creating a new robotic arm corresponding to the passed name.

## Architecture

- **config -** yaml configuration files
- **include -** header files to be included
- **launch -** ROS launchfiles to run the different nodes
- **libs -** external libraries used in this package, currently only ikGeo is used
- **scripts -** python scripts, for now only a script to publish toy data, given in the config folder
- **src -** source files
- **test -** a test file to check if the waypoint parser is giving expected results, based on test data
- **tools -** these tools are complementar to the code implemented. It helps testing robotic arm forward and inverse kinematics based on toy data given in the config files
- **URDF -** description file for the robots, when needed

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Last Update - 2025-01-26
