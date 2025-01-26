# WP5 - MAM Planner

Implement the planner to compute and execute the path.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Credits](#credits)
- [Maintainers](#maintainers)

## Overview

This package implement the planner used in this project. It is based on Moveit! and ikGeo. Its specificity is that for each new path, it will first compute all the possible configurations for a robotic arm, at the main waypoint. Then it will test all of these configurations to find the one that will successed, if any.

The planner also take into account obstacles that are given as objects in the *config/obstacles.yaml* file and the body of the robot, with its tool, to perform self collision avoidance.

## Architecture

- **config -** yaml configuration files
- **include -** header files to be included
- **launch -** ROS launchfiles to run the different nodes
- **src -** source files

## Credits

This repository use the work of the following repositories:

- [Moveit!](https://github.com/moveit/moveit) - *BSD-3-Clause*
- [IK-Geo-Cpp](https://github.com/Verdant-Evolution/ik-geo-cpp)

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Last Update - 2025-01-26
