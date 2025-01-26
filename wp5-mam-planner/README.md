# WP5 - Common Utilities

Implement the planner to compute and execute the path.

## Table of Contents

- [Overview](#overview)
- [Getting Started](#getting-started)
- [Architecture](#architecture)
- [Maintainers](#maintainers)

## Overview

This package implement the planner used in this project. It is based on Moveit! and ikGeo. Its specificity is that for each new path, it will first compute all the possible configurations for a robotic arm, at the main waypoint. Then it will test all of these configurations to find the one that will successed, if any.

## Architecture

- **config -** yaml configuration files
- **include -** header files to be included
- **launch -** ros launchfiles to run the different nodes
- **src -** source files

## Credits

This repository use the work of the following repositories:

- [Moveit!](https://github.com/moveit/moveit) - *BSD-3-Clause*
- [IK-Geo-Cpp](https://github.com/Verdant-Evolution/ik-geo-cpp)

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Last Update - 2025-01-26
