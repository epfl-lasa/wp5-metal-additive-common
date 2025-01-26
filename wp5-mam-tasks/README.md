# WP5 - MAM Tasks

Implement the task manager with the specificities related to the different tasks asked.

## Table of Contents

- [Overview](#overview)
- [Specialities](#specialities)
  - [TaskFSM.h](#taskfsmh)
  - [TaskFactory.h](#taskfactoryh)
- [Architecture](#architecture)
- [Maintainers](#maintainers)

## Overview

This package implement the task manager. It includes a task Finite State Machine that will manage all the different steps to perform the task.

It also manages the waypoints parsing as well as a structure for the Region Of Interest, given by DTU.

## Specialities

### TaskFSM.h

The FSM is implemented using Boost::MSM library. The overall structure is done inside this header file. Then a FSM is instantiated inside the TaskManager to be enabled, disabled, paused and so on ... from the global TaskManager.

### TaskFactory.h

This part is implemented using the factory mechanism. The goal here is to be able to create a new task, only based on the name of the task. To be able to add new possibilities to this factory, please add the task as a new entry in the list.

With the use of some C++ mechanisms such as the deleted constructor, the static feature and the inline functionnality, we are able to use this class without instantiating any object. Just calling the creatTask function with the correct *taskName*, we are capable of creating a new task corresponding to the passed name.

## Architecture

- **config -** yaml configuration files
- **include -** header files to be included
- **launch -** ROS launchfiles to run the different nodes
- **scripts -** python scripts, for now only a script to publish toy data, given in the config folder
- **src -** source files
- **test -** a test file to check if the waypoint parser is giving expected results, based on test data

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Last Update - 2025-01-26
