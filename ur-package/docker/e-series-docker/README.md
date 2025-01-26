# UR Polyscope - e-series

## Table of Contents

- [Overview](#overview)
- [Run a given program](#run-a-given-program)
- [Maintainers](#maintainers)

## Overview

This folder contains the docker to run the polyscopeto deal with e-series UR robotic arms.

## Run a given program

To go to the settings, click on the hamburger menu on the top right then settings:

![settings](doc/ur_e_settings.png)

Verify if urcaps is loaded, under settings -> urcaps. If not, click on the *+* button on the bottom left:

![urcaps](doc/ur_e_urcap.png)

Load the urcap file:

![load_urcap](doc/ur_e_load_file.png)

Restart the docker, as asked by polyscope:

![restart](doc/ur_e_restart.png)

When docker restarted, load the program to control the robot using an external source, ROS in our case:

![load_program](doc/ur_e_load_program.png)

![load_file](doc/ur_e_external_ros_file.png)

You can verify that everything goes well, under program tab. If the ip address of the robot is 127.0.0.1, *localhost*, then the control is good to go:

![program](doc/ur_e_program.png)

Now activate the robot, clicking on *power off* on the bottom left.

![power_on](doc/ur_e_power_on.png)

Then press the *ON* button.

![robot_on](doc/ur_e_on.png)

Finally press *START* button, then you can exit this page, pressing *Exit* button on the bottom left.

![robot_start](doc/ur_e_start.png)

At this step, everything is ready on the UR simulation side. As soon as the planner is started and ready to connect to the robot you can play the program, pressing play button on the bottom right and select *robot program*.

![play_program](doc/ur_e_robot_program.png)

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Last Update - 2025-01-26
