#!/usr/bin/bash
# -*- coding: utf-8 -*-

# Script to help setup the environment inside docker with the python install and catkin build
# Author: lmunier - <lmunier@protonmail.com>
# Date: 2024-05-13

rosdep update
cd ~/catkin_ws/src

rosdep install -y --from-path ros-modbus-device-driver

cd ros-modbus-device-driver
pip3 install -r requirements.txt

cd ~/catkin_ws
catkin build
source devel/setup.bash
