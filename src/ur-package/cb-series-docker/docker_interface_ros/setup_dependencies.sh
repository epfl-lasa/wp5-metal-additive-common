#!/bin/bash

cd src
git clone  --branch feature/dockerise https://github.com/epfl-lasa/iiwa_ros.git
git clone https://bitbucket.org/traclabs/trac_ik.git
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
cd send_pos
mkdir Third_party
cd Third_party
git clone https://github.com/coin-or/qpOASES.git
#mkdir build
#cd build
#cmake ..
#make
#sudo make install
cd ../../..

catkin_make
source devel/setup.bash
read terminate
