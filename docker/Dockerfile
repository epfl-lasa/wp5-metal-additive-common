# Handle ros distro
ARG ROS_DISTRO=noetic

FROM ros:${ROS_DISTRO} as ros-ws

ARG ROS_DISTRO
ARG USER=root
ARG HOME=/home/${USER}

ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES video,compute,utility
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

# create and configure a new user
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y sudo
RUN useradd -m ${USER} && echo "${USER}:${USER}passwd" | chpasswd && adduser ${USER} sudo

### Add a few essential tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    git \
    openssh-client \
    python3 \
    python3-pip \
    net-tools \
    build-essential \
    cmake

RUN pip3 install --upgrade pip

# Install catkin tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y wget
RUN sh -c \
    'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
    > /etc/apt/sources.list.d/ros-latest.list'

RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    python3-catkin-tools \
    python3-osrf-pycommon

FROM ros-ws as libraries

# Install waypoints tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    libcgal-dev

RUN pip3 install \
    opencv-python

# Install moveit tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    python3-vcstool \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-commander \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-pcl-ros

# Install trac ik
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-${ROS_DISTRO}-trac-ik \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    ros-${ROS_DISTRO}-diagnostic-updater

# Install ANiMA's dependencies
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-${ROS_DISTRO}-catkin-virtualenv

WORKDIR ${HOME}/catkin_ws/src
RUN chown -R ${USER}:${USER} ${HOME}/catkin_ws

FROM libraries as cmake-options

# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> ${HOME}/.bashrc ; fi

FROM cmake-options as simulation-tools

# Install gazebo
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control

FROM simulation-tools as finalisation

# Give bashrc back to user
WORKDIR ${HOME}
RUN chown -R ${USER}:${HOST_GID} .bashrc

# Build the workspace and source the devel/setup.bash
USER ${USER}
WORKDIR ${HOME}/catkin_ws
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin build"
RUN echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc

