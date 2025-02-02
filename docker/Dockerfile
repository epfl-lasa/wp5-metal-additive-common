# Handle ros distro
ARG ROS_DISTRO=noetic

FROM osrf/ros:${ROS_DISTRO}-desktop-full-focal AS ros-ws

ARG ROS_DISTRO
ARG USER=root
ARG HOME=/home/${USER}

ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES video,compute,utility
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

# Create and configure a new user
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y sudo
RUN useradd -m ${USER} && echo "${USER}:${USER}passwd" | chpasswd && adduser ${USER} sudo

# Add a few essential tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    git \
    openssh-client \
    python3 \
    python3-pip \
    net-tools \
    build-essential \
    cmake \
    gdb \
    valgrind

RUN pip3 install --upgrade pip

# Install catkin tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    python3-catkin-tools \
    python3-osrf-pycommon

WORKDIR ${HOME}/catkin_ws/src
RUN chown -R ${USER}:${USER} ${HOME}/catkin_ws

FROM ros-ws AS libraries

# Install moveit tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    python3-vcstool \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-commander \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-chomp-motion-planner \
    ros-${ROS_DISTRO}-moveit-chomp-optimizer-adapter \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-pcl-ros

# Install trac ik
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-${ROS_DISTRO}-trac-ik \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    ros-${ROS_DISTRO}-diagnostic-updater

# Install ik_geo for python use
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN pip install ik-geo

# Install NLopt library
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    libnlopt-dev \
    libnlopt-cxx-dev

# Install ANiMA's dependencies
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-${ROS_DISTRO}-catkin-virtualenv \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-pilz-industrial-motion-planner \
    ros-${ROS_DISTRO}-industrial-core \
    ros-${ROS_DISTRO}-industrial-robot-status-interface

WORKDIR ${HOME}/catkin_ws/src
RUN chown -R ${USER}:${USER} ${HOME}/catkin_ws

# Install UR dependencies
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-${ROS_DISTRO}-ur-msgs

FROM libraries AS cmake-options

# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> ${HOME}/.bashrc ; fi

FROM cmake-options AS simulation-tools

# Install gazebo
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control

# Install realsense camera dependencies
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description

# Install Rust
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    curl

USER ${USER}
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN echo "source ${HOME}/.cargo/env" >> ~/.bashrc

ENV PATH="${HOME}/.cargo/bin:${PATH}"
ENV CARGO_HOME="${HOME}/.cargo"

FROM simulation-tools AS finalisation

# Give bashrc back to user
WORKDIR ${HOME}
RUN chown -R ${USER}:${HOST_GID} .bashrc

# Build the workspace and source the devel/setup.bash
WORKDIR ${HOME}/catkin_ws
COPY .. ${HOME}/catkin_ws/src/
RUN rosdep update

USER root
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN rosdep install --from-paths ${HOME}/catkin_ws/src --ignore-src -r -y

# Install modbus python dependencies
WORKDIR ${HOME}/catkin_ws/src/ros-modbus-device-driver
RUN pip3 install -r requirements.txt

# Ensure correct ownership of the workspace
RUN chown -R ${USER}:${USER} ${HOME}/catkin_ws

# ------------------------------------------------------------------------------
# catkin build ROS noetic packages
USER ${USER}
RUN source ${HOME}/.bashrc                                                  && \
    source /opt/ros/noetic/setup.bash                                       && \
    catkin build
# ------------------------------------------------------------------------------

# Add the workspace to the bashrc
USER root
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Remove copied files
RUN rm -rf ${HOME}/catkin_ws/src

USER ${USER}
WORKDIR ${HOME}/catkin_ws
CMD [ "bash" ]
