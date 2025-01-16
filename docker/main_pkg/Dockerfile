################################################################################
# There are places where your actions are needed in this Dockerfile.
# These are marked with the label `ACTIONS_NEEDED`.
# After you are done with those:
#
# Pull the robetarme noetic base image with
# `docker login registry.gitlab.com`
# `docker pull registry.gitlab.com/certh-iti-robotics-lab/robetarme/robetarme/ros-noetic:base`
#
# Then build this Dockerfile with
# `docker compose build`,
#
# and run it with
# `docker compose up -d`.
#
# Then gain access to the container with
# `docker attach CONTAINER_NAME`,
# and `docker exec -it --user robetarme_user CONTAINER_NAME bash`
# ** where CONTAINER_NAME is the one specified in the `docker-compose.yml` file under
# services/.../container_name.
#
# Then simply launch your ROS noetic packages.
################################################################################

FROM registry.gitlab.com/certh-iti-robotics-lab/robetarme/robetarme/ros-noetic:base

ARG USER=robetarme_user
ARG CMAKE_BUILD_TYPE=Debug

#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# Do not modify above this line
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# ACTIONS_NEEDED
# ------------------------------------------------------------------------------
# Install your dependencies here
# ------------------------------------------------------------------------------
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
    valgrind \
    libeigen3-dev \
    libboost-all-dev

RUN pip3 install --upgrade pip

# Install catkin tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    python3-catkin-tools \
    python3-osrf-pycommon

# Install moveit tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    python3-vcstool \
    ros-noetic-rqt-joint-trajectory-controller \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-moveit \
    ros-noetic-moveit-commander \
    ros-noetic-moveit-ros-planning-interface \
    ros-noetic-moveit-ros-move-group \
    ros-noetic-moveit-visual-tools \
    ros-noetic-chomp-motion-planner \
    ros-noetic-moveit-chomp-optimizer-adapter \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-scaled-joint-trajectory-controller \
    ros-noetic-speed-scaling-interface \
    ros-noetic-speed-scaling-state-controller \
    ros-noetic-pass-through-controllers \
    ros-noetic-ur-client-library \
    ros-noetic-pcl-ros

# Install trac ik
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-noetic-trac-ik \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-diagnostic-updater

# Install NLopt library
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    libnlopt-dev \
    libnlopt-cxx-dev

# Install ANiMA's dependencies
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-noetic-catkin-virtualenv \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-pilz-industrial-motion-planner \
    ros-noetic-industrial-core \
    ros-noetic-industrial-robot-status-interface

# Install UR dependencies
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-noetic-ur-msgs

# Install realsense camera dependencies
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description

# Install Rust
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    curl

USER ${USER}
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN echo "source /home/${USER}/.cargo/env" >> ~/.bashrc

ENV CARGO_HOME="/home/${USER}/.cargo"
ENV PATH="/home/${USER}/.cargo/bin:${PATH}"
ENV CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}

# ------------------------------------------------------------------------------

# ACTIONS_NEEDED
# ------------------------------------------------------------------------------
# Place your ROS packages into the host's catkin_ws/src directory
# i.e. $PWD/../../catkin_ws/src. This directory will be copied verbatim into
# the image.
#
# CAUTION! Anything else you `COPY` must be specified with
# `COPY --chown=${USER}:${USER} source destination`
# ------------------------------------------------------------------------------

#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# Do not modify below this line
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# ------------------------------------------------------------------------------
# COPY the src/ directory of your host's ROS workspace into the container's
# ------------------------------------------------------------------------------
COPY --chown=${USER}:${USER} catkin_ws/src/ /home/${USER}/catkin_ws/src/
# ------------------------------------------------------------------------------

RUN rosdep update

USER root
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN rosdep install --from-paths /home/${USER}/catkin_ws/src --ignore-src -r -y

# Install modbus python dependencies
WORKDIR /home/${USER}/catkin_ws/src/ros-modbus-device-driver
RUN pip3 install -r requirements.txt

# ------------------------------------------------------------------------------
# catkin build ROS noetic packages
USER ${USER}
RUN cd /home/${USER}/catkin_ws                                              && \
    source /opt/ros/noetic/setup.bash                                       && \
    catkin build                                                            && \
    source /home/${USER}/catkin_ws/devel/setup.bash
USER root
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Bind mount/shared directory between container and host;
# robetarme_user is owner
# https://docs.docker.com/storage/bind-mounts/
USER ${USER}
RUN mkdir /home/${USER}/shared
USER root
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Entrypoint stuff
# ------------------------------------------------------------------------------
COPY --chown=${USER}:${USER} docker/main_pkg/entrypointd.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# ------------------------------------------------------------------------------

WORKDIR /home/${USER}
CMD ["/bin/bash"]