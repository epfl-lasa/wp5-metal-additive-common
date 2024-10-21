# Handle ros distro
ARG ROS_DISTRO=noetic
FROM ghcr.io/aica-technology/ros-ws:${ROS_DISTRO} as ros-ws

# User provided arguments
ARG HOST_GID=1000
ARG GIT_NAME=""
ARG GIT_EMAIL=""
ARG USE_SIMD=OFF
ARG NB_CPU_RESIZED=6
# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

### Add the user to the current GID of the host to avoid permisson issues in volumes
# AICA uses the same name for user and user group
ENV USER_GROUP=${USER}
WORKDIR /home/${USER}/
USER root
RUN if [ "HOST_GID" != "1000"] ; \
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

### Add a few tools
RUN sudo apt update && sudo apt install -y \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-industrial-robot-status-controller \
    ros-${ROS_DISTRO}-ur-msgs \
    ros-${ROS_DISTRO}-ur-robot-driver \
    && sudo apt upgrade -y && sudo apt clean

# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi

# Remove temp files
RUN sudo ldconfig
RUN rm -rf /tmp/*

# Install ur5 LASA
WORKDIR /home/${USER}
RUN git clone --depth 1 https://github.com/epfl-lasa/ur5_lasa.git

RUN cp -R ur5_lasa/src/ ros_ws/src/

# Cleaning the workspace
RUN rm -r /home/${USER}/ur5_lasa

### apt update and clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean

# Give bashrc back to user
WORKDIR /home/${USER}
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi

### Build ros workspace
WORKDIR /home/${USER}/ros_ws
RUN rosdep update
RUN source /home/${USER}/.bashrc && rosdep install --from-paths src --ignore-src -r -y
RUN source /home/${USER}/.bashrc && catkin_make && source devel/setup.bash;

### Final apt clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean


