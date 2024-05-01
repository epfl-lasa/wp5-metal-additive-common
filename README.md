# Robetarme WP5 Deliverable

## Overview

The repository comprises multiple packages written in python first, for development purpose. Currently, it is designed to work with ROS Noetic and MoveIt, but plans are underway to transition it to ROS2.

## Clone

This repository contains submodules :

```bash
git clone --recurse-submodules git@github.com:epfl-lasa/wp5-metal-additive.git
```

## Installation

We are currently using docker and its docker compose option. Please set it up before going forward.

When everything is up and running, follow these steps to set up the environment at the roots of the folder. All the environment variables needed in the docker are set up inside the .env file. You can modify them as you need.:

```bash
# Initialize the submodules
git submodule update --init --recursive
git submodule update --recursive --remote

# Build the Docker containers
docker compose build

# Start the Docker containers in detached mode
docker compose up -d

# Access the Docker container's shell
docker exec -it wp5-metal-additive-ros-1 bash
```

This will set up the necessary environment within Docker for running the codebase.

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
