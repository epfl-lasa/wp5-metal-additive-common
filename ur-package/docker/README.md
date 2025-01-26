# UR Polyscope docker

## Table of Contents

- [Overview](#overview)
- [Requirements](#requirements)
- [Getting Started](#getting-started)
- [Maintainers](#maintainers)

## Overview

This folder contains the docker to run both polyscope versions, allowing to deal with both cb and e series from UR.

## Requirements

This code is tested on both Ubuntu 20.04 and 22.04. It uses docker, side to docker compose, with the nvidia toolkit if any nvidia graphics card is installed. So you need to install :

- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [docker compose](https://docs.docker.com/compose/install/linux/#install-the-plugin-manually)

If you have an nvidia graphic card :

- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Getting Started

To run the needed version, simply run:

```bash
docker compose --profile cb-series up --build # for cb-series robot
docker compose --profile e-series up --build # for e-series robot
```

It will build the docker images if it is not already done, then setting up the docker container.

If you need more informations on how to run a specific program from both of these version, especially if you need to know how to connect them with ROS, have a look at their respective README :

- [cb-series README](cb-series-docker/README.md)
- [e-series README](e-series-docker/README.md)

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Last Update - 2025-01-26
