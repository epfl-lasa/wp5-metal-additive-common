# WP5 - Common Utilities

Implement tools to be used in the different ROS packages.

## Table of Contents

- [Overview](#overview)
- [Getting Started](#getting-started)
- [Architecture](#architecture)
- [Maintainers](#maintainers)

## Overview

- **conversion_tools -** Implement conversion functions to transformPoses, extractVector, convert from a format to an other (between eigen, ROS and std::vector).
- **debug_tools -** Implement usefull debugging tools. It helps to convert from variable to its human readable string version. It also implements different display function and the waitOnUser one to wait on a feedback from the user.
- **math_tools -** Implement multiple usefull mathematical functions from ROS geometry transforms to basic mathematics.
- **yaml_tools -** Implement tools to extract value from YAML file, from YAML file path validation to its values extraction.

## Getting Started

To use one, or more, of these library, just include the ROS package in yours.

## Architecture

- **include -** header files to be included
- **src -** source files

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Last Update - 2025-01-26
