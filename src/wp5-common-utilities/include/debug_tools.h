/**
 * @file debug_tools.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-29
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <geometry_msgs/Pose.h>

#include <string>

/**
 * @namespace DebugTools
 * @brief A collection of utility functions for debugging and visualization.
 *
 * This namespace provides functions to visualize waypoints, trajectories, and other
 * relevant information in Rviz and other visualization tools. The functions are
 * designed to facilitate debugging and monitoring of the robot's behavior during
 * planning and execution.
 */
namespace DebugTools {
/**
 * @brief Get a string representation of a Pose message.
 *
 * This function returns a string representation of a Pose message. The string
 * includes the position and orientation of the pose.
 *
 * @param pose The Pose message to convert to a string.
 * @return A string representation of the Pose message.
 */
std::string getPoseString(const geometry_msgs::Pose& pose);
} // namespace DebugTools