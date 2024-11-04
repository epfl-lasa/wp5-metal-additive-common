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

#include <sstream>
#include <string>
#include <vector>

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
 * @brief Get a string representation of a vector.
 *
 * This function returns a string representation of a vector. The
 * string includes the values of each element in the vector, separated by commas.
 *
 * @param vec The vector to convert to a string.
 * @return A string representation of the vector.
 */
template <typename T>
std::string getVecString(const std::vector<T>& vec) {
  std::ostringstream oss;
  oss << "[";

  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i];

    if (i < vec.size() - 1) {
      oss << ", ";
    }
  }

  oss << "]";
  return oss.str();
}

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