/**
 * @file debug_tools.h
 * @brief A collection of utility functions for debugging and visualization.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.1
 * @date 2024-10-29
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 *
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>

#include <array>
#include <sstream>
#include <string>
#include <vector>

#include "conversion_tools.h"

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
 * @tparam T The type of elements in the vector.
 * @param vec The vector to convert to a string.
 * @return A string representation of the vector.
 */
template <typename T>
std::string getVecString(const std::vector<T>& vec) {
  // Check if the vector is empty
  if (vec.empty()) {
    return "[]";
  }

  // Create a string stream to build the string representation
  std::ostringstream oss;
  oss << "[";

  // Handle first element separately to avoid adding a comma before it
  auto it = vec.begin();
  oss << *it;
  ++it;

  for (; it != vec.end(); ++it) {
    oss << ", " << *it;
  }

  oss << "]";
  return oss.str();
}

/**
 * @brief Converts an Eigen object to a string representation.
 *
 * This function template takes an Eigen object, converts it to a vector using
 * `ConversionTools::eigenToVector`, and then converts that vector to a string
 * using `getVecString<double>`.
 *
 * @tparam T The type of the Eigen object.
 * @param obj The Eigen object to be converted to a string.
 * @return A string representation of the Eigen object.
 */
template <typename T>
std::string getEigenString(const T& obj) {
  return getVecString<double>(ConversionTools::eigenToVector(obj));
}

template <typename T>
std::string getGeoString(const T& obj) {
  return getVecString<double>(ConversionTools::geometryToVector(obj));
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

/**
 * @brief Converts a TransformStamped message to a string representation.
 *
 * This function takes a geometry_msgs::TransformStamped object and converts it
 * into a human-readable string format. The resulting string contains the
 * translation and rotation components of the transform.
 *
 * @param transform The TransformStamped message to be converted.
 * @return A string representation of the TransformStamped message.
 */
std::string getTransformString(const geometry_msgs::TransformStamped& transform);

/**
 * @brief Publishes a set of waypoints to a ROS topic.
 *
 * @param frameId The frame ID to associate with the waypoints.
 * @param waypoints A vector of 3D vectors representing the waypoints.
 * @param pub The ROS publisher to use for publishing the waypoints.
 * @param color An array of four floats representing the color (RGBA) to use for the waypoints.
 */
void publishWaypoints(const std::string& frameId,
                      const std::vector<Eigen::Vector3d>& waypoints,
                      const ros::Publisher& pub,
                      const std::array<float, 4> color);

/**
 * @brief Publishes a path of waypoints to a given ROS topic.
 *
 * This function takes a frame ID, a vector of waypoints, and a ROS publisher,
 * and publishes the path to the specified topic.
 *
 * @param frameId The ID of the reference frame for the waypoints.
 * @param waypoints A vector of geometry_msgs::Pose representing the waypoints of the path to be published.
 * @param pub The ROS publisher to publish the waypoints to.
 */
void publishPath(const std::string& frameId,
                 const std::vector<geometry_msgs::Pose>& waypoints,
                 const ros::Publisher& pub);

/**
 * @brief Publishes a given trajectory to a specified ROS topic.
 *
 * This function takes a moveit_msgs::RobotTrajectory object and a ROS publisher, and publishes
 * the trajectory to the topic associated with the publisher.
 *
 * @param robotState The robot state associated with the trajectory.
 * @param trajectory The trajectory to be published.
 * @param pub The ROS publisher to publish the trajectory to.
 */
void publishTrajectory(const moveit::core::RobotState& robotState,
                       const moveit_msgs::RobotTrajectory& trajectory,
                       const ros::Publisher& pub);

/**
 * @brief Prints the time stamps of each point in the given robot trajectory.
 *
 * This function iterates through the trajectory points in the provided
 * RobotTrajectory message and prints the time stamps associated with each point.
 *
 * @param trajectory The robot trajectory whose time stamps are to be printed.
 */
void printTrajectoryTimeStamps(const moveit_msgs::RobotTrajectory& trajectory);

/**
 * @brief Pauses the program execution and waits for user input.
 *
 * This function displays a message to the user and waits for the user to press Enter before continuing.
 *
 * @param message The message to display to the user.
 */
void waitOnUser(const std::string& message);
} // namespace DebugTools