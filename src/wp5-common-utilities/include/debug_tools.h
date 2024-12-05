/**
 * @file debug_tools.h
 * @brief A collection of utility functions for debugging and visualization.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.1
 * @date 2024-10-29
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>

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
 * @brief Converts an Eigen::VectorXd to a string representation.
 *
 * This function takes an Eigen::VectorXd object and returns a string
 * that represents the vector. The elements of the vector are
 * concatenated into a single string, typically separated by spaces
 * or commas.
 *
 * @param vec The Eigen::VectorXd to be converted to a string.
 * @return A string representation of the input vector.
 */
std::string getEigenVecString(const Eigen::VectorXd& vec);

/**
 * @brief Converts an Eigen Quaternion to a string representation.
 *
 * This function takes an Eigen::Quaterniond object and returns a string
 * that represents the quaternion in a readable format.
 *
 * @param quat The Eigen::Quaterniond object to be converted to a string.
 * @return A std::string representing the quaternion.
 */
std::string getEigenQuatString(const Eigen::Quaterniond& quat);

/**
 * @brief Publishes a given pose to a specified ROS topic.
 *
 * This function takes a geometry_msgs::Pose object, a frame ID string, and a ROS publisher,
 * and publishes the pose to the topic associated with the publisher.
 *
 * @param pose The pose to be published.
 * @param frameId The frame ID associated with the pose.
 * @param pub The ROS publisher to publish the pose to.
 */
void publishPose(const geometry_msgs::Pose& pose, const std::string& frameId, ros::Publisher& pub);

/**
 * @brief Publishes a given trajectory to a specified ROS topic.
 *
 * This function takes a moveit_msgs::RobotTrajectory object and a ROS publisher, and publishes
 * the trajectory to the topic associated with the publisher.
 *
 * @param moveGroup The MoveGroupInterface object associated with the trajectory.
 * @param trajectory The trajectory to be published.
 * @param pub The ROS publisher to publish the trajectory to.
 */
void publishTrajectory(const moveit::planning_interface::MoveGroupInterface& moveGroup,
                       const moveit_msgs::RobotTrajectory& trajectory,
                       ros::Publisher& pub);

/**
 * @brief Prints the time stamps of each point in the given robot trajectory.
 *
 * This function iterates through the trajectory points in the provided
 * RobotTrajectory message and prints the time stamps associated with each point.
 *
 * @param trajectory The robot trajectory whose time stamps are to be printed.
 */
void printTrajectoryTimeStamps(const moveit_msgs::RobotTrajectory& trajectory);
} // namespace DebugTools