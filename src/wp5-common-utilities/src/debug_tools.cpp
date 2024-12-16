/**
 * @file debug_tools.cpp
 * @brief A collection of utility functions for debugging and visualization.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.1
 * @date 2024-10-29
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#include "debug_tools.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "conversion_tools.h"

namespace DebugTools {
std::string getPoseString(const geometry_msgs::Pose& pose) {
  std::string poseString = "";

  // Convert Position to string
  poseString += "Position - xyz " + getGeoString(pose.position) + " ";

  // Convert Orientation to string
  poseString += "Orientation - xyzw " + getGeoString(pose.orientation);

  return poseString;
}

std::string getTransformString(const geometry_msgs::TransformStamped& transform) {
  std::string transformString = "";

  // Convert Position to string
  transformString += "Translation - xyz " + getGeoString(transform.transform.translation) + " ";

  // Convert Orientation to string
  transformString += "Rotation - xyzw " + getGeoString(transform.transform.rotation);

  return transformString;
}

void publishPose(const geometry_msgs::Pose& pose, const std::string& frameId, ros::Publisher& pub) {
  float TIME_WAIT = 0.2;
  size_t NB_PUBLISH = 3;

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = frameId;
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.pose = pose;

  for (size_t i = 0; i < NB_PUBLISH; ++i) {
    pub.publish(poseStamped);
    ros::Duration(TIME_WAIT).sleep();
  }
}

void publishTrajectory(const moveit::planning_interface::MoveGroupInterface& moveGroup,
                       const moveit_msgs::RobotTrajectory& trajectory,
                       ros::Publisher& pub) {
  moveit_msgs::DisplayTrajectory displayTrajectory;
  moveit::core::robotStateToRobotStateMsg(*moveGroup.getCurrentState(), displayTrajectory.trajectory_start);
  displayTrajectory.trajectory.push_back(trajectory);
  pub.publish(displayTrajectory);
}

void printTrajectoryTimeStamps(const moveit_msgs::RobotTrajectory& trajectory) {
  const auto& points = trajectory.joint_trajectory.points;
  for (size_t i = 0; i < points.size(); ++i) {
    ROS_INFO("Waypoint %zu: time_from_start = %f", i, points[i].time_from_start.toSec());
  }
}

void waitOnUser(const std::string& message) {
  bool msgShowed = false;
  bool userInput = false;

  while (!userInput) {
    if (!msgShowed) {
      ROS_INFO_STREAM(message);
      msgShowed = true;
    }

    if (std::cin.get() == '\n') {
      userInput = true;
    }

    ros::Duration(0.1).sleep();
  }
}
} // namespace DebugTools