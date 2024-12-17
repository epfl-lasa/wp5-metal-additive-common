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
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

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

void publishWaypoints(const std::string& frameId,
                      const std::vector<Eigen::Vector3d>& waypoints,
                      const ros::Publisher& pub,
                      const std::array<float, 4> color) {
  float TIME_WAIT = 0.2;
  size_t NB_PUBLISH = 3;
  float RADIUS = 0.05;

  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();

  marker.ns = "waypoints";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the size of the points
  marker.scale.x = RADIUS;
  marker.scale.y = RADIUS;
  marker.scale.z = RADIUS;

  // Set the color of the points
  marker.color.a = color[0];
  marker.color.r = color[1];
  marker.color.g = color[2];
  marker.color.b = color[3];

  for (auto& waypoint : waypoints) {
    marker.points.push_back(ConversionTools::eigenToGeometry(waypoint));
  }

  for (size_t i = 0; i < NB_PUBLISH; ++i) {
    pub.publish(marker);
    ros::Duration(TIME_WAIT).sleep();
  }
}

void publishPath(const std::string& frameId,
                 const std::vector<geometry_msgs::Pose>& waypoints,
                 const ros::Publisher& pub) {
  float TIME_WAIT = 0.2;
  size_t NB_PUBLISH = 3;

  nav_msgs::Path path;
  path.header.frame_id = frameId;
  path.header.stamp = ros::Time::now();
  path.poses.resize(waypoints.size());

  for (size_t i = 0; i < waypoints.size(); ++i) {
    path.poses[i].header.frame_id = frameId;
    path.poses[i].header.stamp = ros::Time::now();
    path.poses[i].pose = waypoints[i];
  }

  for (size_t i = 0; i < NB_PUBLISH; ++i) {
    pub.publish(path);
    ros::Duration(TIME_WAIT).sleep();
  }
}

void publishTrajectory(const moveit::planning_interface::MoveGroupInterface& moveGroup,
                       const moveit_msgs::RobotTrajectory& trajectory,
                       const ros::Publisher& pub) {
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