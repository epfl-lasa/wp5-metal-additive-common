/**
 * @file math_tools.cpp
 * @author [Louis Munier] - lmunier@protonmail.com
 * @brief
 * @version 0.3
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#include "math_tools.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace MathTools {

const bool isNumber(const std::string& str) {
  if (str.empty()) {
    return false;
  }

  std::istringstream iss(str);
  double value;
  iss >> std::noskipws >> value; // noskipws considers leading whitespace invalid

  return iss.eof() && !iss.fail();
}

const bool areQuatEquivalent(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double tolerance) {
  Eigen::Matrix3d rot1 = q1.toRotationMatrix();
  Eigen::Matrix3d rot2 = q2.toRotationMatrix();

  return (rot1 - rot2).norm() < tolerance;
}

const bool arePosEquivalent(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double tolerance) {
  return (p1 - p2).norm() < tolerance;
}

geometry_msgs::Pose transformPose(tf2_ros::Buffer& tfBuffer,
                                  const std::string& source_frame,
                                  const std::string& target_frame,
                                  const geometry_msgs::Pose& pose) {
  if (source_frame == target_frame) {
    return pose;
  }

  // Define the pose in the source frame
  geometry_msgs::PoseStamped source_pose;
  source_pose.header.frame_id = source_frame;
  source_pose.header.stamp = ros::Time::now();
  source_pose.pose = pose;

  try {
    // Lookup the transform from source frame to target frame
    geometry_msgs::TransformStamped transformStamped =
        tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));

    // Transform the pose
    geometry_msgs::PoseStamped target_pose;
    tf2::doTransform(source_pose, target_pose, transformStamped);

    return target_pose.pose;
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("[MathTools] - Received an exception trying to transform a pose: %s", ex.what());
    return pose;
  }
}

} // namespace MathTools