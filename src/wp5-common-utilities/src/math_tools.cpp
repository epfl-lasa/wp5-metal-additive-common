/**
 * @file math_tools.cpp
 * @author [Louis Munier] - lmunier@protonmail.com
 * @brief A collection of mathematical utility functions to centralize common operations.
 *
 * @version 0.4
 * @date 2024-11-20
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#include "math_tools.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "debug_tools.h"

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
  double dotProduct = q1.dot(q2);
  bool areEquivalent = std::abs(dotProduct) > 1.0 - tolerance;

#ifdef DEBUG_MODE
  if (!areEquivalent) {
    std::string quat1 = DebugTools::getEigenQuatString(q1);
    std::string quat2 = DebugTools::getEigenQuatString(q2);

    ROS_WARN_STREAM("[MathTools] - Quaternions are not equivalent: " << quat1 << " and " << quat2);
  }
#endif

  return areEquivalent;
}

const bool arePosEquivalent(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double tolerance) {
  bool areEquivalent = (p1 - p2).norm() < tolerance;

#ifdef DEBUG_MODE
  if (!areEquivalent) {
    std::string pos1 = DebugTools::getEigenVecString(p1);
    std::string pos2 = DebugTools::getEigenVecString(p2);

    ROS_WARN_STREAM("[MathTools] - Positions are not equivalent: " << pos1 << " and " << pos2);
  }
#endif

  return areEquivalent;
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