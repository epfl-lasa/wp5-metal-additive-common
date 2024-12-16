/**
 * @file math_tools.cpp
 * @brief A collection of mathematical utility functions to centralize common operations.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.4
 * @date 2024-11-20
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#include "math_tools.h"

#include <geometry_msgs/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "conversion_tools.h"
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
    std::string quat1 = DebugTools::getEigenString<Eigen::Quaterniond>(q1);
    std::string quat2 = DebugTools::getEigenString<Eigen::Quaterniond>(q2);

    ROS_WARN_STREAM("[MathTools] - Quaternions are not equivalent: " << quat1 << " and " << quat2);
  }
#endif

  return areEquivalent;
}

const bool arePosEquivalent(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double tolerance) {
  bool areEquivalent = (p1 - p2).norm() < tolerance;

#ifdef DEBUG_MODE
  if (!areEquivalent) {
    std::string pos1 = DebugTools::getEigenString<Eigen::Vector3d>(p1);
    std::string pos2 = DebugTools::getEigenString<Eigen::Vector3d>(p2);

    ROS_WARN_STREAM("[MathTools] - Positions are not equivalent: " << pos1 << " and " << pos2);
  }
#endif

  return areEquivalent;
}

const bool arePoseEquivalent(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
  Eigen::Quaterniond quat1 = ConversionTools::geometryToEigen(pose1).first;
  Eigen::Quaterniond quat2 = ConversionTools::geometryToEigen(pose2).first;

  Eigen::Vector3d pos1 = ConversionTools::geometryToEigen(pose1).second;
  Eigen::Vector3d pos2 = ConversionTools::geometryToEigen(pose2).second;

  return areQuatEquivalent(quat1, quat2) && arePosEquivalent(pos1, pos2);
}

bool getTransform(const std::string& sourceFrame,
                  const std::string& targetFrame,
                  geometry_msgs::TransformStamped& transform) {
  try {
    // Lookup the transform from source frame to target frame
    tf2_ros::Buffer& tfBuffer = TfBufferSingleton::getInstance().getBuffer();
    transform = tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(1.0));

    return true;
  } catch (tf2::TransformException& ex) {
    ROS_ERROR_STREAM("[MathTools] - Received an exception trying to get a transform: " << ex.what());

    return false;
  }
}

geometry_msgs::Pose transformPose(const std::string& sourceFrame,
                                  const std::string& targetFrame,
                                  const geometry_msgs::Pose& pose) {
  if (sourceFrame == targetFrame) {
    return pose;
  }

  // Define the pose in the source frame
  geometry_msgs::PoseStamped source_pose;
  source_pose.header.frame_id = sourceFrame;
  source_pose.header.stamp = ros::Time::now();
  source_pose.pose = pose;

  // Lookup the transform from source frame to target frame
  geometry_msgs::TransformStamped transformStamped;
  if (!getTransform(sourceFrame, targetFrame, transformStamped)) {
    ROS_ERROR_STREAM("[MathTools] - Failed to get transform from " << sourceFrame << " to " << targetFrame);

    return pose;
  }

  // Transform the pose
  geometry_msgs::PoseStamped target_pose;
  tf2::doTransform(source_pose, target_pose, transformStamped);

  return target_pose.pose;
}

std::pair<Eigen::Quaterniond, Eigen::Vector3d> addOffset(
    const std::pair<Eigen::Quaterniond, Eigen::Vector3d>& quatPosNoOffset,
    const std::pair<Eigen::Quaterniond, Eigen::Vector3d>& offset) {
  // Apply the orientation offset, then normalize
  Eigen::Quaterniond desiredQuat = (quatPosNoOffset.first * offset.first).normalized();

  // Apply the position offset
  Eigen::Matrix3d rotationMatrix = desiredQuat.toRotationMatrix();
  Eigen::Vector3d posOffset = quatPosNoOffset.second + rotationMatrix * offset.second;

  // Create the resulting pair using std::make_pair
  return std::make_pair(desiredQuat, posOffset);
}

geometry_msgs::Pose addOffset(const geometry_msgs::Pose& poseNoOffset, const geometry_msgs::Pose& offset) {
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> quatPosNoOffset = ConversionTools::geometryToEigen(poseNoOffset);
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> offsetPair = ConversionTools::geometryToEigen(offset);

  std::pair<Eigen::Quaterniond, Eigen::Vector3d> desiredPose = addOffset(quatPosNoOffset, offsetPair);

  return ConversionTools::eigenToGeometry(desiredPose.first, desiredPose.second);
}

const Eigen::Vector3d getNormalFromPlan(const std::array<Eigen::Vector3d, 3>& pointsArray) {
  // Compute vector normal to plane define by the 3 points
  const Eigen::Vector3d planVecStart = pointsArray[1] - pointsArray[0];
  const Eigen::Vector3d planVecEnd = pointsArray[2] - pointsArray[0];
  const Eigen::Vector3d normalVector = planVecEnd.cross(planVecStart).normalized();

  return normalVector;
}

const Eigen::Quaterniond getQuatFromNormalTheta(const Eigen::Vector3d normalVector, const double theta) {
  // Rotate vector defined by plane, forming by points : pointsArray, by theta
  const Eigen::Quaterniond quatRotation(cos(theta / 2),
                                        normalVector.x() * sin(theta / 2),
                                        normalVector.y() * sin(theta / 2),
                                        normalVector.z() * sin(theta / 2));

  return quatRotation;
}

geometry_msgs::Pose applyRotationToPose(const geometry_msgs::Pose& pose, const geometry_msgs::Quaternion& rotation) {
  tf2::Quaternion poseOrientation;
  tf2::fromMsg(pose.orientation, poseOrientation);

  tf2::Quaternion rotationQuat;
  tf2::fromMsg(rotation, rotationQuat);

  // Apply the rotation to the pose's orientation
  tf2::Quaternion newOrientation = rotationQuat * poseOrientation;

  // Rotate the position vector
  tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  position = tf2::quatRotate(rotationQuat, position);

  // Create the new pose with the transformed orientation and position
  geometry_msgs::Pose newPose = pose;
  newPose.orientation = tf2::toMsg(newOrientation);
  newPose.position.x = position.x();
  newPose.position.y = position.y();
  newPose.position.z = position.z();

  return newPose;
}

} // namespace MathTools