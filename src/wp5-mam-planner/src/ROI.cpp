/**
 * @file ROI.cpp
 * @brief Declaration of the Region of Interest class
 *
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 *
 * @version 0.2
 * @date 2024-10-22
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */

#include "ROI.h"

#include "conversion_tools.h"

using namespace std;

void ROI::clear() {
  id_.clear();
  poses_.clear();
}

void ROI::print() const {
  ROS_INFO_STREAM("[ROI] - ID: " << id_.c_str());

  uint idx = 0;
  for (const auto& pose : poses_) {
    Eigen::Vector3d pos = pose.getPosition();
    Eigen::Quaterniond quat = pose.getOrientation();

    ROS_INFO_STREAM("[ROI] - Pose " << idx++);
    ROS_INFO_STREAM("[ROI] - Reference Frame: " << pose.getFrameRef());
    ROS_INFO_STREAM("[ROI] - Position: " << pos.x() << pos.y() << pos.z());
    ROS_INFO_STREAM("[ROI] - Quaternion: " << quat.w() << quat.x() << quat.y() << quat.z());
  }
}

const ROI::Pose ROI::getPose(uint index) const {
  if (index <= poses_.size()) {
    return poses_[index];
  } else {
    ROS_ERROR_STREAM("[ROI] - Pose index " << index << " out of range.");
    return Pose();
  }
}

const geometry_msgs::Pose ROI::getPoseROS(uint index) const {
  if (index >= poses_.size()) {
    ROS_ERROR_STREAM("[ROI] - Pose index " << index << " out of range.");
    return geometry_msgs::Pose();
  }

  return ConversionTools::eigenToGeometry(poses_[index].getOrientation(), poses_[index].getPosition());
}

const std::vector<geometry_msgs::Pose> ROI::getPosesROS() const {
  std::vector<geometry_msgs::Pose> posesROS;

  for (const auto& pose : poses_) {
    posesROS.push_back(ConversionTools::eigenToGeometry(pose.getOrientation(), pose.getPosition()));
  }

  return posesROS;
}

void ROI::emplaceBackPose(const std::string frame, const vector<double>& poseVector) {
  if (poseVector.size() != Pose::SIZE) {
    ROS_ERROR_STREAM("[ROI] - Pose vector size " << poseVector.size() << " not valid, should be " << Pose::SIZE << ".");
    return;
  }

  poses_.emplace_back(frame,
                      Eigen::Vector3d(poseVector[0], poseVector[1], poseVector[2]),
                      Eigen::Quaterniond(poseVector[3], poseVector[4], poseVector[5], poseVector[6]));
}

void ROI::emplaceBackPose(const std::string frame, const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient) {
  poses_.emplace_back(frame, pos, orient);
}
