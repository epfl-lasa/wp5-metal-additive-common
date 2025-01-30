/**
 * @file ROI.cpp
 * @brief Declaration of the Region of Interest class
 *
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 *
 * @version 0.2
 * @date 2024-10-22
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#include "ROI.h"

#include "conversion_tools.h"
#include "debug_tools.h"

using namespace std;

void ROI::clear() {
  id_.clear();
  poses_.clear();
}

void ROI::print() const {
  ROS_INFO_STREAM("[ROI] - ID: " << id_.c_str());

  uint idx = 0;
  for (const auto& pose : poses_) {
    Eigen::Vector3d normal = pose.getNormal();
    Eigen::Vector3d pos = pose.getPosition();

    ROS_INFO_STREAM("[ROI] - Pose ID " << idx++);
    ROS_INFO_STREAM("[ROI] - Reference Frame: " << pose.getFrameRef());
    ROS_INFO_STREAM("[ROI] - Position xyz: " << DebugTools::getEigenString(pos));
    ROS_INFO_STREAM("[ROI] - Normal xyz: " << DebugTools::getEigenString(normal));
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

const std::vector<Eigen::Vector3d> ROI::getPositions() const {
  std::vector<Eigen::Vector3d> positions;

  for (const auto& pose : poses_) {
    positions.push_back(pose.getPosition());
  }

  return positions;
}

void ROI::emplaceBackPose(const std::string frame, const vector<double>& poseVector) {
  if (poseVector.size() != Pose::SIZE) {
    ROS_ERROR_STREAM("[ROI] - Pose vector size " << poseVector.size() << " not valid, should be " << Pose::SIZE << ".");
    return;
  }

  poses_.emplace_back(frame,
                      Eigen::Vector3d(poseVector[0], poseVector[1], poseVector[2]),
                      Eigen::Vector3d(poseVector[3], poseVector[4], poseVector[5]));
}

void ROI::emplaceBackPose(const std::string frame, const Eigen::Vector3d& pos, const Eigen::Vector3d& normal) {
  poses_.emplace_back(frame, pos, normal);
}
