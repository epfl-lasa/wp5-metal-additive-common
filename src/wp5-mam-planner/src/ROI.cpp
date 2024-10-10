/**
 * @file ROI.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "ROI.h"

#include <ros/ros.h>

using namespace std;

void ROI::clear() {
  id_.clear();
  posStart_.setZero();
  posEnd_.setZero();
  quat_.setIdentity();
}

void ROI::print() const {
  ROS_INFO_STREAM("ID: " << id_.c_str());
  ROS_INFO_STREAM("Starting Position: " << posStart_.x() << posStart_.y() << posStart_.z());
  ROS_INFO_STREAM("Ending Position: " << posEnd_.x() << posEnd_.y() << posEnd_.z());
  ROS_INFO_STREAM("Quaternion: " << quat_.x() << quat_.y() << quat_.z() << quat_.w());
}

vector<double> ROI::getPoseVector(const string& posType) const {
  Eigen::Vector3d pos{};
  Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();

  if (posType == "start") {
    pos = posStart_;
  } else if (posType == "end") {
    pos = posEnd_;
  } else {
    ROS_ERROR("Invalid position type, should be either 'start' or 'end'.");
    return vector<double>();
  }

  return {pos.x(), pos.y(), pos.z(), identity.x(), identity.y(), identity.z(), identity.w()};
}
