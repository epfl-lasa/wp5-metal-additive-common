/**
 * @file ROI.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <string>
#include <vector>

class ROI {
public:
  ROI() = default;

  bool empty() const { return id_.empty(); }

  void clear() {
    id_.clear();
    posStart_.setZero();
    posEnd_.setZero();
    quat_.setIdentity();
  }

  void print() const {
    ROS_INFO_STREAM("ID: " << id_.c_str());
    ROS_INFO_STREAM("Starting Position: " << posStart_.x() << posStart_.y() << posStart_.z());
    ROS_INFO_STREAM("Ending Position: " << posEnd_.x() << posEnd_.y() << posEnd_.z());
    ROS_INFO_STREAM("Quaternion: " << quat_.x() << quat_.y() << quat_.z() << quat_.w());
  }

  std::vector<double> getPoseVector(const std::string& posType) const {
    Eigen::Vector3d pos{};
    Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();

    if (posType == "start") {
      pos = posStart_;
    } else if (posType == "end") {
      pos = posEnd_;
    } else {
      ROS_ERROR("Invalid position type, should be either 'start' or 'end'.");
      return std::vector<double>();
    }

    return {pos.x(), pos.y(), pos.z(), identity.x(), identity.y(), identity.z(), identity.w()};
  }

  // Getters and setters for the private members
  const std::string& getID() const { return id_; }
  void setID(const std::string& id) { id_ = id; }

  const Eigen::Vector3d& getPosStart() const { return posStart_; }
  void setPosStart(const Eigen::Vector3d& posStart) { posStart_ = posStart; }

  const Eigen::Vector3d& getPosEnd() const { return posEnd_; }
  void setPosEnd(const Eigen::Vector3d& posEnd) { posEnd_ = posEnd; }

  const Eigen::Quaterniond& getQuat() const { return quat_; }
  void setQuat(const Eigen::Quaterniond& quat) { quat_ = quat; }

private:
  std::string id_{};
  Eigen::Vector3d posStart_{};
  Eigen::Vector3d posEnd_{};
  Eigen::Quaterniond quat_{};
};