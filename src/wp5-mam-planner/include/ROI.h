/**
 * @file ROI.h
 * @brief Declaration of the Region of Interest class
 *
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 *
 * @version 0.2
 * @date 2024-10-22
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */
#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

class ROI {
public:
  struct Pose {
    static const uint SIZE = 7;

    Pose() = default;
    Pose(const std::string frame, const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& position) :
        frameRef_(frame), quaternion_(quaternion), position_(position) {}

    std::vector<double> toVector() const {
      return {quaternion_.x(),
              quaternion_.y(),
              quaternion_.z(),
              quaternion_.w(),
              position_.x(),
              position_.y(),
              position_.z()};
    }

    // Public getters for Pose members
    const std::string& getFrameRef() const { return frameRef_; }
    const Eigen::Vector3d& getPosition() const { return position_; }
    const Eigen::Quaterniond& getQuaternion() const { return quaternion_; }

  private:
    std::string frameRef_{};        ///< Reference frame of the pose
    Eigen::Quaterniond quaternion_; ///< Orientation of the pose
    Eigen::Vector3d position_;      ///< Position of the pose
  };

  ROI(std::string poseID) : id_(poseID) {};

  void clear();

  void print() const;

  // Getters and setters for the private members
  const std::string getID() const { return id_; }

  const Pose getPose(uint index) const;

  const std::vector<Pose> getPoses() const { return poses_; };

  const geometry_msgs::Pose getPoseROS(uint index) const;

  const std::vector<geometry_msgs::Pose> getPosesROS() const;

  void emplaceBackPose(const std::string frame, const std::vector<double>& poseVector);

  void emplaceBackPose(const std::string frame, const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position);

private:
  std::string id_{};
  std::vector<Pose> poses_{};
};