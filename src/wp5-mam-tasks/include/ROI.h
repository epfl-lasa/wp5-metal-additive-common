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
    static const uint SIZE = 6;

    Pose() = default;
    Pose(const std::string frame, const Eigen::Vector3d& position, const Eigen::Vector3d& normal) :
        frameRef_(frame), position_(position), normal_(normal) {}

    std::vector<double> toVector() const {
      return {position_.x(), position_.y(), position_.z(), normal_.x(), normal_.y(), normal_.z()};
    }

    // Public getters for Pose members
    const std::string& getFrameRef() const { return frameRef_; }
    const Eigen::Vector3d& getPosition() const { return position_; }
    const Eigen::Vector3d& getNormal() const { return normal_; }

  private:
    std::string frameRef_{};   ///< Reference frame of the pose
    Eigen::Vector3d position_; ///< Position of the pose
    Eigen::Vector3d normal_;   ///< Orientation of the pose
  };

  ROI(std::string poseID) : id_(poseID) {};

  void clear();

  void print() const;

  // Getters and setters for the private members
  const std::string getID() const { return id_; }

  const Pose getPose(uint index) const;

  const std::vector<Pose> getPoses() const { return poses_; };

  const std::vector<Eigen::Vector3d> getPositions() const;

  void emplaceBackPose(const std::string frame, const std::vector<double>& poseVector);

  void emplaceBackPose(const std::string frame, const Eigen::Vector3d& pos, const Eigen::Vector3d& normal);

private:
  std::string id_{};
  std::vector<Pose> poses_{};
};