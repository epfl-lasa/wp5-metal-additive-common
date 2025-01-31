/**
 * @file ROI.h
 * @brief Declaration of the Region of Interest class
 *
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 *
 * @version 0.2
 * @date 2024-10-22
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
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

    /**
     * @brief Convert the pose to a vector of doubles
     * @return A vector containing the position and orientation of the pose
     */
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

  /**
   * @brief Constructor for the ROI class
   * @param poseID The ID of the ROI
   */
  ROI(std::string poseID) : id_(poseID) {};

  /**
   * @brief Clear all poses in the ROI
   */
  void clear();

  /**
   * @brief Print the details of the ROI
   */
  void print() const;

  // Getters and setters for the private members
  const std::string getID() const { return id_; }

  const Pose getPose(uint index) const;

  const std::vector<Pose> getPoses() const { return poses_; };

  const std::vector<Eigen::Vector3d> getPositions() const;

  void emplaceBackPose(const std::string frame, const std::vector<double>& poseVector);

  void emplaceBackPose(const std::string frame, const Eigen::Vector3d& pos, const Eigen::Vector3d& normal);

private:
  std::string id_{};          ///< ID of the ROI
  std::vector<Pose> poses_{}; ///< Poses of the ROI
};