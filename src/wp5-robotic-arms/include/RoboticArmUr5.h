/**
 * @file RoboticArmUr5.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <array>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "IRoboticArmBase.h"
#include "ik_geo.h"

// clang-format off
constexpr double UR5_H_MATRIX[] = {
    0.0, 0.0, 1.0,
    0.0, -1.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0,
    0.0, -1.0, 0.0
};

constexpr double UR5_P_MATRIX[] = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0892,
    -0.425, 0.0, 0.0,
    -0.3922, 0.0, 0.0,
    0.0, -0.1091, 0.0,
    0.0, 0.0, -0.0946,
    0.0, -0.0823, 0.0
};
// clang-format on

/**
 * @brief Child class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class RoboticArmUr5 : public IRoboticArmBase {
public:
  explicit RoboticArmUr5();
  ~RoboticArmUr5();

  /**
   * @brief Get the forward kinematics of the robotic arm.
   * @param ikSolver Type of inverse kinematics solver to use.
   * @param jointPos Joint positions of the robotic arm.
   */
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFK(IkSolver ikSolver, const std::vector<double>& jointPos) override;

  /**
   * @brief Bring the getIK function from the parent class.
   */
  using IRoboticArmBase::getIK;

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   * @param ikSolver Type of inverse kinematics solver to use.
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @param jointPos Vector of joint positions of the robotic arm.
   * @return Pair of the return code and the next joint positions.
   */
  bool getIK(IkSolver ikSolver,
             const Eigen::Quaterniond& quaternion,
             const Eigen::Vector3d& position,
             std::vector<std::vector<double>>& jointPos);

private:
  // clang-format off
  static constexpr double UR5_H_MATRIX[] = {
      0.0, 0.0, 1.0,
      0.0, -1.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0,
      0.0, -1.0, 0.0
  };

  static constexpr double UR5_P_MATRIX[] = {
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0892,
      -0.425, 0.0, 0.0,
      -0.3922, 0.0, 0.0,
      0.0, -0.1091, 0.0,
      0.0, 0.0, -0.0946,
      0.0, -0.0823, 0.0
  };
  // clang-format on

  ik_geo::Robot* robotGeoSolver_ = nullptr; ///< IK-Geo solver

  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFkGeoSolution_(const std::vector<double>& jointPos);
  bool getIkGeoSolution_(const Eigen::Quaterniond& quaternion,
                         const Eigen::Vector3d& position,
                         std::vector<std::vector<double>>& jointPos);
};
