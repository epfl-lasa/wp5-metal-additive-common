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
#include "IRosInterfaceBase.h"
#include "ik_geo.h"

/**
 * @brief Child class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class RoboticArmUr5 : public IRoboticArmBase {
public:
  // Declare the test class as a friend to allow access to private members
  friend class RoboticArmUr5Test_TestSwapJoints_Test;

  explicit RoboticArmUr5(ROSVersion rosVersion, std::string customYamlPath = "");
  ~RoboticArmUr5();

  /**
   * @brief Bring the getFK function from the parent class.
   */
  using IRoboticArmBase::getFK;

  /**
   * @brief Get the forward kinematics of the robotic arm.
   * @param jointPos Joint positions of the robotic arm.
   */
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFKGeo(const std::vector<double>& jointPos);

  /**
   * @brief Bring the getIK function from the parent class.
   */
  using IRoboticArmBase::getIK;

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @param jointPos Vector of joint positions of the robotic arm.
   * @return Pair of the return code and the next joint positions.
   */
  bool getIKGeo(const Eigen::Quaterniond& quaternion,
                const Eigen::Vector3d& position,
                std::vector<std::vector<double>>& jointPos);

  /**
   * @brief Get the current state of the robotic arm.
   * @return Tuple containing joint positions, velocities, and torques.
   */
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getState() override;

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

  void swapJoints_(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& currentRobotState);
};
