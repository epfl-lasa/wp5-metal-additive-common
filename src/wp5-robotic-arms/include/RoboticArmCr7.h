/**
 * @file RoboticArmCr7.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <Eigen/Dense>
#include <variant>
#include <vector>

#include "IRoboticArmBase.h"

/**
 * @brief Child class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class RoboticArmCr7 : public IRoboticArmBase {
public:
  explicit RoboticArmCr7();

  /**
   * @brief Get the forward kinematics of the robotic arm.
   * @param ikSolver Type of inverse kinematics solver to use.
   * @param jointPos Joint positions of the robotic arm.
   */
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFK(IkSolver ikSolver, const std::vector<double>& jointPos);

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   * @param ikSolver Type of inverse kinematics solver to use.
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @param jointPos Joint positions of the robotic arm.
   * @return Pair of the return code and the next joint positions.
   */
  bool getIK(IkSolver ikSolver,
             const Eigen::Quaterniond& quaternion,
             const Eigen::Vector3d& position,
             std::vector<double>& jointPos,
             const KDL::JntArray& nominal);
};
