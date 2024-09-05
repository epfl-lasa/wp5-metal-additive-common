/**
 * @file RoboticArmUr5.h
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
#include <array>
#include <string>
#include <trac_ik/trac_ik.hpp>
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
   * @param jointPositions Joint positions of the robotic arm.
   */
  std::vector<double> getFK(std::vector<double> jointPositions);

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   * @param ikSolver Type of inverse kinematics solver to use.
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @return Pair of the return code and the next joint positions.
   */
  std::variant<std::vector<double>, std::vector<std::vector<double>>> getIK(IkSolver ikSolver,
                                                                            Eigen::Quaterniond quaternion,
                                                                            Eigen::Vector3d position);

private:
  TRAC_IK::TRAC_IK* tracIkSolver_ = nullptr; ///< TRAC-IK solver
  ik_geo::Robot* ikGeoSolver_ = nullptr;     ///< IK-Geo solver

  std::vector<double> getTracIkSolution_(Eigen::Quaterniond quaternion, Eigen::Vector3d position);
  std::vector<std::vector<double>> getIkGeoSolution_(Eigen::Quaterniond quaternion, Eigen::Vector3d position);
};
