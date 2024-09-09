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
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <string>
#include <trac_ik/trac_ik.hpp>
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
   * @param jointPositions Joint positions of the robotic arm.
   */
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFK(IkSolver ikSolver, const std::vector<double>& jointPositions);

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   * @param ikSolver Type of inverse kinematics solver to use.
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @return Pair of the return code and the next joint positions.
   */
  std::variant<std::vector<double>, std::vector<std::vector<double>>> getIK(IkSolver ikSolver,
                                                                            const Eigen::Quaterniond& quaternion,
                                                                            const Eigen::Vector3d& position);

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

  double epsilon_ = 1e-5;                             ///< Epsilon for the IK solver
  double timeout_ = 0.01;                             ///< Timeout for the IK solver
  TRAC_IK::SolveType solverType_ = TRAC_IK::Distance; ///< Solve type for the IK solver

  TRAC_IK::TRAC_IK* tracIkSolver_ = nullptr;            ///< TRAC-IK solver
  KDL::ChainFkSolverPos_recursive* fkSolver_ = nullptr; ///< FK solver
  KDL::Chain chain_;                                    ///< KDL chain
  KDL::JntArray ll_, ul_;                               ///< lower joint limits, upper joint limits

  ik_geo::Robot* robotGeoSolver_ = nullptr; ///< IK-Geo solver

  void initializeTracIkSolver_();

  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getTracFkSolution_(const std::vector<double>& jointPositions);
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFkGeoSolution_(const std::vector<double>& jointPositions);

  std::vector<double> getTracIkSolution_(const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& position);
  std::vector<std::vector<double>> getIkGeoSolution_(const Eigen::Quaterniond& quaternion,
                                                     const Eigen::Vector3d& position);
};
