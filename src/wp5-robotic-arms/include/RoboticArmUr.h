/**
 * @file RoboticArmUr.h
 * @author [Louis Munier] - lmunier@protonmail.com
 * @brief
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
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
class RoboticArmUr : public IRoboticArmBase {
public:
  explicit RoboticArmUr(ROSVersion rosVersion,
                        std::string robotName,
                        std::string configFilename,
                        const std::array<double, 18>& hMatrix,
                        const std::array<double, 21>& pMatrix);
  ~RoboticArmUr();

  /**
   * @brief Bring the getFKTrac function from the parent class.
   */
  using IRoboticArmBase::getFKTrac;

  /**
   * @brief Get the forward kinematics of the robotic arm using IK-Geo algorihtm.
   * @param jointPos Joint positions of the robotic arm.
   */
  const std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFKGeo(const std::vector<double>& jointPos);

  /**
   * @brief Bring the getIKTrac function from the parent class.
   */
  using IRoboticArmBase::getIKTrac;

  /**
   * @brief Get the analytical inverse kinematics of the robotic arm using IK-Geo algorihtm.
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @param jointPos Vector of joint positions of the robotic arm.
   * @return Pair of the return code and the next joint positions.
   */
  const bool getIKGeo(const Eigen::Quaterniond& quaternion,
                      const Eigen::Vector3d& position,
                      std::vector<std::vector<double>>& jointPos);

  /**
   * @brief Get the current state of the robotic arm.
   * @return Tuple containing joint positions, velocities, and torques.
   */
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getState() override;

private:
  const std::array<double, 18> UR_H_MATRIX{}; ///< H matrix for the UR robot, size 3*6
  const std::array<double, 21> UR_P_MATRIX{}; ///< P matrix for the UR robot, size 3*7
  ik_geo::Robot* robotGeoSolver_ = nullptr;   ///< IK-Geo solver

  void swapJoints_(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& currentRobotState);
};
