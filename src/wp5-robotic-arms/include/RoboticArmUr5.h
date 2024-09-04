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

#include <memory>
#include <string>
#include <trac_ik/trac_ik.hpp>
#include <vector>

#include "IRoboticArmBase.h"
#include "ik_geo.h"

/**
 * @brief Child class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class RoboticArmUr5 : public IRoboticArmBase {
public:
  explicit RoboticArmUr5();

  /**
   * @brief Get the forward kinematics of the robotic arm.
   * @param jointPositions Joint positions of the robotic arm.
   */
  std::vector<double> getFK(std::vector<double> jointPositions);

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   * @param ikType Type of inverse kinematics to use.
   * @return Pair of the return code and the next joint positions.
   */
  std::vector<double> getIK(IkType ikType);

private:
  std::unique_ptr<TRAC_IK::TRAC_IK> trackIkSolver_ = nullptr; ///< TRAC-IK solver
  std::unique_ptr<ik_geo::Robot> ikGeoSolver_ = nullptr;      ///< IK-Geo solver
};
