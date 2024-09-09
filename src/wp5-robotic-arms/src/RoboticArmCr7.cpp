/**
 * @file RoboticArmCr7.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "RoboticArmCr7.h"

#include <ros/ros.h>

#include <iostream>

using namespace std;

RoboticArmCr7::RoboticArmCr7() : IRoboticArmBase(string("xMateCR7")) {}

pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmCr7::getFK(IkSolver ikSolver, const vector<double>& jointPos) {
  if (ikSolver == IkSolver::TRAC_IK_SOLVER) {
    return IRoboticArmBase::getTracFkSolution_(jointPos);
  } else {
    ROS_ERROR("Invalid forward kinematics solver type");
  }

  return make_pair(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
}

variant<vector<double>, vector<vector<double>>> RoboticArmCr7::getIK(IkSolver ikSolver,
                                                                     const Eigen::Quaterniond& quaternion,
                                                                     const Eigen::Vector3d& position) {
  if (ikSolver == IkSolver::TRAC_IK_SOLVER) {
    return IRoboticArmBase::getTracIkSolution_(quaternion, position);
  } else {
    ROS_ERROR("Invalid inverse kinematics solver type");
  }

  return vector<double>();
}
