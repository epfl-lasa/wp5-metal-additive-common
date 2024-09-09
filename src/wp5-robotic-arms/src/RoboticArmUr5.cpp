/**
 * @file RoboticArmUr5.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "RoboticArmUr5.h"

#include <ros/ros.h>

using namespace std;

RoboticArmUr5::RoboticArmUr5() : IRoboticArmBase(string("ur5_robot")) {
  robotGeoSolver_ = new ik_geo::Robot(ik_geo::Robot::three_parallel_two_intersecting(UR5_H_MATRIX, UR5_P_MATRIX));
}

RoboticArmUr5::~RoboticArmUr5() { delete robotGeoSolver_; }

pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmUr5::getFK(IkSolver ikSolver, const vector<double>& jointPos) {
  if (ikSolver == IkSolver::TRAC_IK_SOLVER) {
    return IRoboticArmBase::getTracFkSolution_(jointPos);
  } else if (ikSolver == IkSolver::IK_GEO_SOLVER) {
    return getFkGeoSolution_(jointPos);
  } else {
    ROS_ERROR("Invalid forward kinematics solver type");
  }

  return make_pair(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
}

variant<vector<double>, vector<vector<double>>> RoboticArmUr5::getIK(IkSolver ikSolver,
                                                                     const Eigen::Quaterniond& quaternion,
                                                                     const Eigen::Vector3d& position) {
  if (ikSolver == IkSolver::TRAC_IK_SOLVER) {
    return IRoboticArmBase::getTracIkSolution_(quaternion, position);
  } else if (ikSolver == IkSolver::IK_GEO_SOLVER) {
    return getIkGeoSolution_(quaternion, position);
  } else {
    ROS_ERROR("Invalid inverse kinematics solver type");
  }

  return vector<double>();
}

pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmUr5::getFkGeoSolution_(const vector<double>& jointPos) {
  // Arrays to hold the results of the forward kinematics
  array<double, 9> rotArr;
  array<double, 3> posArr;

  // Convert vector to array
  array<double, 6> jointPosArr;
  copy(jointPos.begin(), jointPos.end(), jointPosArr.begin());

  // Compute forward kinematics
  robotGeoSolver_->fk(jointPosArr, rotArr, posArr);

  Eigen::Vector3d posVector = Eigen::Vector3d::Map(posArr.data());
  Eigen::Quaterniond quaternion(Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotArr.data()));

  // Return the position and quaternion
  return make_pair(move(quaternion), move(posVector));
}

vector<vector<double>> RoboticArmUr5::getIkGeoSolution_(const Eigen::Quaterniond& quaternion,
                                                        const Eigen::Vector3d& position) {
  Eigen::Matrix3d rotMatrix = quaternion.toRotationMatrix();
  Eigen::Map<const Eigen::Vector3d> posVector(position.data());

  // Compute IK
  vector<ik_geo::Solution> solutions;
  robotGeoSolver_->ik(rotMatrix.data(), posVector.data(), solutions);

  // Convert the solutions to a vector of vectors
  vector<vector<double>> solutionsVector;
  solutionsVector.reserve(solutions.size()); // Reserve space to avoid multiple allocations

  transform(solutions.begin(), solutions.end(), back_inserter(solutionsVector), [](const ik_geo::Solution& solution) {
    return vector<double>(solution.q.begin(), solution.q.end());
  });

  return move(solutionsVector);
}