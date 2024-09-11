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

RoboticArmUr5::RoboticArmUr5(ROSVersion rosVersion, string customYamlPath) :
    IRoboticArmBase(string("ur5_robot"), rosVersion, customYamlPath) {
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

bool RoboticArmUr5::getIK(IkSolver ikSolver,
                          const Eigen::Quaterniond& quaternion,
                          const Eigen::Vector3d& position,
                          vector<vector<double>>& jointPos) {
  if (ikSolver == IkSolver::IK_GEO_SOLVER) {
    return getIkGeoSolution_(quaternion, position, jointPos);
  } else {
    ROS_ERROR("Invalid inverse kinematics solver type");
  }

  jointPos.clear();
  return false;
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

tuple<vector<double>, vector<double>, vector<double>> RoboticArmUr5::getState() {
  tuple<vector<double>, vector<double>, vector<double>> currentRobotState = rosInterface_->getState();
  swapJoints_(currentRobotState);

  return move(currentRobotState);
}

bool RoboticArmUr5::getIkGeoSolution_(const Eigen::Quaterniond& quaternion,
                                      const Eigen::Vector3d& position,
                                      vector<vector<double>>& jointPos) {
  double posVector[3] = {position.x(), position.y(), position.z()};

  double rotMatrixArray[9]{};
  Eigen::Matrix3d rotMatrix = quaternion.toRotationMatrix();
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotMatrixArray, rotMatrix.rows(), rotMatrix.cols()) =
      rotMatrix;

  // Compute IK
  vector<ik_geo::Solution> solutions;
  robotGeoSolver_->ik(rotMatrixArray, posVector, solutions);

  jointPos.clear();
  jointPos.reserve(solutions.size()); // Reserve space to avoid multiple allocations

  transform(solutions.begin(), solutions.end(), back_inserter(jointPos), [](const ik_geo::Solution& solution) {
    return vector<double>(solution.q.begin(), solution.q.end());
  });

  return true;
}

void RoboticArmUr5::swapJoints_(tuple<vector<double>, vector<double>, vector<double>>& currentRobotState) {
  // Swap the data 0 to 2 for UR5
  apply(
      [](auto&... vecs) {
        // Lambda body: Expand the tuple into a parameter pack and swap elements
        (..., swap(vecs[0], vecs[2]));
      },
      currentRobotState);
}