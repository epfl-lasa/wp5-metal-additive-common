/**
 * @file RoboticArmUr5.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.2
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "RoboticArmUr5.h"

#include <ros/ros.h>

using namespace std;

const double RoboticArmUr5::TOLERANCE = 1e-5;

RoboticArmUr5::RoboticArmUr5(ROSVersion rosVersion, string configFileName) :
    IRoboticArmBase(string("ur5_robot"), rosVersion, configFileName) {
  robotGeoSolver_ = new ik_geo::Robot(ik_geo::Robot::three_parallel_two_intersecting(UR5_H_MATRIX, UR5_P_MATRIX));
}

RoboticArmUr5::~RoboticArmUr5() { delete robotGeoSolver_; }

const pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmUr5::getFKGeo(const vector<double>& jointPos) {
  // Offset to fix convention between trac-ik (the basic one to use) and ik-geo solvers
  Eigen::Quaterniond offset = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);

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
  quaternion = quaternion * offset;
  return make_pair(move(quaternion), move(posVector));
}

const bool RoboticArmUr5::getIKGeo(const Eigen::Quaterniond& quaternion,
                                   const Eigen::Vector3d& position,
                                   vector<vector<double>>& jointPos) {
  // Offset to fix convention between trac-ik (the basic one to use) and ik-geo solvers
  Eigen::Quaterniond offset = Eigen::Quaterniond(0.5, -0.5, -0.5, -0.5);

  uint totSolutions = 0;
  const uint MAX_REJECTIONS = 90; // percentage of rejected solutions
  double posVector[3] = {position.x(), position.y(), position.z()};

  double rotMatrixArray[9]{};
  Eigen::Matrix3d rotMatrix = (quaternion * offset).toRotationMatrix();
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotMatrixArray, rotMatrix.rows(), rotMatrix.cols()) =
      rotMatrix;

  // Compute IK
  vector<ik_geo::Solution> ikSolutions;
  robotGeoSolver_->ik(rotMatrixArray, posVector, ikSolutions);

  // Check if the IK solver found valid solutions
  jointPos.clear();
  for (const auto& solution : ikSolutions) {
    vector<double> solutionVector(solution.q.begin(), solution.q.end());
    auto [quaternionSolution, positionSolution] = getFKGeo(solutionVector);

    bool isQuatValid = areQuaternionsEquivalent_(quaternion, quaternionSolution);
    bool isPosValid = arePositionsEquivalent_(position, positionSolution);

    if (isQuatValid && isPosValid) {
      jointPos.push_back(solutionVector);
    }
  }

  // Check wether the number of rejected solutions is not too high
  totSolutions = ikSolutions.size();
  if (totSolutions < MAX_REJECTIONS * jointPos.size() / 100) {
    ROS_WARN("Too many solutions were rejected.");
    return false;
  }

  return true;
}

tuple<vector<double>, vector<double>, vector<double>> RoboticArmUr5::getState() {
  tuple<vector<double>, vector<double>, vector<double>> currentRobotState = rosInterface_->getState();
  swapJoints_(currentRobotState);

  return move(currentRobotState);
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

const bool RoboticArmUr5::areQuaternionsEquivalent_(const Eigen::Quaterniond& q1,
                                                    const Eigen::Quaterniond& q2,
                                                    double tolerance) const {
  Eigen::Matrix3d rot1 = q1.toRotationMatrix();
  Eigen::Matrix3d rot2 = q2.toRotationMatrix();

  return (rot1 - rot2).norm() < tolerance;
}

// Function to check if two positions are equivalent
const bool RoboticArmUr5::arePositionsEquivalent_(const Eigen::Vector3d& p1,
                                                  const Eigen::Vector3d& p2,
                                                  double tolerance) const {
  return (p1 - p2).norm() < tolerance;
}