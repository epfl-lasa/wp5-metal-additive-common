/**
 * @file RoboticArmUr.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.2
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "RoboticArmUr.h"

#include <ros/ros.h>

#include "IRosInterfaceBase.h"
#include "math_tools.h"
#include "yaml_tools.h"

using namespace std;

const double RoboticArmUr::TOLERANCE = 1e-5;

RoboticArmUr::RoboticArmUr(ROSVersion rosVersion, string robotName, string configFilename) :
    IRoboticArmBase(string(robotName),
                    rosVersion,
                    YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_ROBOTIC_ARMS_DIR)))) {}

RoboticArmUr::RoboticArmUr(ROSVersion rosVersion,
                           string robotName,
                           string configFilename,
                           const std::array<double, 18>& hMatrix,
                           const std::array<double, 21>& pMatrix) :
    IRoboticArmBase(string(robotName),
                    rosVersion,
                    YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_ROBOTIC_ARMS_DIR)))),
    UR_H_MATRIX(hMatrix),
    UR_P_MATRIX(pMatrix) {
  robotGeoSolver_ = new ik_geo::Robot(ik_geo::Robot::three_parallel_two_intersecting(UR_H_MATRIX.data(), UR_P_MATRIX.data()));
}

RoboticArmUr::~RoboticArmUr() { delete robotGeoSolver_; }

const pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmUr::getFKGeo(const vector<double>& jointPos) {
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

const bool RoboticArmUr::getIKGeo(const Eigen::Quaterniond& quaternion,
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

    bool isQuatValid = MathTools::areQuatEquivalent(quaternion, quaternionSolution);
    bool isPosValid = MathTools::arePosEquivalent(position, positionSolution);

    if (isQuatValid && isPosValid) {
      jointPos.push_back(solutionVector);
    }
  }

  // Check wether the number of rejected solutions is not too high
  totSolutions = ikSolutions.size();
  if (totSolutions < MAX_REJECTIONS * jointPos.size() / 100) {
    ROS_WARN("[IRoboticArmUR] - Too many solutions were rejected.");
    return false;
  }

  return true;
}

tuple<vector<double>, vector<double>, vector<double>> RoboticArmUr::getState() {
  tuple<vector<double>, vector<double>, vector<double>> currentRobotState = rosInterface_->getState();
  swapJoints_(currentRobotState);

  return move(currentRobotState);
}

void RoboticArmUr::swapJoints_(tuple<vector<double>, vector<double>, vector<double>>& currentRobotState) {
  // Swap the data 0 to 2 for UR5
  apply(
      [](auto&... vecs) {
        // Lambda body: Expand the tuple into a parameter pack and swap elements
        (..., swap(vecs[0], vecs[2]));
      },
      currentRobotState);
}
