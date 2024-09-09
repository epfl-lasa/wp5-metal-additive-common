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
  chainStart_ = "base_link_inertia";
  chainEnd_ = "virtual_link";

  initializeTracIkSolver_();

  robotGeoSolver_ = new ik_geo::Robot(ik_geo::Robot::three_parallel_two_intersecting(UR5_H_MATRIX, UR5_P_MATRIX));
}

RoboticArmUr5::~RoboticArmUr5() {
  delete tracIkSolver_;
  delete fkSolver_;
  delete robotGeoSolver_;
}

pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmUr5::getFK(IkSolver ikSolver, const vector<double>& jointPos) {
  if (ikSolver == IkSolver::TRAC_IK_SOLVER) {
    return getTracFkSolution_(jointPos);
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
    return getTracIkSolution_(quaternion, position);
  } else if (ikSolver == IkSolver::IK_GEO_SOLVER) {
    return getIkGeoSolution_(quaternion, position);
  } else {
    ROS_ERROR("Invalid inverse kinematics solver type");
  }

  return vector<double>();
}

void RoboticArmUr5::initializeTracIkSolver_() {
  tracIkSolver_ = new TRAC_IK::TRAC_IK(chainStart_, chainEnd_, pathUrdf_, timeout_, epsilon_, solverType_);

  if (!tracIkSolver_->getKDLChain(chain_)) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  if (!tracIkSolver_->getKDLLimits(ll_, ul_)) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain_.getNrOfJoints() == ll_.data.size());
  assert(chain_.getNrOfJoints() == ul_.data.size());

  ROS_INFO("Using %d joints", chain_.getNrOfJoints());

  // Set up KDL IK
  fkSolver_ = new KDL::ChainFkSolverPos_recursive(chain_); // Forward kinematics solver
}

pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmUr5::getTracFkSolution_(const vector<double>& jointPos) {
  KDL::JntArray joint_array(chain_.getNrOfJoints());
  for (size_t i = 0; i < jointPos.size(); ++i) {
    joint_array(i) = jointPos[i];
  }

  // Perform forward kinematics
  KDL::Frame cartesian_pose;
  if (fkSolver_->JntToCart(joint_array, cartesian_pose) < 0) {
    throw runtime_error("Failed to compute forward kinematics");
  }

  // Extract position and orientation
  Eigen::Vector3d posVector(cartesian_pose.p.data);
  Eigen::Quaterniond quaternion;
  cartesian_pose.M.GetQuaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

  return make_pair(move(quaternion), move(posVector));
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

vector<double> RoboticArmUr5::getTracIkSolution_(const Eigen::Quaterniond& quaternion,
                                                 const Eigen::Vector3d& position) {
  // Initialize computing materials
  KDL::JntArray result;
  KDL::Frame endEffectorPose;
  KDL::JntArray nominal(nJoint_);
  nominal.data.setZero();

  endEffectorPose.p = KDL::Vector(position.x(), position.y(), position.z());
  endEffectorPose.M = KDL::Rotation::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

  // Compute IK
  tracIkSolver_->CartToJnt(nominal, endEffectorPose, result);
  vector<double> result_vector(result.data.data(), result.data.data() + result.data.size());

  return move(result_vector);
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