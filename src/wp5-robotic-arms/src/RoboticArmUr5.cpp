/**
 * @file RoboticArmUr5.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
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

pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmUr5::getFK(IkSolver ikSolver, vector<double> joint_pos) {
  if (ikSolver == IkSolver::TRAC_IK_SOLVER) {
    return getTracFkSolution_(joint_pos);
  } else if (ikSolver == IkSolver::IK_GEO_SOLVER) {
    return getFkGeoSolution_(joint_pos);
  } else {
    ROS_ERROR("Invalid forward kinematics solver type");
  }

  return make_pair(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
}

variant<vector<double>, vector<vector<double>>> RoboticArmUr5::getIK(IkSolver ikSolver,
                                                                     Eigen::Quaterniond quaternion,
                                                                     Eigen::Vector3d position) {
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

pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmUr5::getTracFkSolution_(vector<double> joint_pos) {
  KDL::JntArray joint_array(chain_.getNrOfJoints());
  for (size_t i = 0; i < joint_pos.size(); ++i) {
    joint_array(i) = joint_pos[i];
  }

  // Perform forward kinematics
  KDL::Frame cartesian_pose;
  if (fkSolver_->JntToCart(joint_array, cartesian_pose) < 0) {
    throw runtime_error("Failed to compute forward kinematics");
  }

  // Extract position and orientation
  Eigen::Vector3d position(cartesian_pose.p.data);
  Eigen::Quaterniond orientation;
  cartesian_pose.M.GetQuaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w());

  return make_pair(orientation, position);
}

pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmUr5::getFkGeoSolution_(vector<double> joint_pos) {
  // Arrays to hold the results of the forward kinematics
  array<double, 9> rotation_matrix;
  array<double, 3> position_vector;

  // Convert vector to array
  array<double, 6> joint_pos_arr;
  copy(joint_pos.begin(), joint_pos.end(), joint_pos_arr.begin());

  // Compute forward kinematics
  robotGeoSolver_->fk(joint_pos_arr, rotation_matrix, position_vector);
  cout << "Position: " << position_vector[0] << " " << position_vector[1] << " " << position_vector[2] << endl;
  Eigen::Vector3d pos_vector = Eigen::Vector3d::Map(position_vector.data());
  Eigen::Quaterniond quaternion(Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation_matrix.data()));

  // Return the position and quaternion
  return make_pair(quaternion, pos_vector);
}

vector<double> RoboticArmUr5::getTracIkSolution_(Eigen::Quaterniond quaternion, Eigen::Vector3d position) {
  // Initialize computing materials
  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  KDL::JntArray nominal(nJoint_);
  nominal.data.setZero();

  end_effector_pose.p = KDL::Vector(position.x(), position.y(), position.z());
  end_effector_pose.M = KDL::Rotation::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

  // Compute IK
  tracIkSolver_->CartToJnt(nominal, end_effector_pose, result);

  return vector<double>(result.data.data(), result.data.data() + result.data.size());
}

vector<vector<double>> RoboticArmUr5::getIkGeoSolution_(Eigen::Quaterniond quaternion, Eigen::Vector3d position) {
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rot_matrix(quaternion.toRotationMatrix().data());
  Eigen::Map<Eigen::Vector3d> pos_vector(position.data());

  // Compute IK
  vector<ik_geo::Solution> solutions;
  robotGeoSolver_->ik(rot_matrix.data(), pos_vector.data(), solutions);

  // Convert the solutions to a vector of vectors
  vector<vector<double>> solutions_vector;
  solutions_vector.reserve(solutions.size()); // Reserve space to avoid multiple allocations

  transform(solutions.begin(), solutions.end(), back_inserter(solutions_vector), [](const ik_geo::Solution& solution) {
    return vector<double>(solution.q.begin(), solution.q.end());
  });

  return solutions_vector;
}