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

#include <kdl/frames.hpp>

using namespace std;

RoboticArmUr5::RoboticArmUr5() : IRoboticArmBase(string("ur5_robot")) {
  TRAC_IK::SolveType solverType = TRAC_IK::Distance;
  tracIkSolver_ = new TRAC_IK::TRAC_IK("base_link_inertia", "virtual_link", pathUrdf_, 0.01, 0.05, solverType);

  ik_geo::Robot* ikGeoSolver_ =
      new ik_geo::Robot(ik_geo::Robot::three_parallel_two_intersecting(UR5_H_MATRIX, UR5_P_MATRIX));
}

RoboticArmUr5::~RoboticArmUr5() {
  cout << "Destroying the UR5 robotic arm" << endl;
  delete tracIkSolver_;
  delete ikGeoSolver_;
}

vector<double> RoboticArmUr5::getFK(vector<double> jointPositions) {
  //TODO(lmunier): Implement the forward kinematics of the UR5 robotic arm
  cout << "Forward kinematics of the UR5 robotic arm" << endl;
}

variant<vector<double>, vector<vector<double>>> RoboticArmUr5::getIK(IkSolver ikSolver,
                                                                     Eigen::Quaterniond quaternion,
                                                                     Eigen::Vector3d position) {
  if (ikSolver == IkSolver::IK_GEO_SOLVER) {
    return getIkGeoSolution_(quaternion, position);
  } else if (ikSolver == IkSolver::TRAC_IK_SOLVER) {
    return getTracIkSolution_(quaternion, position);
  } else {
    ROS_ERROR("Invalid IK type");
  }
}

vector<double> RoboticArmUr5::getTracIkSolution_(Eigen::Quaterniond quaternion, Eigen::Vector3d position) {
  // Initialize computing materials
  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  KDL::JntArray nominal(nJoint_);
  for (uint j = 0; j < nominal.data.size(); j++) {
    nominal(j) = 0.0;
  }

  end_effector_pose.p = KDL::Vector(position.x(), position.y(), position.z());
  end_effector_pose.M = KDL::Rotation::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

  // Compute IK
  tracIkSolver_->CartToJnt(nominal, end_effector_pose, result);

  // From result to vector
  vector<double> result_vector;
  for (uint j = 0; j < result.data.size(); j++) {
    result_vector.push_back(result(j));
  }

  return result_vector;
}

vector<vector<double>> RoboticArmUr5::getIkGeoSolution_(Eigen::Quaterniond quaternion, Eigen::Vector3d position) {
  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

  // Convert Eigen::Matrix3d to flat double array
  double rot_matrix[9];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rot_matrix[i * 3 + j] = rotation_matrix(i, j);
    }
  }

  // Convert Eigen::Vector3d to flat double array
  double pos_vector[3] = {position.x(), position.y(), position.z()};

  // Call the IK solver
  vector<ik_geo::Solution> solutions = {};
  ikGeoSolver_->ik(rot_matrix, pos_vector, solutions);

  // Convert the solutions to a vector of vectors
  vector<vector<double>> solutions_vector;
  for (auto& solution : solutions) {
    vector<double> solution_vector;

    for (auto& q : solution.q) {
      solution_vector.push_back(q);
    }
    solutions_vector.push_back(solution_vector);
  }

  return solutions_vector;
}