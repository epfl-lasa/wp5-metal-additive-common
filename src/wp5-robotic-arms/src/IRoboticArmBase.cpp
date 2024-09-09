/**
 * @file IRoboticArmBase.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#include "IRoboticArmBase.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

using namespace std;

IRoboticArmBase::IRoboticArmBase(string robotName, string customYamlPath) : robotName_(robotName) {
  string yamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/../../config/arm_robot_config.yaml";

  if (!customYamlPath.empty()) {
    yamlPath = customYamlPath;
    cout << "Using custom YAML file: " << yamlPath << endl;
  }

  ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    cout << "Using general YAML file: " << yamlPath << endl;
  } else {
    yamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/config/arm_robot_config.yaml";
    cout << "Using local YAML file: " << yamlPath << endl;
  }

  // Load parameters from YAML file
  YAML::Node robotConfig = YAML::LoadFile(yamlPath)[robotName_];

  chainStart_ = robotConfig["chain_start"].as<string>();
  chainEnd_ = robotConfig["chain_end"].as<string>();

  pathUrdf_ = robotConfig["path_urdf"].as<string>();
  jointNames_ = robotConfig["joint_names"].as<vector<string>>();
  originalHomeJoint_ = robotConfig["original_home_joint"].as<vector<double>>();
  referenceFrame_ = robotConfig["reference_frame"].as<string>();

  nJoint_ = jointNames_.size();

  // Initialize Trac-IK solver
  initializeTracIkSolver_();
}

IRoboticArmBase::~IRoboticArmBase() {
  delete tracIkSolver_;
  delete fkSolver_;
}

void IRoboticArmBase::printInfo() {
  cout << "Robot name: " << robotName_ << endl;
  cout << "URDF Path: " << pathUrdf_ << endl;
  cout << "Number of joints: " << nJoint_ << endl;

  cout << "Joint names: ";
  for (const string& jointName : jointNames_) {
    cout << jointName << " ";
  }
  cout << endl;

  cout << "Reference frame: " << referenceFrame_ << endl;

  cout << "Original home joint: ";
  for (double joint : originalHomeJoint_) {
    cout << joint << " ";
  }
  cout << endl;
}

void IRoboticArmBase::initializeTracIkSolver_() {
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

pair<Eigen::Quaterniond, Eigen::Vector3d> IRoboticArmBase::getTracFkSolution_(const vector<double>& jointPos) {
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

vector<double> IRoboticArmBase::getTracIkSolution_(const Eigen::Quaterniond& quaternion,
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
