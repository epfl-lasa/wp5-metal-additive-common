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

#include <algorithm>
#include <fstream>
#include <iostream>

using namespace std;

const int IRoboticArmBase::NB_JOINTS_ = 6; ///< Number of joints of the robotic arm

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

  if (getNbJoints() != jointNames_.size()) {
    throw runtime_error("Number of joints does not match the number of joint names");
  }

  // Initialize Trac-IK solver
  initializeTracIkSolver_();
}

IRoboticArmBase::~IRoboticArmBase() {
  delete tracIkSolver_;
  delete fkSolver_;
}

pair<Eigen::Quaterniond, Eigen::Vector3d> IRoboticArmBase::getFK(IkSolver ikSolver, const vector<double>& jointPos) {
  if (ikSolver == IkSolver::TRAC_IK_SOLVER) {
    return getTracFkSolution_(jointPos);
  } else {
    ROS_ERROR("Invalid forward kinematics solver type");
  }

  return make_pair(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
}

bool IRoboticArmBase::getIK(IkSolver ikSolver,
                            const Eigen::Quaterniond& quaternion,
                            const Eigen::Vector3d& position,
                            vector<double>& jointPos,
                            const KDL::JntArray& nominal) {
  if (ikSolver == IkSolver::TRAC_IK_SOLVER) {
    return getTracIkSolution_(quaternion, position, jointPos, nominal);
  } else {
    ROS_ERROR("Invalid inverse kinematics solver type");
  }

  return false;
}

void IRoboticArmBase::printInfo() {
  cout << "Robot name: " << robotName_ << endl;
  cout << "URDF Path: " << pathUrdf_ << endl;
  cout << "Number of joints: " << getNbJoints() << endl;

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
  Eigen::Quaterniond quaternion{};
  cartesian_pose.M.GetQuaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

  return make_pair(move(quaternion), move(posVector));
}

bool IRoboticArmBase::getTracIkSolution_(const Eigen::Quaterniond& quaternion,
                                         const Eigen::Vector3d& position,
                                         vector<double>& jointPos,
                                         const KDL::JntArray& nominal) {
  // Initialize computing materials
  KDL::JntArray result{};
  KDL::Frame endEffectorPose{};

  endEffectorPose.p = KDL::Vector(position.x(), position.y(), position.z());
  endEffectorPose.M = KDL::Rotation::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

  // Compute IK
  int isValid = tracIkSolver_->CartToJnt(nominal, endEffectorPose, result);
  jointPos = vector<double>(result.data.data(), result.data.data() + result.data.size());

  return isValid >= 0;
}

void IRoboticArmBase::initializeTracIkSolver_() {
  tracIkSolver_ = new TRAC_IK::TRAC_IK(chainStart_, chainEnd_, pathUrdf_, timeout_, epsilon_, solverType_);

  if (!tracIkSolver_->getKDLChain(chain_)) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  // Set up KDL IK
  fkSolver_ = new KDL::ChainFkSolverPos_recursive(chain_); // Forward kinematics solver
}
