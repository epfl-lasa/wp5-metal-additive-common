/**
 * @file IRoboticArmBase.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.2
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "IRoboticArmBase.h"

#include <ros/ros.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "RosInterfaceNoetic.h"
#include "wp5_common_utilities/YamlTools.h"

using namespace std;

IRoboticArmBase::IRoboticArmBase(string robotName, ROSVersion rosVersion, string configFileName) :
    rosVersion_(rosVersion),
    robotName_(robotName),
    yamlPath_(YamlTools::getYamlPath_(configFileName, string(WP5_ROBOTIC_ARMS_DIR))),
    pathUrdf_(YamlTools::loadYamlValue_<string>(yamlPath_, robotName, "path_urdf")),
    jointNames_(YamlTools::loadYamlValue_<vector<string>>(yamlPath_, robotName, "joint_names")),
    chainStart_(YamlTools::loadYamlValue_<string>(yamlPath_, robotName, "chain_start")),
    chainEnd_(YamlTools::loadYamlValue_<string>(yamlPath_, robotName, "chain_end")),
    referenceFrame_(YamlTools::loadYamlValue_<string>(yamlPath_, robotName, "reference_frame")),
    originalHomeJoint_(YamlTools::loadYamlValue_<vector<double>>(yamlPath_, robotName, "original_home_joint")) {
  // Initialize Trac-IK solver
  initializeTracIkSolver_();

  if (getNbJoints() != chain_.getNrOfJoints()) {
    throw runtime_error("Number of joints in the kinematic chain does not match the number of joint names");
  }

  // Initialize ROS interface
  if (rosVersion_ == ROSVersion::ROS1_NOETIC) {
    rosInterface_ = make_unique<RosInterfaceNoetic>(robotName_);
  }
}

const pair<Eigen::Quaterniond, Eigen::Vector3d> IRoboticArmBase::getFK(const vector<double>& jointPos) {
  KDL::JntArray jointArray(getNbJoints());
  for (size_t i = 0; i < getNbJoints(); ++i) {
    jointArray(i) = jointPos[i];
  }

  // Perform forward kinematics
  KDL::Frame cartesianPose;
  if (tracFkSolver_->JntToCart(jointArray, cartesianPose) < 0) {
    throw runtime_error("Failed to compute forward kinematics");
  }

  // Extract position and orientation
  Eigen::Vector3d posVector(cartesianPose.p.data);
  Eigen::Quaterniond quaternion{};
  cartesianPose.M.GetQuaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

  return make_pair(move(quaternion), move(posVector));
}

const bool IRoboticArmBase::getIK(const Eigen::Quaterniond& quaternion,
                                  const Eigen::Vector3d& position,
                                  vector<double>& jointPos,
                                  const KDL::JntArray& nominal) {
  // Ensure that the joint position vector has the correct size
  KDL::JntArray nominalArray = nominal;
  if (nominalArray.rows() != getNbJoints()) {
    nominalArray = KDL::JntArray(getNbJoints());
  }

  // Initialize computing materials
  KDL::JntArray result{};
  KDL::Frame endEffectorPose{};

  endEffectorPose.p = KDL::Vector(position.x(), position.y(), position.z());
  endEffectorPose.M = KDL::Rotation::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

  // Compute IK
  int isValid = tracIkSolver_->CartToJnt(nominalArray, endEffectorPose, result);
  jointPos = vector<double>(result.data.data(), result.data.data() + result.data.size());

  return isValid >= 0;
}

tuple<vector<double>, vector<double>, vector<double>> IRoboticArmBase::getState() {
  tuple<vector<double>, vector<double>, vector<double>> currentRobotState = rosInterface_->getState();

  return currentRobotState;
}

const bool IRoboticArmBase::isAtJointPosition(const vector<double>& jointPos) {
  vector<double> currentJointPos = get<0>(getState());

  return equal(jointPos.begin(), jointPos.end(), currentJointPos.begin(), currentJointPos.end());
}

void IRoboticArmBase::printInfo() const {
  string tmpString = "";
  ROS_INFO_STREAM("Robot name: " << robotName_);
  ROS_INFO_STREAM("URDF Path: " << pathUrdf_);
  ROS_INFO_STREAM("Number of joints: " << getNbJoints());

  tmpString = "Joint names: [";
  for (const string& jointName : jointNames_) {
    tmpString += jointName + ", ";
  }
  ROS_INFO_STREAM(tmpString << "]");
  ROS_INFO_STREAM("Reference frame: " << referenceFrame_);

  tmpString = "Original home joint: [";
  for (double joint : originalHomeJoint_) {
    tmpString += to_string(joint) + ", ";
  }
  ROS_INFO_STREAM(tmpString << "]");
}

void IRoboticArmBase::initializeTracIkSolver_() {
  tracIkSolver_ = make_unique<TRAC_IK::TRAC_IK>(chainStart_, chainEnd_, pathUrdf_, timeout_, epsilon_, solverType_);

  if (!tracIkSolver_->getKDLChain(chain_)) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  // Forward kinematics solver
  tracFkSolver_ = make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
}
