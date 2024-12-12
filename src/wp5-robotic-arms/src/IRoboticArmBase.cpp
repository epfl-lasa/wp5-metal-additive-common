/**
 * @file IRoboticArmBase.h
 * @brief Declaration of the IRoboticArmBase class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @author [Tristan Bonato] - tristan_bonato@hotmail.com
 * @version 0.2
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#include "IRoboticArmBase.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "RosInterfaceNoetic.h"
#include "debug_tools.h"
#include "yaml_tools.h"

using namespace std;

IRoboticArmBase::IRoboticArmBase(string robotName, ROSVersion rosVersion, const YAML::Node& config) :
    rosVersion_(rosVersion),
    robotName_(robotName),
    contFreq_(YamlTools::loadYamlValue<uint>(config, robotName + "/controller_frequency")),
    pathUrdf_(YamlTools::loadYamlValue<string>(config, robotName + "/path_urdf")),
    jointNames_(YamlTools::loadYamlValue<vector<string>>(config, robotName + "/joint_names")),
    chainStart_(YamlTools::loadYamlValue<string>(config, robotName + "/chain_start")),
    chainEnd_(YamlTools::loadYamlValue<string>(config, robotName + "/chain_end")),
    referenceFrame_(YamlTools::loadYamlValue<string>(config, robotName + "/reference_frame")),
    originalHomeJoint_(YamlTools::loadYamlValue<vector<double>>(config, robotName + "/original_home_joint")) {
  initializeTracIkSolver_();

  if (getNbJoints() != chain_.getNrOfJoints()) {
    throw runtime_error(
        "[IRoboticArmBase] - Number of joints in the kinematic chain does not match the number of joint names");
  }

#ifdef DEBUG_MODE
  printInfo();
#endif

  // Initialize ROS interface
  if (rosVersion_ == ROSVersion::ROS1_NOETIC) {
    rosInterface_ = make_unique<RosInterfaceNoetic>(robotName_);
  }
}

const pair<Eigen::Quaterniond, Eigen::Vector3d> IRoboticArmBase::getFKTrac(const vector<double>& jointPos) {
  KDL::JntArray jointArray(getNbJoints());
  std::copy(jointPos.begin(), jointPos.end(), jointArray.data.data());

  // Perform forward kinematics
  KDL::Frame cartesianPose;
  if (tracFkSolver_->JntToCart(jointArray, cartesianPose) < 0) {
    throw runtime_error("[IRoboticArmBase] - Failed to compute forward kinematics");
  }

  // Extract position and orientation
  Eigen::Vector3d posVector(cartesianPose.p.data);
  Eigen::Quaterniond quaternion{};
  cartesianPose.M.GetQuaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

  return make_pair(quaternion, posVector);
}

const bool IRoboticArmBase::getIKTrac(const Eigen::Quaterniond& quaternion,
                                      const Eigen::Vector3d& position,
                                      std::vector<double>& jointPos,
                                      const std::vector<double>& currentJointPos) {
  bool isValid = false;

  // Ensure that the joint position vector has the correct size
  KDL::JntArray nominalArray(getNbJoints());

  if (!currentJointPos.empty()) {
    if (currentJointPos.size() != getNbJoints()) {
      ROS_ERROR_STREAM("[IRoboticArmBase] - Invalid joint position vector size. It should be " << getNbJoints());
    } else {
      std::copy(currentJointPos.begin(), currentJointPos.end(), nominalArray.data.data());
    }
  }

  // Initialize computing materials
  KDL::JntArray result(getNbJoints());
  KDL::Frame endEffectorPose;

  endEffectorPose.p = KDL::Vector(position.x(), position.y(), position.z());
  endEffectorPose.M = KDL::Rotation::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

  // Compute IK
  isValid = tracIkSolver_->CartToJnt(nominalArray, endEffectorPose, result) >= 0;

  if (isValid) {
    jointPos = std::vector<double>(result.data.data(), result.data.data() + result.data.size());
  } else {
    ROS_ERROR("[IRoboticArmBase] - Failed to compute inverse kinematics");
  }

  return isValid;
}

tuple<vector<double>, vector<double>, vector<double>> IRoboticArmBase::getState() {
  tuple<vector<double>, vector<double>, vector<double>> currentRobotState = rosInterface_->getState();

  return currentRobotState;
}

const bool IRoboticArmBase::isAtJointPosition(const vector<double>& jointPos) {
  vector<double> currentJointPos = get<0>(getState());

#ifdef DEBUG_MODE
  ROS_WARN("[IRoboticArmBase] - Current joint Goal: %s", DebugTools::getVecString<double>(jointPos).c_str());
  ROS_WARN("[IRoboticArmBase] - Current joint position: %s", DebugTools::getVecString<double>(currentJointPos).c_str());
#endif

  return equal(jointPos.begin(), jointPos.end(), currentJointPos.begin(), currentJointPos.end());
}

void IRoboticArmBase::printInfo() const {
  string tmpString = "";
  ROS_INFO("[IRoboticArmBase] ----- Robot Information START -----");

  ROS_INFO_STREAM("[IRoboticArmBase] - Robot name: " << robotName_);
  ROS_INFO_STREAM("[IRoboticArmBase] - URDF Path: " << pathUrdf_);
  ROS_INFO_STREAM("[IRoboticArmBase] - Number of joints: " << getNbJoints());
  ROS_INFO_STREAM("[IRoboticArmBase] - Controller frequency: " << contFreq_ << " Hz");

  tmpString = "[IRoboticArmBase] - Joint names: ";
  ROS_INFO_STREAM(tmpString + DebugTools::getVecString<string>(jointNames_));

  ROS_INFO_STREAM("[IRoboticArmBase] - Chain : " << chainStart_ << " <-> " << chainEnd_);
  ROS_INFO_STREAM("[IRoboticArmBase] - Reference frame: " << referenceFrame_);

  tmpString = "[IRoboticArmBase] - Original home joint: ";
  ROS_INFO_STREAM(tmpString + DebugTools::getVecString<double>(originalHomeJoint_));

  ROS_INFO("[IRoboticArmBase] ----- Robot Information END -----");
}

void IRoboticArmBase::initializeTracIkSolver_() {
  tracIkSolver_ = make_unique<TRAC_IK::TRAC_IK>(chainStart_, chainEnd_, pathUrdf_, timeout_, epsilon_, solverType_);

  if (!tracIkSolver_->getKDLChain(chain_)) {
    ROS_ERROR("[IRoboticArmBase] - There was no valid KDL chain found");
    return;
  }

  // Forward kinematics solver
  tracFkSolver_ = make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
}
