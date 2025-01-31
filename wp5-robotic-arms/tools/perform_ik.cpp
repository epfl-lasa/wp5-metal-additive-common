/**
 * @file perform_ik.cpp
 * @brief Small piece of code to perform inverse kinematics on a robotic arm.
 * The goal is to verify the feasibility of a given pose.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.1
 * @date 2024-18-09
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 *
 */
#include <ros/ros.h>

#include <string>

#include "RoboticArmFactory.h"
#include "conversion_tools.h"
#include "debug_tools.h"
#include "math_tools.h"
#include "yaml_tools.h"

int main(int argc, char** argv) {
  bool angleDegree = "";
  std::string poseSet = "";
  std::string robotName = "";
  std::string rosVersion = "";

  ros::init(argc, argv, "perform_ik");
  ros::NodeHandle nh;

  // Get obstacles from the config file
  std::string yamlPath = YamlTools::getYamlPath("ik_pose_config.yaml", std::string(WP5_ROBOTIC_ARMS_DIR));
  YAML::Node config = YAML::LoadFile(yamlPath);

  nh.getParam("angleDegree", angleDegree);
  nh.getParam("poseSet", poseSet);
  nh.getParam("robotName", robotName);
  nh.getParam("rosVersion", rosVersion);

  // Create the robotic arm
  std::unique_ptr<IRoboticArmBase> roboticArm =
      RoboticArmFactory::createRoboticArm(robotName, IRosInterfaceBase::rosVersionsMap.at(rosVersion));

  // Get the joint positions from the command line
  std::vector<double> pose = config[robotName][poseSet].as<std::vector<double>>();
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> poseQuatVec = ConversionTools::vectorToEigenQuatPose(pose);

  // TracIK
  std::vector<double> tracIKResult{};
  roboticArm->getIKTrac(poseQuatVec.first, poseQuatVec.second, tracIKResult);

  // Print the results
  if (angleDegree) {
    for (auto& result : tracIKResult) {
      result = MathTools::radToDeg(result);
    }
  }
  std::cout << "[TracIK] -----------" << std::endl;
  std::cout << "Joint Configuration: " << DebugTools::getVecString(tracIKResult) << std::endl;

  // IK Geo
  std::vector<std::vector<double>> geoIKResult{};
  roboticArm->getIKGeo(poseQuatVec.first, poseQuatVec.second, geoIKResult);

  // Print the results
  if (angleDegree) {
    for (auto& result : geoIKResult) {
      for (auto& joint : result) {
        joint = MathTools::radToDeg(joint);
      }
    }
  }
  std::cout << "[IK Geo] -----------" << std::endl;
  for (auto& result : geoIKResult) {
    std::cout << "Joint Configuration: " << DebugTools::getVecString(result) << std::endl;
  }

  ros::shutdown();
}