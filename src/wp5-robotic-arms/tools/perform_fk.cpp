/**
 * @file perform_fk.cpp
 * @brief Small piece of code to perform forward kinematics on a robotic arm.
 * It is to get the position and orientation of the end-effector given the joint
 * positions. For experiments purpose.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.1
 * @date 2024-18-09
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */
#include <ros/ros.h>

#include <string>

#include "RoboticArmFactory.h"
#include "debug_tools.h"
#include "yaml_tools.h"

int main(int argc, char** argv) {
  std::string jointSet = "";
  std::string robotName = "";
  std::string rosVersion = "";

  ros::init(argc, argv, "perform_fk");
  ros::NodeHandle nh;

  // Load the robot name and ROS version from the YAML file

  // Get obstacles from the config file
  std::string yamlPath = YamlTools::getYamlPath("fk_joint_config.yaml", std::string(WP5_ROBOTIC_ARMS_DIR));
  YAML::Node config = YAML::LoadFile(yamlPath);

  nh.getParam("jointSet", jointSet);
  nh.getParam("robotName", robotName);
  nh.getParam("rosVersion", rosVersion);

  // Create the robotic arm
  std::unique_ptr<IRoboticArmBase> roboticArm =
      RoboticArmFactory::createRoboticArm(robotName, IRosInterfaceBase::rosVersionsMap.at(rosVersion));

  // Get the joint positions from the command line
  std::vector<double> jointPos = config[robotName][jointSet].as<std::vector<double>>();

  // Perform forward kinematics
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> fkResult = roboticArm->getFKTrac(jointPos);

  // Print the results
  std::cout << "Quaternion: " << DebugTools::getEigenString<Eigen::Quaterniond>(fkResult.first) << std::endl;
  std::cout << "Position: " << DebugTools::getEigenString<Eigen::Vector3d>(fkResult.second) << std::endl;

  ros::shutdown();
}