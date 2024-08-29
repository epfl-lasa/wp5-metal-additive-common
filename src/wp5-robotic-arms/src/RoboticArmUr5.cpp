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

#include <yaml-cpp/yaml.h>

#include <fstream>

using namespace controllers;
using namespace state_representation;
using namespace std;

RoboticArmUr5::RoboticArmUr5() {
  pathUrdf_ = string(WP5_ROBOTIC_ARMS_DIR) + "/urdf/ur5.urdf";
  robotName_ = "ur5_robot";
  string alternativeYamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/config/arm_robot_config.yaml";
  string yamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/../../config/arm_robot_config.yaml";

  // Check if the alternative YAML file exists
  ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    cout << "Using general YAML file: " << yamlPath << endl;
  } else {
    yamlPath = alternativeYamlPath;
    cout << "Using local YAML file: " << yamlPath << endl;
  }

  // Load parameters from YAML file
  YAML::Node config = YAML::LoadFile(yamlPath);
  YAML::Node robotNode = config[robotName_];

  tipLink_ = robotNode["tipLink"].as<string>();
  tipJoint_ = robotNode["tipJoint"].as<string>();
  baseLink_ = robotNode["reference_frame"].as<string>();
  jointNames_ = robotNode["controller_joint_names"].as<std::vector<std::string>>();
  referenceFrame_ = robotNode["reference_frame"].as<string>();
  nJoint_ = robotNode["numberJoint"].as<int>();
  originalHomeJoint = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
  model_ = make_unique<robot_model::Model>(robotName_, pathUrdf_);

  double damp = 1e-6;
  double alpha = 0.5;
  double gamma = 0.8;
  double margin = 0.07;
  double tolerance = 1e-3;
  unsigned int maxNumberOfIterations = 1000;
  paramsIK_ = {damp, alpha, gamma, margin, tolerance, maxNumberOfIterations};
}
