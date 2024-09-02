/**
 * @file IRoboticArmBase.h
 * @brief Declaration of the IRoboticArmBase class
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @version 0.1
 * @date 2024-03-07
 * @copyright Copyright (c) 2024 - EPFL
 */

#pragma once
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
// #include <trac_ik/trac_ik.hpp>
#include <vector>

/**
 * @brief Mother class to create all the prototype functions needed in different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class IRoboticArmBase {
public:
  /**
   * @brief Default constructor for IRoboticArmBase.
   */
  IRoboticArmBase() = default;

  /**
   * @brief Destructor for IRoboticArmBase.
   */
  virtual ~IRoboticArmBase() = default;

  /**
   * @brief Get the name of the robotic arm.
   * @return Name of the robotic arm.
   */
  std::string getName() { return robotName_; }

  /**
   * @brief Original home joint positions of the robotic arm.
   */
  std::vector<double> originalHomeJoint = {};

protected:
  /**
   * @brief Initialization function for inverse kinematics.
   */
  std::string robotName_ = "";
  std::vector<std::string> jointNames_;
  std::string baseLink_ = "";
  std::string tipLink_ = "";
  std::string tipJoint_ = "";
  std::string referenceFrame_ = "";
  std::string pathUrdf_ = "";

  std::string paramURDFnJoint_ = "";
  int nJoint_ = 0;

private:
};
