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

#include <memory>
#include <string>
#include <vector>

enum IkType : uint8_t {
  IK_GEO = 0,
  TRACK_IK,
  NB_TYPE // Keep at the end of enum => number of types
};

/**
 * @brief Mother class to create all the prototype functions needed in different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class IRoboticArmBase {
public:
  /**
   * @brief Constructor for IRoboticArmBase.
   */
  IRoboticArmBase(std::string robotName);

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
  std::vector<double> getOriginalHomeJoint() { return originalHomeJoint_; }

  /**
   * @brief Get the number of joints of the robotic arm.
   * @return Number of joints of the robotic arm.
   */
  int getNJoint() { return nJoint_; }

  /**
   * @brief Get the URDF path of the robotic arm.
   *
   */
  std::string getPathUrdf() { return pathUrdf_; }

  /**
   * @brief Get the forward kinematics of the robotic arm.
   * @param jointPositions Joint positions of the robotic arm.
   */
  virtual std::vector<double> getFK(std::vector<double> jointPositions) = 0;

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   * @param ikType Type of inverse kinematics to use.
   * @return Pair of the return code and the next joint positions.
   */
  virtual std::vector<double> getIK(IkType ikType) = 0;

  /**
   * @brief Print the information for this robotic arm.
   */
  void printInfo();

protected:
  /**
   * @brief Initialization function for inverse kinematics.
   */
  int nJoint_ = 0;
  std::string robotName_ = "";
  std::vector<std::string> jointNames_ = {""};
  std::string tipLink_ = "";
  std::string referenceFrame_ = "";
  std::string pathUrdf_ = "";
  std::vector<double> originalHomeJoint_ = {};
};
