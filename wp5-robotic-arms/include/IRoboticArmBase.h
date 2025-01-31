/**
 * @file IRoboticArmBase.h
 * @brief Declaration of the IRoboticArmBase class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @author [Tristan Bonato] - tristan_bonato@hotmail.com
 * @version 0.3
 * @date 2025-01-31
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <string>
#include <trac_ik/trac_ik.hpp>
#include <utility>
#include <variant>
#include <vector>

#include "IRosInterfaceBase.h"

enum IkSolver : uint8_t {
  IK_GEO_SOLVER = 0,
  TRAC_IK_SOLVER,
  NB_TYPE // Keep at the end of enum => number of types
};

/**
 * @brief Mother class to create all the prototype functions needed in different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class IRoboticArmBase {
public:
  // Declare the test class as a friend to allow access to private members
  friend class IRoboticArmBaseTest_TestSwapJoints_Test;
  friend class IRoboticArmBaseTest_TestFilterIKGeoSolutions_Test;

  /**
   * @brief Constructor for IRoboticArmBase.
   *
   * @param robotName Name of the robotic arm.
   * @param rosVersion ROS version.
   * @param config YAML node configuration for the robotic arm.
   */
  IRoboticArmBase(std::string robotName, ROSVersion rosVersion, const YAML::Node& config);

  /**
   * @brief Destructor for IRoboticArmBase.
   */
  ~IRoboticArmBase() = default;

  /**
   * @brief Get the name of the robotic arm.
   * @return Name of the robotic arm.
   */
  const std::string getName() const { return robotName_; }

  /**
   * @brief Original home joint positions of the robotic arm.
   * @return Original home joint positions of the robotic arm.
   */
  const std::vector<double> getOriginalHomeJoint() const { return originalHomeJoint_; }

  /**
   * @brief Get the reference frame of the robotic arm.
   * @return Reference frame of the robotic arm.
   */
  const std::string getReferenceFrame() const { return referenceFrame_; }

  /**
   * @brief Get the number of joints of the robotic arm.
   * @return Number of joints of the robotic arm.
   */
  const size_t getNbJoints() const { return jointNames_.size(); }

  /**
   * @brief Get the control frequency of the robotic arm.
   * @return Control frequency of the robotic arm.
   */
  const uint getCtrlFreq() const { return contFreq_; }

  /**
   * @brief Get the URDF path of the robotic arm.
   * @return URDF path of the robotic arm.
   */
  const std::string getPathUrdf() const { return pathUrdf_; }

  /**
   * @brief Get the forward kinematics of the robotic arm arm using Trac-IK algorithm.
   * @param jointPos Joint positions of the robotic arm.
   */
  const std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFKTrac(const std::vector<double>& jointPos);

  /**
   * @brief Get the forward kinematics of the robotic arm.
   * @param jointPos Joint positions of the robotic arm.
   */
  virtual const std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFKGeo(const std::vector<double>& jointPos) {
    ROS_ERROR("[IRoboticArmBase] - This function is not implemented for this robotic arm.");
    return std::make_pair(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  }

  /**
   * @brief Get the inverse kinematics of the robotic arm using Trac-IK algorithm.
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @param jointPos Joint positions of the robotic arm.
   * @param currentJointPos Current joint positions of the robotic arm.
   * @return True if the IK was successful, false otherwise.
   */
  const bool getIKTrac(const Eigen::Quaterniond& quaternion,
                       const Eigen::Vector3d& position,
                       std::vector<double>& jointPos,
                       const std::vector<double>& currentJointPos = {});

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @param jointPos Vector of joint positions of the robotic arm.
   * @param minJointMovements Flag to minimize joint movements from the current joint positions.
   * @return Pair of the return code and the next joint positions.
   */
  virtual const bool getIKGeo(const Eigen::Quaterniond& quaternion,
                              const Eigen::Vector3d& position,
                              std::vector<std::vector<double>>& jointPos,
                              const bool minJointMovements = true) {
    ROS_ERROR("[IRoboticArmBase] - This function is not implemented for this robotic arm.");

    jointPos.clear();
    return false;
  }

  /**
   * @brief Get the current state of the robotic arm.
   * @return Tuple containing joint positions, velocities, and torques.
   */
  virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getState();

  /**
   * @brief Check if the robotic arm is already in a given joint configuration.
   *
   * @param jointPos Joint positions to check.
   * @param tolerance Tolerance on each joints for the comparison.
   * @return True if the robotic arm is in the given joint configuration, false otherwise.
   */
  const bool isAtJointPosition(const std::vector<double>& jointPos, double tolerance = 1e-3);

  /**
   * @brief Print the information for this robotic arm.
   */
  void printInfo() const;

protected:
  // Attributes
  const ROSVersion rosVersion_ = ROSVersion::VERSION_UNDEFINED;
  std::unique_ptr<IRosInterfaceBase> rosInterface_ = nullptr;

  virtual void swapJoints_(
      std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& currentRobotState) {
    ROS_ERROR("[IRoboticArmBase] - This function is not implemented for this robotic arm.");
  }

  virtual void filterIKGeoSolutions_(std::vector<std::vector<double>>& jointPos,
                                     const Eigen::Quaterniond& quaternion,
                                     const Eigen::Vector3d& position) {
    ROS_ERROR("[IRoboticArmBase] - This function is not implemented for this robotic arm.");
  }

private:
  // Attributes
  const std::string robotName_ = "";            ///< Name of the robotic arm
  const std::string yamlPath_ = "";             ///< Path to the YAML file to use
  const uint contFreq_ = 0;                     ///< Control frequency of the robotic arm
  const std::string pathUrdf_ = "";             ///< Path to the URDF file to use
  const std::vector<std::string> jointNames_{}; ///< Names of the joints of the robotic arm

  const std::string chainStart_ = "";             ///< Start of the robotic arm's kinematic chain
  const std::string chainEnd_ = "";               ///< End of the robotic arm's kinematic chain
  const std::string referenceFrame_ = "";         ///< Reference frame of the robotic arm
  const std::vector<double> originalHomeJoint_{}; ///< Original home joint positions of the robotic arm (in radians)

  // TRAC-IK solver parameters
  const double epsilon_ = 1e-5;                             ///< Epsilon for the IK solver
  const double timeout_ = 0.01;                             ///< Timeout for the IK solver
  const TRAC_IK::SolveType solverType_ = TRAC_IK::Distance; ///< Solve type for the IK solver

  std::unique_ptr<TRAC_IK::TRAC_IK> tracIkSolver_ = nullptr;                ///< TRAC-IK solver
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> tracFkSolver_ = nullptr; ///< FK solver
  KDL::Chain chain_{};                                                      ///< KDL robot kinematic chain

  // Methods
  void initializeTracIkSolver_();
};
