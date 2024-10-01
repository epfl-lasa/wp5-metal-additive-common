/**
 * @file IRoboticArmBase.h
 * @brief Declaration of the IRoboticArmBase class
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @version 0.2
 * @date 2024-10-01
 * @copyright Copyright (c) 2024 - EPFL
 */

#pragma once

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
  /**
   * @brief Constructor for IRoboticArmBase.
   */
  IRoboticArmBase(std::string robotName, ROSVersion rosVersion, std::string customYamlPath = "");

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
   * @brief Get the URDF path of the robotic arm.
   *
   */
  const std::string getPathUrdf() const { return pathUrdf_; }

  /**
   * @brief Get the forward kinematics of the robotic arm.
   * @param jointPos Joint positions of the robotic arm.
   */
  const std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFK(const std::vector<double>& jointPos);

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   * @param ikSolver Type of inverse kinematics solver to use.
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @param jointPos Joint positions of the robotic arm.
   * @param nominal (Optional) Nominal joint positions.
   * @return Pair of the return code and the next joint positions.
   */
  const bool getIK(const Eigen::Quaterniond& quaternion,
                   const Eigen::Vector3d& position,
                   std::vector<double>& jointPos,
                   const KDL::JntArray& nominal = KDL::JntArray());

  /**
   * @brief Get the current state of the robotic arm.
   * @return Tuple containing joint positions, velocities, and torques.
   */
  virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getState();

  /**
   * @brief Check if the robotic arm is already in a given joint configuration.
   *
   * @param jointPos Joint positions to check.
   * @return True if the robotic arm is in the given joint configuration, false otherwise.
   */
  const bool isAtJointPosition(const std::vector<double>& jointPos);

  /**
   * @brief Print the information for this robotic arm.
   */
  void printInfo() const;

protected:
  // Attributes
  const ROSVersion rosVersion_ = ROSVersion::VERSION_UNDEFINED;
  std::unique_ptr<IRosInterfaceBase> rosInterface_ = nullptr;

private:
  // Attributes
  const std::string robotName_ = "";            ///< Name of the robotic arm
  const std::string yamlPath_ = "";             ///< Path to the YAML file to use
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
  std::string determineYamlPath_(const std::string& customYamlPath);

  template <typename T>
  T loadYamlValue_(const std::string& robotName, const std::string& key) const {
    YAML::Node robotConfig = YAML::LoadFile(yamlPath_)[robotName];
    return robotConfig[key].as<T>();
  }
};
