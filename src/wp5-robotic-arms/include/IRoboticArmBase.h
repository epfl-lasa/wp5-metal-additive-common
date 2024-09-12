/**
 * @file IRoboticArmBase.h
 * @brief Declaration of the IRoboticArmBase class
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @version 0.1
 * @date 2024-09-09
 * @copyright Copyright (c) 2024 - EPFL
 */

#pragma once

#include <Eigen/Dense>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <string>
#include <trac_ik/trac_ik.hpp>
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
  ~IRoboticArmBase();

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
  const int getNbJoints() const { return NB_JOINTS_; }

  /**
   * @brief Get the URDF path of the robotic arm.
   *
   */
  const std::string getPathUrdf() const { return pathUrdf_; }

  /**
   * @brief Get the forward kinematics of the robotic arm.
   * @param jointPos Joint positions of the robotic arm.
   */
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFK(const std::vector<double>& jointPos);

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   * @param ikSolver Type of inverse kinematics solver to use.
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @param jointPos Joint positions of the robotic arm.
   * @param nominal (Optional) Nominal joint positions.
   * @return Pair of the return code and the next joint positions.
   */
  bool getIK(const Eigen::Quaterniond& quaternion,
             const Eigen::Vector3d& position,
             std::vector<double>& jointPos,
             const KDL::JntArray& nominal = KDL::JntArray(NB_JOINTS_));

  /**
   * @brief Get the current state of the robotic arm.
   * @return Tuple containing joint positions, velocities, and torques.
   */
  virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getState();

  /**
   * @brief Print the information for this robotic arm.
   */
  void printInfo();

protected:
  // Attributes
  const ROSVersion rosVersion_ = ROSVersion::VERSION_UNDEFINED;
  std::unique_ptr<IRosInterfaceBase> rosInterface_ = nullptr;

private:
  // Attributes
  static const int NB_JOINTS_;
  std::string robotName_ = "";
  std::vector<std::string> jointNames_{};

  std::string chainStart_ = "";
  std::string chainEnd_ = "";
  std::string pathUrdf_ = "";
  std::string referenceFrame_ = "";
  std::vector<double> originalHomeJoint_{};

  // TRAC-IK solver parameters
  double epsilon_ = 1e-5;                             ///< Epsilon for the IK solver
  double timeout_ = 0.01;                             ///< Timeout for the IK solver
  TRAC_IK::SolveType solverType_ = TRAC_IK::Distance; ///< Solve type for the IK solver

  TRAC_IK::TRAC_IK* tracIkSolver_ = nullptr;            ///< TRAC-IK solver
  KDL::ChainFkSolverPos_recursive* fkSolver_ = nullptr; ///< FK solver
  KDL::Chain chain_{};                                  ///< KDL robot kinematic chain

  // Methods
  void initializeTracIkSolver_();
};
