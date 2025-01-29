/**
 * @file ITaskBase.h
 * @brief Declaration of the ITaskBase class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2025-01-26
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#pragma once

/**
 * @file
 * @brief This file contains the declaration of the ITaskBase class and its associated enums and dependencies.
 */

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "IPlannerBase.h"
#include "IRoboticArmBase.h"
#include "IRosInterfaceBase.h"
#include "ROI.h"
#include "RoboticArmUr.h"
#include "Subtask.h"
#include "math_tools.h"

/**
 * @brief Base class for tasks.
 */
class ITaskBase {
public:
  /**
   * @brief Constructor.
   * @param nh Node handle for ROS.
   * @param config YAML node configuration for the task.
   */
  ITaskBase(ros::NodeHandle& nh, const YAML::Node& config);

  /**
   * @brief Initializes the task.
   * @return True if initialization is successful, false otherwise.
   */
  virtual bool initialize() = 0;

  /**
   * @brief Scans a specified area.
   *
   * This function performs a scan operation over a designated area.
   * The specifics of the scan operation, such as the area to be scanned
   * and the scanning parameters, are determined by the implementation.
   *
   * @return true if the scan operation is successful, false otherwise.
   */
  bool scanArea();

  /**
   * @brief Computes the trajectory based on the given waypoints.
   *
   * This is a pure virtual function that must be implemented by derived classes.
   * It takes a vector of ROI::Pose objects representing the waypoints
   * and computes the corresponding trajectory.
   *
   * @param waypoints A vector of ROI::Pose objects representing the waypoints.
   * @return true if the trajectory computation is successful, false otherwise.
   */
  virtual bool computeTrajectory(const std::vector<ROI::Pose>& waypoints) = 0;

  /**
   * @brief Executes the task.
   * @return True if execution is successful, false otherwise.
   */
  virtual bool execute() = 0;

  /**
   * @brief Moves to the homing configuration.
   * @return True if successful, false otherwise.
   */
  virtual bool goHomingConfiguration();

  /**
   * @brief Moves to the working position.
   * @return True if successful, false otherwise.
   */
  virtual bool goWorkingPosition();

protected:
  ros::NodeHandle nh_{};          ///< ROS node handle.
  const std::string robotName_{}; ///< Robot name.
  const ROSVersion rosVersion_{}; ///< ROS version.

  std::unique_ptr<Subtask> subtask_ = nullptr;      ///< Subtask
  std::unique_ptr<IPlannerBase> planner_ = nullptr; ///< Pointer to MAMPlanner instance.

  const std::vector<double> homeConfig_{};      ///< Home joint configuration.
  const double workingAngle_{};                 ///< Working angle for the task.
  const double workingSpeed_{};                 ///< Working speed for the task.
  const Eigen::Vector3d eePosOffset_{};         ///< End effector pose offset.
  const Eigen::Vector3d eePosWorkOffset_{};     ///< End effector  working pose offset.
  const std::vector<double> eePoseScan_{};      ///< End effector scanning pose.
  const std::vector<double> jointConfigScan_{}; ///< End effector scanning jointconfig.

  const geometry_msgs::Transform transform_; ///< Transform between two frames.

#ifdef DEBUG_MODE
  ros::Publisher pathPub_{nh_.advertise<nav_msgs::Path>("/debug_path", 10)};
#endif

private:
  const geometry_msgs::Transform getTransform_(const std::string& sourceFrame, const std::string& targetFrame);
};
