/**
 * @file TaskCleaning.h
 * @brief Declaration of the TaskCleaning class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

#include "IRoboticArmBase.h"
#include "IRosInterfaceBase.h"
#include "ITaskBase.h"
#include "ROI.h"

class TaskCleaning : public ITaskBase {
public:
  /**
   * @brief Constructor of the TaskCleaning class.
   * @param nh The ROS node handle.
   * @param configFilename The path to the configuration file.
   */
  TaskCleaning(ros::NodeHandle& nh, std::string configFilename);

  /**
   * @brief Initializes the task.
   * @return True if initialization is successful, false otherwise.
   */
  bool initialize();

  /**
   * @brief Computes the trajectory to reach the waypoints and perform the task.
   * @param waypoints The waypoints to reach.
   * @return True if the trajectory is computed successfully, false otherwise.
   */
  bool computeTrajectory(const std::vector<ROI::Pose>& waypoints);

  /**
   * @brief Executes the task.
   * @return True if execution is successful, false otherwise.
   */
  bool execute();
};
