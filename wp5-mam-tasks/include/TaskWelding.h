/**
 * @file TaskCleaning.h
 * @brief Declaration of the TaskCleaning class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.3
 * @date 2025-01-31
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

class TaskWelding : public ITaskBase {
public:
  TaskWelding(ros::NodeHandle& nh, std::string configFilename);

  /**
   * @brief Initializes the task.
   * @return True if initialization is successful, false otherwise.
   */
  bool initialize();

  bool computeTrajectory(const std::vector<ROI::Pose>& waypoints);
  bool execute();

private:
  const geometry_msgs::Pose getPoseOffset_(const ROI::Pose waypoint,
                                           const Eigen::Vector3d rotVector,
                                           const Eigen::Vector3d offset);
};
