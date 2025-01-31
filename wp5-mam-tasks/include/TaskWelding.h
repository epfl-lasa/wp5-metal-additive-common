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

  /**
   * @brief Computes the trajectory to reach the waypoints and perform the task.
   *
   * This function extend the received waypoints to include the pose offsets.
   * It computes other waypoints to have the starting and ending pose for each
   * trajectory segment.
   *
   * @param waypoints The waypoints to reach.
   * @return True if the trajectory is computed successfully, false otherwise.
   */
  bool computeTrajectory(const std::vector<ROI::Pose>& waypoints);

  /**
   * @brief Executes the task.
   * @return True if execution is successful, false otherwise.
   */
  bool execute();

private:
  /**
   * @brief Computes the pose offset and add it to the given waypoint.
   *
   * @param waypoint The waypoint.
   * @param rotVector The rotation vector.
   * @param offset The offset.
   * @return The waypoint offseted.
   */
  const geometry_msgs::Pose getPoseOffset_(const ROI::Pose waypoint,
                                           const Eigen::Vector3d rotVector,
                                           const Eigen::Vector3d offset);
};
