/**
 * @file PlannerCleaning.h
 * @brief Declaration of the PlannerCleaning class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <string>
#include <vector>

#include "IPlannerBase.h"

class PlannerCleaning : public IPlannerBase {
public:
  /**
   * @brief Constructor.
   *
   * @param rosVersion The ROS version used
   * @param nh The ROS node handle
   * @param robotName The name of the robot to plan the trajectory for
   * @param workingSpeed The working speed of the robot when performing the task [m/s]
   */
  PlannerCleaning(ROSVersion rosVersion, ros::NodeHandle& nh, std::string robotName, double workingSpeed);

  /**
   * @brief Destructor.
   */
  ~PlannerCleaning() = default;

protected:
  /**
   * @brief Plans the task trajectory of the robot.
   *
   * @param waypoints The waypoints to plan the trajectory for
   * @return true if the trajectory was planned successfully, false otherwise
   */
  bool planTrajectoryTask_(const std::vector<geometry_msgs::Pose>& waypoints) override;
};