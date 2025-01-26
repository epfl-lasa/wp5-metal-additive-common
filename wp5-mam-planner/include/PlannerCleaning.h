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
   */
  PlannerCleaning(ROSVersion rosVersion, ros::NodeHandle& nh, std::string robotName, double workingSpeed);

  /**
   * @brief Destructor.
   */
  ~PlannerCleaning() = default;

  /**
   * @brief Plans the welding trajectory of the robot.
   */
  bool planTrajectory(const std::vector<geometry_msgs::Pose>& waypoints);
};