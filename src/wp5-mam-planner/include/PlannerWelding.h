/**
 * @file PlannerWelding.h
 * @brief Declaration of the PlannerWelding class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <string>
#include <vector>

#include "IPlannerBase.h"

class PlannerWelding : public IPlannerBase {
public:
  /**
   * @brief Constructor.
   */
  PlannerWelding(ROSVersion rosVersion, ros::NodeHandle& nh, std::string robotName);

  /**
   * @brief Destructor.
   */
  ~PlannerWelding() = default;

  /**
   * @brief Plans the welding trajectory of the robot.
   */
  bool planTrajectory(std::vector<geometry_msgs::Pose> waypoints);

private:
  bool computePath_(const std::vector<double>& startConfig,
                    const geometry_msgs::Pose& targetPose,
                    const bool isWeldging = false);
};