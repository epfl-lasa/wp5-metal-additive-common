/**
 * @file PlannerWelding.h
 * @brief Declaration of the PlannerWelding class
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

class PlannerWelding : public IPlannerBase {
public:
  /**
   * @brief Constructor.
   */
  PlannerWelding(ROSVersion rosVersion, ros::NodeHandle& nh, std::string robotName, double workingSpeed);

  /**
   * @brief Destructor.
   */
  ~PlannerWelding() = default;

  /**
   * @brief Plans the welding trajectory of the robot.
   */
  bool planTrajectory(const std::vector<geometry_msgs::Pose>& waypoints);

private:
  bool computeWeldingPossiblePaths_(const geometry_msgs::Pose& startPose, const geometry_msgs::Pose& targetPose);

  bool computeTransitionPath_(const moveit_msgs::RobotTrajectory& trajectory,
                              const geometry_msgs::Pose& targetWaypoint,
                              const MotionDir direction,
                              const ConfigPosition position);
};