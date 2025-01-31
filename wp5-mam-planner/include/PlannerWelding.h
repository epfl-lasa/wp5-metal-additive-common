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
   *
   * @param rosVersion The ROS version used
   * @param nh The ROS node handle
   * @param robotName The name of the robot to plan the trajectory for
   * @param workingSpeed The working speed of the robot when performing the task [m/s]
   */
  PlannerWelding(ROSVersion rosVersion, ros::NodeHandle& nh, std::string robotName, double workingSpeed);

  /**
   * @brief Destructor.
   */
  ~PlannerWelding() = default;

protected:
  /**
   * @brief Plans the task trajectory of the robot.
   *
   * @param waypoints The waypoints to plan the trajectory for
   * @return true if the trajectory was planned successfully, false otherwise
   */
  bool planTrajectoryTask_(const std::vector<geometry_msgs::Pose>& waypoints) override;

private:
  /**
   * @brief Compute possible welding paths between the start and target poses
   *
   * This ensure having checked all the possible solutions before planning the transition paths.
   * The objective is to go through all the possible configurations to find the one that minimizes
   * the total jerk and meets the constraints.
   *
   * @param startPose The starting pose for the welding path
   * @param targetPose The target pose for the welding path
   * @return true if the welding paths were computed successfully, false otherwise
   */
  bool computeWeldingPossiblePaths_(const geometry_msgs::Pose& startPose, const geometry_msgs::Pose& targetPose);

  /**
   * @brief Compute the transition path (engaging / disengaging robot) from the given trajectory to the target waypoint
   *
   * @param trajectory The current welding trajectory to transition from
   * @param targetWaypoint The target waypoint for the transition
   * @param direction The direction of the motion
   * @param position The position in the trajectory to extract the second joint configuration from
   * @return true if the transition path was computed successfully, false otherwise
   */
  bool computeTransitionPath_(const moveit_msgs::RobotTrajectory& trajectory,
                              const geometry_msgs::Pose& targetWaypoint,
                              const MotionDir direction,
                              const ConfigPosition position);
};