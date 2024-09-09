/**
 * @file MAMPlanner.h
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-03
 */

#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>

#include <memory>

#include "RoboticArmCr7.h"
#include "RoboticArmUr5.h"

class MAMPlanner {
public:
  /**
   * @brief Constructor.
   */
  MAMPlanner();

  /**
   * @brief Plans the trajectory of the robot.
   */
  void planTrajectory();

  /**
   * @brief Executes the trajectory of the robot.
   */
  void executeTrajectory();

private:
  std::unique_ptr<IRoboticArmBase> robot_ = nullptr; ///< Robotic arm
  ros::NodeHandle nh_;                               ///< ROS node handle
  ros::AsyncSpinner spinner_;                        ///< ROS spinner to handle callbacks asynchronously

  ros::Publisher pub_welding_state_;      ///< Publisher for the welding state
  ros::Publisher pub_display_trajectory_; ///< Publisher for the display trajectory

  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_ = nullptr;  ///< Planning scene
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ = nullptr; ///< MoveGroup interface

  void initMoveit_();
  void addStaticObstacles_();
};