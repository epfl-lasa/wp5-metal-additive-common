/**
 * @file MAMPlanner.h
 * @brief Declaration of the MAMPlanner class
 *
 * @version 0.2
 * @date 2024-09-12
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ObstaclesManagement.h"
#include "RoboticArmUr.h"

class MAMPlanner {
public:
  /**
   * @brief Constructor.
   */
  MAMPlanner(ROSVersion rosVersion, ros::NodeHandle& nh, std::string robotName);

  /**
   * @brief Destructor.
   */
  ~MAMPlanner() = default;

  /**
   * @brief Plans the trajectory of the robot.
   */
  bool planTrajectory(std::vector<geometry_msgs::Pose> waypoints);

  /**
   * @brief Executes the trajectory of the robot.
   */
  bool executeTrajectory();

  bool goToJointConfig(const std::vector<double>& jointConfig);

  bool goToPose(const geometry_msgs::Pose& targetPose);

private:
  std::unique_ptr<IRoboticArmBase> robot_ = nullptr;         ///< Robotic arm
  std::unique_ptr<ObstaclesManagement> obstacles_ = nullptr; ///< Obstacles management

  ros::NodeHandle nh_;        ///< ROS node handle
  ros::AsyncSpinner spinner_; ///< ROS spinner to handle callbacks asynchronously

  ros::Publisher pubWeldingState_; ///< Publisher for the welding state
  ros::Publisher pubWaypointRviz_; ///< Publisher for the waypoint in Rviz
  ros::Publisher pubTrajectory_;   ///< Publisher for the path

  bool pathFound_ = false;                             ///< Flag indicating if a path is found
  int8_t currentWPointID_ = 0;                         ///< Current waypoint ID
  std::vector<moveit_msgs::RobotTrajectory> bestPlan_; ///< Best plan containing the list of trajectories

  moveit::core::RobotStatePtr robotState_ = nullptr;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup_ = nullptr; ///< MoveGroup interface

  void initMoveit_();
  void setupMovegroup_();
  bool computePath_(const std::vector<double>& startConfig,
                    const geometry_msgs::Pose& targetPose,
                    const bool isWeldging = false);

  bool move_();
};
