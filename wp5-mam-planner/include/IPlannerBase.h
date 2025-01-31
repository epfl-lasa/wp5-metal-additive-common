/**
 * @file IPlannerBase.h
 * @brief Declaration of the IPlannerBase class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.3
 * @date 2025-01-31
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
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

class IPlannerBase {
protected:
  enum MotionDir : int8_t { FORWARD = 1, BACKWARD = -1 };
  enum ConfigPosition : int8_t { FIRST, LAST };

public:
  /**
   * @brief Constructor.
   */
  IPlannerBase(ROSVersion rosVersion, ros::NodeHandle& nh, std::string robotName, double workingSpeed);

  /**
   * @brief Destructor.
   */
  virtual ~IPlannerBase() = default;

  /**
   * @brief Plans the trajectory of the robot.
   */
  bool planTrajectory(const std::vector<geometry_msgs::Pose>& waypoints);

  /**
   * @brief Executes the trajectory of the robot.
   */
  bool executeTrajectory();

  bool planCartesianFromJointConfig(const std::vector<double>& startJointConfig,
                                    const std::vector<geometry_msgs::Pose>& waypoints,
                                    std::vector<moveit_msgs::RobotTrajectory>& pathPlanned);

  bool goToScanArea(const std::vector<double>& eePoseScan);

  bool goToJointConfig(const std::vector<double>& jointConfig);

  bool goToPose(const geometry_msgs::Pose& targetPose);

protected:
  std::unique_ptr<IRoboticArmBase> robot_ = nullptr;         ///< Robotic arm
  std::unique_ptr<ObstaclesManagement> obstacles_ = nullptr; ///< Obstacles management

  ros::NodeHandle nh_;             ///< ROS node handle
  ros::ServiceClient laserClient_; ///< Client for the welding laser service

  std::string trajectoriesStrat_ = "";     ///< Strategy for the trajectories [store, read, plan]
  std::string trajectoriesDirectory_ = ""; ///< Directory for the trajectories
  std::string trajectoriesFilename_ = "";  ///< Filename for the trajectories

#ifdef DEBUG_MODE
  ros::Publisher pubTrajectory_; ///< Publisher for the trajectory
#endif

  bool pathFound_ = false;                                       ///< Flag indicating if a path is found
  int currentWPointID_ = 0;                                      ///< Current waypoint ID
  double workingSpeed_ = 0.0;                                    ///< Working speed
  std::vector<moveit_msgs::RobotTrajectory> sortedWeldingPaths_; ///< Sorted list of possible welding paths

  // List of trajectories to be executed, with task (welding / cleaning) to be activated
  std::vector<std::pair<moveit_msgs::RobotTrajectory, bool>> trajTaskToExecute_;

  moveit::core::RobotStatePtr robotState_ = nullptr;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup_ = nullptr; ///< MoveGroup interface

  // void adaptConfigToLimitMoves_(std::vector<double>& jointConfig);
  bool saveTrajectory_(const moveit_msgs::RobotTrajectory& trajectory, const std::string& filename);
  bool loadTrajectory_(moveit_msgs::RobotTrajectory& trajectory, const std::string& filename);
  bool loadAllTrajectories_(const std::string& directory);

  virtual bool planTrajectoryTask_(const std::vector<geometry_msgs::Pose>& waypoints) = 0;

  bool extractJointConfig_(const moveit_msgs::RobotTrajectory& trajectory,
                           std::vector<double>& jointConfig,
                           const ConfigPosition position);

  double computeTotalJerk_(const moveit_msgs::RobotTrajectory& trajectory);
  void sortTrajectoriesByJerk_(std::vector<moveit_msgs::RobotTrajectory>& trajectories);

  void reverseTrajectory_(moveit_msgs::RobotTrajectory& trajectory);

  bool retimeTrajectory_(moveit_msgs::RobotTrajectory& trajectory, const double cartesianSpeed);

private:
  void initMoveit_();
  void setupMoveGroup_();
  void cleanMoveGroup_();

  bool move_();
  bool manageLaser_(bool enable);
};
