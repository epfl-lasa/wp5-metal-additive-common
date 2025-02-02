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

  /**
   * @brief Moves the end effector to the specified scan area.
   *
   * This function commands the robot to move its end effector to a designated
   * scan area defined by the given pose.
   *
   * @param eePoseScan A vector of doubles representing the pose of the end effector
   *                   in the scan area. Quaternion - xyzw [rad] and Position - xyz [m]
   * @return true if the movement to the scan area was successful, false otherwise.
   */
  bool goToScanArea(const std::vector<double>& eePoseScan);

  /**
   * @brief Moves the robot to the specified joint configuration.
   *
   * This function takes a vector of joint angles and commands the robot to move
   * to the specified configuration. The size and order of the joint angles in
   * the vector should match the robot's joint configuration.
   *
   * @param jointConfig A vector of doubles representing the target joint angles [rad].
   * @return true if the robot successfully reaches the target configuration, false otherwise.
   */
  bool goToJointConfig(const std::vector<double>& jointConfig);

  /**
   * @brief Moves the robot to the specified target pose.
   *
   * This function commands the robot to move to the given target pose.
   *
   * @param targetPose The desired pose to which the robot should move.
   * @return true if the robot successfully reaches the target pose, false otherwise.
   */
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

  // List of trajectories to be executed, with task (welding / cleaning) to be enable / disable
  std::vector<std::pair<moveit_msgs::RobotTrajectory, bool>> trajTaskToExecute_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup_ = nullptr; ///< MoveGroup interface

  /**
   * @brief Removes high joint movements from the given joint positions.
   *
   * This function iterates through the provided joint positions and removes any
   * joint movements that exceed the specified limit. The default limit is set to
   * 3 * π/4.
   *
   * @param jointPos A reference to a vector of vectors containing joint positions.
   * @param limitJointMove (optional) The threshold for joint movement. Any joint movement
   *        exceeding this limit will be removed. Default value is 3 * π/4.
   */
  void removeHighJointMoves(std::vector<std::vector<double>>& jointPos, const double limitJointMove = 3 * M_PI_4);

  /**
   * @brief Save the given trajectory to a file
   *
   * @param trajectory The trajectory to save
   * @param filename The name of the file to save the trajectory to
   * @return true if the trajectory was saved successfully, false otherwise
   */
  bool saveTrajectory_(const moveit_msgs::RobotTrajectory& trajectory, const std::string& filename);

  /**
   * @brief Load a trajectory from a file
   *
   * @param trajectory The trajectory to load into
   * @param filename The name of the file to load the trajectory from
   * @return true if the trajectory was loaded successfully, false otherwise
   */
  bool loadTrajectory_(moveit_msgs::RobotTrajectory& trajectory, const std::string& filename);

  /**
   * @brief Load all trajectories from a directory
   *
   * @param directory The directory to load the trajectories from
   * @return true if all trajectories were loaded successfully, false otherwise
   */
  bool loadAllTrajectories_(const std::string& directory);

  /**
   * @brief Plan the trajectory for the task for the given waypoints
   *
   * @param waypoints The waypoints to plan the trajectory for
   * @return true if the trajectory was planned successfully, false otherwise
   */
  virtual bool planTrajectoryTask_(const std::vector<geometry_msgs::Pose>& waypoints) = 0;

  /**
   * @brief Extract a given joint configuration from the given trajectory, based on its
   * position in the trajectory
   *
   * @param trajectory The trajectory to extract the joint configuration from
   * @param jointConfig The extracted joint configuration
   * @param position The position in the trajectory to extract the joint configuration from
   * @return true if the joint configuration was extracted successfully, false otherwise
   */
  bool extractJointConfig_(const moveit_msgs::RobotTrajectory& trajectory,
                           std::vector<double>& jointConfig,
                           const ConfigPosition position);

  /**
   * @brief Compute the total jerk of the given trajectory
   *
   * It is done to have a metric to sort the possible trajectories for the task.
   *
   * @param trajectory The trajectory to compute the total jerk for
   * @return The total jerk of the trajectory
   */
  double computeTotalJerk_(const moveit_msgs::RobotTrajectory& trajectory);

  /**
   * @brief Sort the given trajectories by their total jerk
   *
   * @param trajectories The trajectories to sort
   */
  void sortTrajectoriesByJerk_(std::vector<moveit_msgs::RobotTrajectory>& trajectories);

  /**
   * @brief Reverse the given trajectory
   *
   * This function reverses the given trajectory by reversing the joint trajectory points.
   *
   * @warning Doing this this way lead to shaking trajectory execution. To smooth it back and
   * manage the velocity profile, the trajectory should be re-timed using the retimeTrajectory_ function.
   *
   * @param trajectory The trajectory to reverse
   */
  void reverseTrajectory_(moveit_msgs::RobotTrajectory& trajectory);

  /**
   * @brief Retime the given trajectory to achieve the specified Cartesian speed
   *
   * @param trajectory The trajectory to retime
   * @param cartesianSpeed The desired Cartesian speed [m/s]
   * @return true if the trajectory was retimed successfully, false otherwise
   */
  bool retimeTrajectory_(moveit_msgs::RobotTrajectory& trajectory, const double cartesianSpeed);

private:
  /**
   * @brief Initialize Moveit! for the robot.
   *
   */
  void initMoveit_();

  /**
   * @brief Set up the MoveGroup interface.
   *
   */
  void setupMoveGroup_();

  /**
   * @brief Clean the MoveGroup interface.
   *
   */
  void cleanMoveGroup_();

  /**
   * @brief Move the robot using the currently setup MoveGroup interface.
   *
   * @return true if the operation was successful, false otherwise.
   */
  bool move_();

  /**
   * @brief Manages the state of the laser.
   *
   * This function enables or disables the laser, based on the provided parameter, using its FSM.
   *
   * @param enable A boolean value indicating whether to enable (true) or disable (false) the laser.
   * @return true if the operation was successful, false otherwise.
   */
  bool manageLaser_(bool enable);
};
