/**
 * @file MAMPlanner.h
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-03
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
#include "RoboticArmCr7.h"
#include "RoboticArmUr.h"
#include "Subtask.h"

class MAMPlanner {
public:
  /**
   * @brief Constructor.
   */
  MAMPlanner(ROSVersion rosVersion, ros::NodeHandle& nh);

  /**
   * @brief Destructor.
   */
  ~MAMPlanner() = default;

  /**
   * @brief Plans the trajectory of the robot.
   */
  bool planTrajectory();

  /**
   * @brief Executes the trajectory of the robot.
   */
  void executeTrajectory();

private:
  std::unique_ptr<IRoboticArmBase> robot_ = nullptr;         ///< Robotic arm
  std::unique_ptr<ObstaclesManagement> obstacles_ = nullptr; ///< Obstacles management

  ros::NodeHandle nh_; ///< ROS node handle
  ros::AsyncSpinner spinner_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster br_; ///< ROS spinner to handle callbacks asynchronously

  ros::Publisher pubWeldingState_;             ///< Publisher for the welding state
  ros::Publisher pubDisplayTrajectory_;        ///< Publisher for the display trajectory
  ros::Publisher pubWaypointRviz_;             ///< Publisher for the waypoint in Rviz
  ros::Publisher pubWaypointCoppeliasim_;      ///< Publisher for the waypoint in CoppeliaSim
  std::unique_ptr<Subtask> subtask_ = nullptr; ///< Subtask

  bool pathFound_ = false;
  int currentWPointID_ = 0;
  // std::vector<Waypoint> waypoints_; ///< Waypoints for the robot
  std::vector<moveit_msgs::RobotTrajectory> bestPlan_;

  moveit::core::RobotStatePtr robotState_ = nullptr;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup_ = nullptr; ///< MoveGroup interface

  void initMoveit_();
  void setupMovegroup_();
  bool computePath_(const std::vector<double>& startConfig,
                    const geometry_msgs::Pose& currentPose,
                    const geometry_msgs::Pose& targetPose,
                    const bool isWeldging = false);

  bool computeTrajectory_(const geometry_msgs::Pose currentPose,
                          const geometry_msgs::Pose nextPose,
                          const bool welding);

  geometry_msgs::Pose projectPose_(const geometry_msgs::Pose& pose,
                                   const std::string& fromFrame,
                                   const std::string& toFrame);
  void createNewFrame_(const std::string& parentFrame,
                       const std::string& newFrame,
                       const geometry_msgs::Transform& transform);

  // void getWaypoints_();
  void publishWaypointRviz_(const geometry_msgs::Pose& pose, const std::string& frameId);
  void publishWaypointCoppeliasim_(const geometry_msgs::Pose& pose, const std::string& frameId);
};