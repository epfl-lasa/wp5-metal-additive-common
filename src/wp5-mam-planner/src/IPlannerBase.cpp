/**
 * @file IPlannerBase.cpp
 * @brief Declaration of the IPlannerBase class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */
#include "IPlannerBase.h"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <std_msgs/Bool.h>

#include "RoboticArmFactory.h"
#include "conversion_tools.h"
#include "debug_tools.h"
#include "math_tools.h"
#include "yaml_tools.h"

using namespace std;

IPlannerBase::IPlannerBase(ROSVersion rosVersion, ros::NodeHandle& nh, string robotName) : spinner_(4), nh_(nh) {
  robot_ = RoboticArmFactory::createRoboticArm(robotName, rosVersion);
  initMoveit_();

  pubLaserState_ = nh_.advertise<std_msgs::Bool>("laser_state", 1);

#ifdef DEBUG_MODE
  pubTrajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("debug_trajectory", 1000);
  pubWaypointRviz_ = nh_.advertise<geometry_msgs::PoseStamped>("debug_waypoint", 10);
#endif

  // Add obstacles
  obstacles_ = make_unique<ObstaclesManagement>(ObstaclesManagement(nh_, moveGroup_->getPlanningFrame()));
  obstacles_->addStaticObstacles();
}

bool IPlannerBase::executeTrajectory() {
  bool success = false;
  int currentStep = 0;
  vector<double> firstJointConfig{};

  if (bestPlan_.empty()) {
    ROS_ERROR("[IPlannerBase] - No trajectory to execute");
    return success;
  }

  for (const auto& trajectory : bestPlan_) {
    currentStep++;

    // Check the first joint configuration to be able to begin trajectory execution
    firstJointConfig = trajectory.joint_trajectory.points[0].positions;

    if (!robot_->isAtJointPosition(firstJointConfig)) {
      success = goToJointConfig(firstJointConfig);

      if (!success) {
        ROS_ERROR("[IPlannerBase] - Failed to move to the first joint configuration");
        return success;
      }
    }

    // Execute the trajectory
    success = moveGroup_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    moveGroup_->stop();
    moveGroup_->clearPoseTargets();

    if (!success) {
      ROS_ERROR_STREAM("[IPlannerBase] - Failed to execute trajectory at step " << currentStep);
      return success;
    }

    ROS_INFO("[IPlannerBase] - Trajectory executed successfully");
  }

  return success;
}

bool IPlannerBase::goToJointConfig(const vector<double>& jointConfig) {
  moveGroup_->setJointValueTarget(jointConfig);
  bool success = move_();

  if (!success) {
    string jointConfigStr = DebugTools::getVecString<double>(jointConfig);

    ROS_ERROR_STREAM("[IPlannerBase] - Failed to move to the joint configuration " << jointConfigStr);
  }

  return success;
}

bool IPlannerBase::goToPose(const geometry_msgs::Pose& targetPose) {
  moveGroup_->setPoseTarget(targetPose);
  bool success = move_();

  if (!success) {
    string targetPoseStr = DebugTools::getPoseString(targetPose);

    ROS_ERROR_STREAM("[IPlannerBase] - Failed to move to the target pose " << targetPoseStr);
  }

  return success;
}

void IPlannerBase::initMoveit_() {
  const string robotGroup = "manipulator";
  ros::Duration timeout(2.0);

  try {
    moveGroup_ = make_unique<moveit::planning_interface::MoveGroupInterface>(robotGroup);
  } catch (const runtime_error& e) {
    ROS_ERROR("[IPlannerBase] - Failed to initialize MoveGroupInterface: %s", e.what());
    return;
  }

  setupMovegroup_();
  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

void IPlannerBase::setupMovegroup_() {
  moveGroup_->setPoseReferenceFrame(robot_->getReferenceFrame());
  moveGroup_->setPlannerId("RRTConnect");
  moveGroup_->setPlanningTime(2.0);
  moveGroup_->setNumPlanningAttempts(10);
  moveGroup_->setGoalPositionTolerance(0.005);
  moveGroup_->setGoalOrientationTolerance(0.01);
}

bool IPlannerBase::move_() {
  bool success = moveGroup_->move() == moveit::core::MoveItErrorCode::SUCCESS;
  moveGroup_->stop();
  moveGroup_->clearPoseTargets();

  return success;
}
