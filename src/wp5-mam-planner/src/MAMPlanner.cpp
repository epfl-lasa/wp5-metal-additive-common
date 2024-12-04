/**
 * @file MAMPlanner.cpp
 * @brief Declaration of the MAMPlanner class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-09-12
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */
#include "MAMPlanner.h"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <std_msgs/Bool.h>

#include "RoboticArmFactory.h"
#include "conversion_tools.h"
#include "debug_tools.h"
#include "math_tools.h"
#include "yaml_tools.h"

using namespace std;

MAMPlanner::MAMPlanner(ROSVersion rosVersion, ros::NodeHandle& nh, string robotName) : spinner_(4), nh_(nh) {
  robot_ = RoboticArmFactory::createRoboticArm(robotName, rosVersion);
  initMoveit_();

  pubWeldingState_ = nh_.advertise<std_msgs::Bool>("welding_state", 1);

#ifdef DEBUG_MODE
  pubTrajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("debug_trajectory", 1000);
  pubWaypointRviz_ = nh_.advertise<geometry_msgs::PoseStamped>("debug_waypoint", 10);
#endif

  // Add obstacles
  obstacles_ = make_unique<ObstaclesManagement>(ObstaclesManagement(nh_, moveGroup_->getPlanningFrame()));
  obstacles_->addStaticObstacles();
}

bool MAMPlanner::planTrajectory(std::vector<geometry_msgs::Pose> waypoints) {
  bestPlan_.clear();

  bool success = false;
  bool isPathComputed = false;
  vector<vector<double>> ikSolutions{};

  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    geometry_msgs::Pose pose = waypoints[i];
    geometry_msgs::Pose nextPose = waypoints[i + 1];
    bool welding = i == waypoints.size() - 2; // Last section is the welding one

    bool ikSuccess = robot_->getIKGeo(ConversionTools::geometryToEigen(pose.orientation),
                                      ConversionTools::geometryToEigen(pose.position),
                                      ikSolutions);

    if (ikSuccess) {
      for (const auto& ikSol : ikSolutions) {
        isPathComputed = computePath_(ikSol, nextPose, welding);
        success = success || isPathComputed;
      }

      if (!success) {
        ROS_WARN_STREAM("[MAMPlanner] - No path found to go to Pose " << DebugTools::getPoseString(nextPose));
        return success;
      }
    } else {
      ROS_WARN("[MAMPlanner] - No IK solutions found");
      return success;
    }
  }

  return success;
}

bool MAMPlanner::executeTrajectory() {
  bool success = false;
  int currentStep = 0;
  vector<double> firstJointConfig{};

  if (bestPlan_.empty()) {
    ROS_ERROR("[MAMPlanner] - No trajectory to execute");
    return success;
  }

  for (const auto& trajectory : bestPlan_) {
    currentStep++;

    // Check the first joint configuration to be able to begin trajectory execution
    firstJointConfig = trajectory.joint_trajectory.points[0].positions;

    if (!robot_->isAtJointPosition(firstJointConfig)) {
      success = goToJointConfig(firstJointConfig);

      if (!success) {
        ROS_ERROR("[MAMPlanner] - Failed to move to the first joint configuration");
        return success;
      }
    }

    // Execute the trajectory
    success = moveGroup_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    moveGroup_->stop();
    moveGroup_->clearPoseTargets();

    if (!success) {
      ROS_ERROR_STREAM("[MAMPlanner] - Failed to execute trajectory at step " << currentStep);
      return success;
    }

    ROS_INFO("[MAMPlanner] - Trajectory executed successfully");
  }

  return success;
}

bool MAMPlanner::goToJointConfig(const vector<double>& jointConfig) {
  moveGroup_->setJointValueTarget(jointConfig);
  bool success = move_();

  if (!success) {
    string jointConfigStr = DebugTools::getVecString<double>(jointConfig);

    ROS_ERROR_STREAM("[MAMPlanner] - Failed to move to the joint configuration " << jointConfigStr);
  }

  return success;
}

bool MAMPlanner::goToPose(const geometry_msgs::Pose& targetPose) {
  moveGroup_->setPoseTarget(targetPose);
  bool success = move_();

  if (!success) {
    string targetPoseStr = DebugTools::getPoseString(targetPose);

    ROS_ERROR_STREAM("[MAMPlanner] - Failed to move to the target pose " << targetPoseStr);
  }

  return success;
}

void MAMPlanner::initMoveit_() {
  const string robotGroup = "manipulator";
  ros::Duration timeout(2.0);

  try {
    moveGroup_ = make_unique<moveit::planning_interface::MoveGroupInterface>(robotGroup);
  } catch (const runtime_error& e) {
    ROS_ERROR("[MAMPlanner] - Failed to initialize MoveGroupInterface: %s", e.what());
    return;
  }

  setupMovegroup_();
  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

void MAMPlanner::setupMovegroup_() {
  moveGroup_->setPoseReferenceFrame(robot_->getReferenceFrame());
  moveGroup_->setPlannerId("RRTConnect");
  moveGroup_->setPlanningTime(2.0);
  moveGroup_->setNumPlanningAttempts(10);
  moveGroup_->setGoalPositionTolerance(0.005);
  moveGroup_->setGoalOrientationTolerance(0.01);
}

bool MAMPlanner::computePath_(const vector<double>& startConfig,
                              const geometry_msgs::Pose& targetPose,
                              const bool isWelding) {
  bool success = false;
  const string robotGroup = "manipulator";
  moveit_msgs::RobotTrajectory planTrajectory;

  moveGroup_->clearPoseTargets();

  // Create a RobotState object and set it to the desired starting joint configuration
  moveit::core::RobotState startState(*moveGroup_->getCurrentState());
  const moveit::core::JointModelGroup* jointModelGroup = startState.getJointModelGroup(robotGroup);
  startState.setJointGroupPositions(jointModelGroup, startConfig);

  // Set the starting state in the planning scene
  moveGroup_->setStartState(startState);

  if (isWelding) { // Planify a cartesian path
    vector<geometry_msgs::Pose> waypoints{};
    waypoints.push_back(targetPose);

    double fraction = moveGroup_->computeCartesianPath(waypoints, 0.01, planTrajectory, true);

    // Path 100% computed
    success = fraction == 1.0;
  } else { // Planify a path
    moveGroup_->setPoseTarget(targetPose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = moveGroup_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    planTrajectory = plan.trajectory_;
  }

#ifdef DEBUG_MODE
  DebugTools::publishTrajectory(*moveGroup_, planTrajectory, pubTrajectory_);

  bool msgShowed = false;
  bool userInput = false;
  while (!userInput) {
    if (!msgShowed) {
      ROS_INFO_STREAM("[MAMPlanner] - Path succes : " << success << " Press Enter to continue");
      msgShowed = true;
    }

    if (cin.get() == '\n') {
      userInput = true;
    }
  }
#endif

  if (success) {
    if (currentWPointID_ >= bestPlan_.size()) {
      bestPlan_.push_back(planTrajectory);
    } else {
      double currentPlanSize = planTrajectory.joint_trajectory.points.size();
      double bestPlanSize = bestPlan_[currentWPointID_].joint_trajectory.points.size();

      if (currentPlanSize < bestPlanSize) {
        bestPlan_[currentWPointID_] = planTrajectory;
      }
    }
  }

  return success;
}

bool MAMPlanner::move_() {
  bool success = moveGroup_->move() == moveit::core::MoveItErrorCode::SUCCESS;
  moveGroup_->stop();
  moveGroup_->clearPoseTargets();

  return success;
}
