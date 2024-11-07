/**
 * @file MAMPlanner.cpp
 * @brief Declaration of the MAMPlanner class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.1
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

MAMPlanner::MAMPlanner(ROSVersion rosVersion, ros::NodeHandle& nh, string robotName) :
    spinner_(4), nh_(nh), tfListener_(tfBuffer_), subtask_(make_unique<Subtask>(nh_)) {
  RoboticArmFactory armFactory = RoboticArmFactory();
  robot_ = armFactory.createRoboticArm(robotName, rosVersion);
  robot_->printInfo();

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

bool MAMPlanner::planTrajectory() {
  currentWPointID_ = 0;
  bestPlan_.clear();
  geometry_msgs::Pose currentPose = moveGroup_->getCurrentPose().pose;

  while (subtask_->empty()) {
    float TIME_WAIT = 0.2;
    ROS_WARN("[MAMPlanner] - Waiting for Waypoints");
    ros::Duration(TIME_WAIT).sleep();
  }

  while (!subtask_->empty()) {
    optional<ROI> roi = subtask_->popROI();

    if (!roi.has_value()) {
      ROS_ERROR("[MAMPlanner] - No ROI received");
      return false;
    }

    geometry_msgs::Pose startTaskPose = roi.value().getPoseROS(0);
    geometry_msgs::Pose endTaskPose = roi.value().getPoseROS(1);

    // Robot welding task
    if (!computeTrajectory_(startTaskPose, endTaskPose, true)) {
      return false;
    }
    currentWPointID_++;
  }

  return true;
}

void MAMPlanner::executeTrajectory() {
  bool success = false;
  vector<double> firstJointConfig{};

  if (bestPlan_.empty()) {
    ROS_ERROR("[MAMPlanner] - No trajectory to execute");
    return;
  }

  for (const auto& trajectory : bestPlan_) {
    firstJointConfig = trajectory.joint_trajectory.points[0].positions;

    if (!robot_->isAtJointPosition(firstJointConfig)) {
      moveGroup_->setJointValueTarget(firstJointConfig);

      success = moveGroup_->move() == moveit::core::MoveItErrorCode::SUCCESS;
      moveGroup_->stop();
      moveGroup_->clearPoseTargets();

      if (!success) {
        ROS_ERROR("[MAMPlanner] - Failed to move to the starting joint configuration.");
        return;
      }
    }

    success = moveGroup_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    moveGroup_->stop();
    moveGroup_->clearPoseTargets();

    if (success) {
      ROS_INFO("[MAMPlanner] - Trajectory executed successfully");
    } else {
      ROS_ERROR("[MAMPlanner] - Failed to execute trajectory");
      return;
    }
  }
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
                              const geometry_msgs::Pose& currentPose,
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

  if (isWelding) {
    // Compute Cartesian path
    vector<geometry_msgs::Pose> waypoints{};
    waypoints.push_back(currentPose);
    waypoints.push_back(targetPose);

    double fraction = moveGroup_->computeCartesianPath(waypoints, 0.01, planTrajectory, true);
    success = fraction == 1.0; // Path 100% computed
  } else {
    // Set the target pose
    moveGroup_->setPoseTarget(targetPose);

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = moveGroup_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    planTrajectory = plan.trajectory_;
  }

  if (success) {
    // Retiming logic to ensure constant speed
    robot_trajectory::RobotTrajectory rt(moveGroup_->getCurrentState()->getRobotModel(), robotGroup);
    rt.setRobotTrajectoryMsg(*moveGroup_->getCurrentState(), planTrajectory);

    // Use IterativeParabolicTimeParameterization to retime the trajectory
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    // TODO(lmunier): Check Velocity scaling factor to set: 1.0 for constant speed
    bool retimed = iptp.computeTimeStamps(rt, 0.2);

    if (!retimed) {
      ROS_WARN("[MAMPlanner] - Trajectory retiming failed");
    }

    // Update the planTrajectory with the retimed data
    rt.getRobotTrajectoryMsg(planTrajectory);

#ifdef DEBUG_MODE
    DebugTools::publishTrajectory(*moveGroup_, planTrajectory, pubTrajectory_);
#endif

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

bool MAMPlanner::computeTrajectory_(const geometry_msgs::Pose currentPose,
                                    const geometry_msgs::Pose nextPose,
                                    const bool welding) {
  bool isPathFound = false;
  bool isPathComputed = false;
  vector<vector<double>> ikSolutions{};

  bool ikSuccess = robot_->getIKGeo(ConversionTools::geometryToEigen(currentPose.orientation),
                                    ConversionTools::geometryToEigen(currentPose.position),
                                    ikSolutions);

  if (ikSuccess) {
    ROS_INFO("[MAMPlanner] - IK solutions found");

    for (const auto& ikSol : ikSolutions) {
      isPathComputed = computePath_(ikSol, currentPose, nextPose, welding);
      isPathFound = isPathFound || isPathComputed;
    }

    if (!isPathFound) {
      ROS_WARN_STREAM("[MAMPlanner] - No path found to go to Pose " << DebugTools::getPoseString(nextPose));
      return false;
    }
  } else {
    ROS_WARN("[MAMPlanner] - No IK solutions found");
    return false;
  }

  return true;
}

void MAMPlanner::createNewFrame_(const string& parentFrame,
                                 const string& newFrame,
                                 const geometry_msgs::Transform& transform) {
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parentFrame;
  transformStamped.child_frame_id = newFrame;
  transformStamped.transform = transform;

  br_.sendTransform(transformStamped);
}
