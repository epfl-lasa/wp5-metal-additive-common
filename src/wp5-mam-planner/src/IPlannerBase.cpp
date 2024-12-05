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

  if (trajectoryToExecute_.empty()) {
    ROS_ERROR("[IPlannerBase] - No trajectory to execute");
    return success;
  }

  for (const auto& trajectory : trajectoryToExecute_) {
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
#ifdef DEBUG_MODE
    string userMsg =
        "[IPlannerBase] - Execute trajectory at step " + to_string(currentStep) + ". Press Enter to continue.";

    DebugTools::publishTrajectory(*moveGroup_, trajectory, pubTrajectory_);
    DebugTools::waitOnUser(userMsg);
#endif

    success = (moveGroup_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
    cleanMoveGroup_();

    if (!success) {
      ROS_ERROR_STREAM("[IPlannerBase] - Failed to execute trajectory at step " << currentStep);
      return success;
    }

    ROS_INFO("[IPlannerBase] - Trajectory executed successfully");
  }

  return success;
}

bool IPlannerBase::planCartesianFromJointConfig(const std::vector<double>& startJointConfig,
                                                const std::vector<geometry_msgs::Pose>& waypoints,
                                                std::vector<moveit_msgs::RobotTrajectory>& pathPlanned) {
  cleanMoveGroup_();
  bool success = false;

  const double eef_step = 0.01;
  const std::string robotGroup = "manipulator";
  moveit_msgs::RobotTrajectory planTrajectory;

  // Create a RobotState object and set it to the desired starting joint configuration
  moveit::core::RobotState startState(*moveGroup_->getCurrentState());
  const moveit::core::JointModelGroup* jointModelGroup = startState.getJointModelGroup(robotGroup);
  startState.setJointGroupPositions(jointModelGroup, startJointConfig);
  moveGroup_->setStartState(startState);

  // Compute Cartesian path
  moveit_msgs::MoveItErrorCodes error_code;
  success = (moveGroup_->computeCartesianPath(waypoints, eef_step, planTrajectory, true) == 1.0);

  if (success) {
    ROS_INFO("[IPlannerBase] - Cartesian path computed successfully.");
    pathPlanned.push_back(planTrajectory);
  } else {
    ROS_ERROR_STREAM("[IPlannerBase] - Failed to compute the Cartesian path. Error code: " << error_code.val);
  }

  return success;
}

bool IPlannerBase::goToJointConfig(const vector<double>& jointConfig) {
  cleanMoveGroup_();

  moveGroup_->setJointValueTarget(jointConfig);
  bool success = move_();

  if (!success) {
    string jointConfigStr = DebugTools::getVecString<double>(jointConfig);

    ROS_ERROR_STREAM("[IPlannerBase] - Failed to move to the joint configuration " << jointConfigStr);
  }

  return success;
}

bool IPlannerBase::goToPose(const geometry_msgs::Pose& targetPose) {
  cleanMoveGroup_();

  moveGroup_->setPoseTarget(targetPose);
  bool success = move_();

  if (!success) {
    string targetPoseStr = DebugTools::getPoseString(targetPose);

    ROS_ERROR_STREAM("[IPlannerBase] - Failed to move to the target pose " << targetPoseStr);
  }

  return success;
}

bool IPlannerBase::extractJointConfig(const moveit_msgs::RobotTrajectory& trajectory,
                                      std::vector<double>& jointConfig,
                                      const ConfigPosition position) {
  if (trajectory.joint_trajectory.points.empty()) {
    ROS_WARN("[IPlannerBase] - The trajectory is empty.");
    return false;
  }

  const auto& point = (position == ConfigPosition::FIRST) ? trajectory.joint_trajectory.points.front()
                                                          : trajectory.joint_trajectory.points.back();
  jointConfig = point.positions;
  return true;
}

void IPlannerBase::reverseTrajectory(moveit_msgs::RobotTrajectory& trajectory) {
  // Reverse the joint trajectory points
  std::reverse(trajectory.joint_trajectory.points.begin(), trajectory.joint_trajectory.points.end());

  // Optionally, reverse the multi-dof joint trajectory points if they exist
  if (!trajectory.multi_dof_joint_trajectory.points.empty()) {
    std::reverse(trajectory.multi_dof_joint_trajectory.points.begin(),
                 trajectory.multi_dof_joint_trajectory.points.end());
  }
}

double IPlannerBase::computeTotalJerk_(const moveit_msgs::RobotTrajectory& trajectory) {
  const int8_t MIN_POINTS = 3;
  double totalJerk = 0.0;

  const auto& points = trajectory.joint_trajectory.points;
  if (points.size() < MIN_POINTS) {
    ROS_WARN_STREAM("[IPlannerBase] - Not enough points to compute jerk : current size of " << points.size());
    return totalJerk;
  }

  for (size_t i = 1; i < points.size() - 1; ++i) {
    const auto& prev = points[i - 1];
    const auto& curr = points[i];
    const auto& next = points[i + 1];

    Eigen::VectorXd velPrev = Eigen::VectorXd::Map(prev.velocities.data(), prev.velocities.size());
    Eigen::VectorXd velCurr = Eigen::VectorXd::Map(curr.velocities.data(), curr.velocities.size());
    Eigen::VectorXd velNext = Eigen::VectorXd::Map(next.velocities.data(), next.velocities.size());

    double timePrev = prev.time_from_start.toSec();
    double timeCurr = curr.time_from_start.toSec();
    double timeNext = next.time_from_start.toSec();

    Eigen::VectorXd accPrev = (velCurr - velPrev) / (timeCurr - timePrev);
    Eigen::VectorXd accNext = (velNext - velCurr) / (timeNext - timeCurr);

    Eigen::VectorXd jerk = (accNext - accPrev) / (timeNext - timePrev);

    totalJerk += jerk.norm();
  }

  return totalJerk;
}

void IPlannerBase::sortTrajectoriesByJerk_(std::vector<moveit_msgs::RobotTrajectory>& trajectories) {
  std::sort(sortedWeldingPaths_.begin(),
            sortedWeldingPaths_.end(),
            [this](const moveit_msgs::RobotTrajectory& a, const moveit_msgs::RobotTrajectory& b) {
              return computeTotalJerk_(a) < computeTotalJerk_(b);
            });
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

  setupMoveGroup_();
  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

void IPlannerBase::setupMoveGroup_() {
  moveGroup_->setPoseReferenceFrame(robot_->getReferenceFrame());
  moveGroup_->setPlannerId("RRTConnect");
  moveGroup_->setPlanningTime(2.0);
  moveGroup_->setNumPlanningAttempts(10);
  moveGroup_->setGoalPositionTolerance(0.005);
  moveGroup_->setGoalOrientationTolerance(0.01);
}

void IPlannerBase::cleanMoveGroup_() {
  moveGroup_->stop();
  moveGroup_->clearPoseTargets();
  moveGroup_->setStartStateToCurrentState();
}

bool IPlannerBase::move_() {
  moveit::planning_interface::MoveGroupInterface::Plan currentPlan;
  bool success = (moveGroup_->plan(currentPlan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
#ifdef DEBUG_MODE
    string userMsg = "[IPlannerBase] - Path succes : " + to_string(success) + " Press Enter to continue.";
    const moveit_msgs::RobotTrajectory& robotTrajectory = currentPlan.trajectory_;

    DebugTools::publishTrajectory(*moveGroup_, robotTrajectory, pubTrajectory_);
    DebugTools::waitOnUser(userMsg);
#endif

    success = (moveGroup_->execute(currentPlan) == moveit::core::MoveItErrorCode::SUCCESS);
    cleanMoveGroup_();
  }

  return success;
}
