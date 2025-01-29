/**
 * @file PlannerWelding.cpp
 * @brief Declaration of the PlannerWelding class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */
#include "PlannerWelding.h"

#include "conversion_tools.h"
#include "debug_tools.h"
#include "math_tools.h"

using namespace std;

PlannerWelding::PlannerWelding(ROSVersion rosVersion, ros::NodeHandle& nh, string robotName, double workingSpeed) :
    IPlannerBase(rosVersion, nh, robotName, workingSpeed) {}

bool PlannerWelding::planTrajectoryTask_(const vector<geometry_msgs::Pose>& waypoints) {
  bool success = false;
  int currentStep = 0;
  int failedStep = -1;
  const int MIN_WAYPOINTS = 4;
  const int STEP_FAILURE = -1;
  const int FIRST_STEP = 1;

  if (waypoints.size() != MIN_WAYPOINTS) {
    ROS_ERROR_STREAM("[PlannerWelding] - The number of waypoints must be " << MIN_WAYPOINTS);
    return success;
  }

  currentStep++;
  success = computeWeldingPossiblePaths_(waypoints[1], waypoints[2]);
  failedStep = failedStep != STEP_FAILURE ? failedStep : (success ? STEP_FAILURE : currentStep);

  for (auto& plan : sortedWeldingPaths_) {
    currentStep = FIRST_STEP;
    trajTaskToExecute_.clear();

    // Compute first transition path in reverse direction
    currentStep++;
    success = computeTransitionPath_(plan, waypoints.front(), MotionDir::BACKWARD, ConfigPosition::FIRST);
    failedStep = failedStep != STEP_FAILURE ? failedStep : (success ? STEP_FAILURE : currentStep);

    // Compute last transition path in forward direction
    currentStep++;
    success = computeTransitionPath_(plan, waypoints.back(), MotionDir::FORWARD, ConfigPosition::LAST);
    failedStep = failedStep != STEP_FAILURE ? failedStep : (success ? STEP_FAILURE : currentStep);

    // If all the paths are computed, store them and break the loop
    if (failedStep == STEP_FAILURE) {
      retimeTrajectory_(plan, workingSpeed_, 1);

      vector<pair<moveit_msgs::RobotTrajectory, bool>>::iterator insertPosition = trajTaskToExecute_.begin() + 1;
      trajTaskToExecute_.insert(insertPosition, make_pair(plan, true));
      break;
    } else {
      ROS_WARN_STREAM("[PlannerWelding] - Failed to compute the welding trajectory at step " << failedStep);
    }
  }

  if (failedStep != -1) {
    ROS_ERROR_STREAM("[PlannerWelding] - Failed to compute the welding trajectory.");
    success = false;
  }

  return success;
}

// Compute the welding path and store each feasible solutions in a vector
bool PlannerWelding::computeWeldingPossiblePaths_(const geometry_msgs::Pose& startPose,
                                                  const geometry_msgs::Pose& targetPose) {
  sortedWeldingPaths_.clear();

  bool success = false;
  bool isPathComputed = false;

  vector<geometry_msgs::Pose> weldingTarget{targetPose};
  vector<vector<double>> ikSolutions{};

  bool ikSuccess = robot_->getIKGeo(ConversionTools::geometryToEigen(startPose.orientation),
                                    ConversionTools::geometryToEigen(startPose.position),
                                    ikSolutions);

  if (ikSuccess) {
    ROS_INFO_STREAM("[PlannerWelding] - Found " << ikSolutions.size() << " IK solutions");

    for (const auto& ikSol : ikSolutions) {
      isPathComputed = planCartesianFromJointConfig(ikSol, weldingTarget, sortedWeldingPaths_);
      success = success || isPathComputed;
    }

    if (!success) {
      ROS_WARN_STREAM("[PlannerWelding] - No path found to go to Pose " << DebugTools::getPoseString(targetPose));
      return success;
    }

    // Sort the path by the total jerk
    sortTrajectoriesByJerk_(sortedWeldingPaths_);
  } else {
    ROS_WARN("[PlannerWelding] - No IK solutions found");
    return success;
  }

  return success;
}

bool PlannerWelding::computeTransitionPath_(const moveit_msgs::RobotTrajectory& trajectory,
                                            const geometry_msgs::Pose& targetWaypoint,
                                            const MotionDir direction,
                                            const ConfigPosition position) {
  vector<double> startConfig{};
  vector<moveit_msgs::RobotTrajectory> transitionPath{};

  vector<geometry_msgs::Pose> targetPose{};
  targetPose.push_back(targetWaypoint);

  // Extract the joint configuration from the trajectory
  extractJointConfig_(trajectory, startConfig, position);

  // Compute the transition path
  bool success = planCartesianFromJointConfig(startConfig, targetPose, transitionPath);

  // If the path is successfully computed, store it
  if (success) {
    if (direction == MotionDir::BACKWARD) {
      reverseTrajectory_(transitionPath.front());
    }

    trajTaskToExecute_.emplace_back(transitionPath.front(), false);
  }

  return success;
}
