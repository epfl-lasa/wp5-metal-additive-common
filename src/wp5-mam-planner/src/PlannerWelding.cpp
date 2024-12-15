/**
 * @file PlannerWelding.cpp
 * @brief Declaration of the PlannerWelding class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */
#include "PlannerWelding.h"

#include "conversion_tools.h"
#include "debug_tools.h"
#include "math_tools.h"

using namespace std;

PlannerWelding::PlannerWelding(ROSVersion rosVersion, ros::NodeHandle& nh, string robotName) :
    IPlannerBase(rosVersion, nh, robotName) {}

bool PlannerWelding::planTrajectory(const vector<geometry_msgs::Pose>& waypoints) {
  bool success = false;
  int failedStep = -1;
  const int MIN_WAYPOINTS = 4;

  vector<double> startConfig{};

  if (waypoints.size() != MIN_WAYPOINTS) {
    ROS_ERROR_STREAM("[PlannerWelding] - The number of waypoints must be " << MIN_WAYPOINTS);
    return success;
  }

  success = computeWeldingPossiblePaths_(waypoints[1], waypoints[2]);
  failedStep = failedStep != -1 ? failedStep : (success ? -1 : 1);

  vector<geometry_msgs::Pose> tmpWaypoints{};
  for (auto& plan : sortedWeldingPaths_) {
    trajTaskToExecute_.clear();

    // Compute first transition path in reverse direction
    extractJointConfig_(plan, startConfig, ConfigPosition::FIRST);
    tmpWaypoints.clear();
    tmpWaypoints.push_back(waypoints[0]);

    success = computeTransitionPath_(startConfig, tmpWaypoints, MotionDir::BACKWARD);
    failedStep = failedStep != -1 ? failedStep : (success ? -1 : 2);

    // Compute first transition path in forward direction
    extractJointConfig_(plan, startConfig, ConfigPosition::LAST);
    tmpWaypoints.clear();
    tmpWaypoints.push_back(waypoints[3]);

    success = computeTransitionPath_(startConfig, tmpWaypoints, MotionDir::FORWARD);
    failedStep = failedStep != -1 ? failedStep : (success ? -1 : 3);

    // If all the paths are computed, store them and break the loop
    if (failedStep == -1) {
      retimeTrajectory_(plan, 0.5, 1); // TODO(lmunier) - Set the speed and frequency as a parameters

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

  // Apply transform for each pose
  // geometry_msgs::Pose startPoseTransformed = MathTools::transformPose("virtual_link", "ee_tool", startPose);
  // geometry_msgs::Pose targetPoseTransformed = MathTools::transformPose("virtual_link", "ee_tool", targetPose);

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

bool PlannerWelding::computeTransitionPath_(const vector<double>& startConfig,
                                            const vector<geometry_msgs::Pose>& targetPose,
                                            const MotionDir direction) {
  vector<moveit_msgs::RobotTrajectory> trajectory{};
  vector<geometry_msgs::Pose> targetPoseTransformed{};

  // for (const auto& pose : targetPose) {
  //   targetPoseTransformed.push_back(MathTools::transformPose("virtual_link", "ee_tool", pose));
  // }

  bool success = planCartesianFromJointConfig(startConfig, targetPose, trajectory);

  if (success) {
    if (direction == MotionDir::BACKWARD) {
      reverseTrajectory_(trajectory[0]);
    }

    trajTaskToExecute_.emplace_back(trajectory[0], false);
  }

  return success;
}
