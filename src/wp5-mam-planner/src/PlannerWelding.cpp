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

using namespace std;

PlannerWelding::PlannerWelding(ROSVersion rosVersion, ros::NodeHandle& nh, string robotName) :
    IPlannerBase(rosVersion, nh, robotName) {}

bool PlannerWelding::planTrajectory(std::vector<geometry_msgs::Pose> waypoints) {
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
        ROS_WARN_STREAM("[IPlannerBase] - No path found to go to Pose " << DebugTools::getPoseString(nextPose));
        return success;
      }
    } else {
      ROS_WARN("[IPlannerBase] - No IK solutions found");
      return success;
    }
  }

  return success;
}

bool PlannerWelding::computePath_(const vector<double>& startConfig,
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
      ROS_INFO_STREAM("[IPlannerBase] - Path succes : " << success << " Press Enter to continue");
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
