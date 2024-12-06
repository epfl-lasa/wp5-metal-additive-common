/**
 * @file TaskWelding.cpp
 * @brief Declaration of the TaskWelding class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */

#include "TaskWelding.h"

#include "PlannerWelding.h"
#include "conversion_tools.h"
#include "yaml_tools.h"

using namespace std;

TaskWelding::TaskWelding(ros::NodeHandle& nh, string configFilename) :
    ITaskBase(nh, YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_TASKS_DIR)))["welding"]) {}

bool TaskWelding::initialize() {
  planner_ = make_unique<PlannerWelding>(rosVersion_, nh_, robotName_);

  return true;
}

//TODO(lmunier) - Solve adding the offset to the waypoints, with the orientation
bool TaskWelding::computeTrajectory(const std::vector<geometry_msgs::Pose>& waypoints) {
  std::vector<geometry_msgs::Pose> waypointsToPlan{};
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> poseOffset{};

  geometry_msgs::Pose eePoseWorkOffset = ConversionTools::vectorToGeometryPose(eePoseWorkOffset_);
  // eePoseWorkOffset = MathTools::applyRotationToPose(eePoseWorkOffset, quatTransform);

  geometry_msgs::Pose eePoseOffset = ConversionTools::vectorToGeometryPose(eePoseOffset_);
  // eePoseOffset = MathTools::applyRotationToPose(eePoseOffset, quatTransform);

  // Add moving to welding pose
  waypointsToPlan.push_back(MathTools::addOffset(waypoints.front(), eePoseWorkOffset));

  // Add weling waypoints
  for (size_t i = 0; i < waypoints.size(); ++i) {
    waypointsToPlan.push_back(MathTools::addOffset(waypoints[i], eePoseOffset));
  }

  // Add moving away from welding pose
  waypointsToPlan.push_back(MathTools::addOffset(waypoints.back(), eePoseWorkOffset));

  return planner_->planTrajectory(waypointsToPlan);
}

bool TaskWelding::execute() { return planner_->executeTrajectory(); }
