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

bool TaskWelding::computeTrajectory(std::vector<geometry_msgs::Pose> waypoints) {
  std::vector<geometry_msgs::Pose> waypointsToPlan{};
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> poseOffset{};

  bool firstWaypoint = true;
  for (const auto& pose : waypoints) {
    if (firstWaypoint) {
      poseOffset = ConversionTools::vectorToEigenQuatPose(eePoseWorkOffset_);
      waypointsToPlan.push_back(MathTools::addOffset(pose, poseOffset));

      firstWaypoint = false;
    }

    poseOffset = ConversionTools::vectorToEigenQuatPose(eePoseOffset_);
    waypointsToPlan.push_back(MathTools::addOffset(pose, poseOffset));
  }

  return planner_->planTrajectory(waypointsToPlan);
}

bool TaskWelding::execute() { return planner_->executeTrajectory(); }
