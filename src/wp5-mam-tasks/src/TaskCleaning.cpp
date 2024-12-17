/**
 * @file TaskCleaning.cpp
 * @brief Declaration of the TaskCleaning class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */

#include "TaskCleaning.h"

#include "PlannerCleaning.h"
#include "yaml_tools.h"

using namespace std;

TaskCleaning::TaskCleaning(ros::NodeHandle& nh, string configFilename) :
    ITaskBase(nh, YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_TASKS_DIR)))["cleaning"]) {}

bool TaskCleaning::initialize() {
  planner_ = make_unique<PlannerCleaning>(rosVersion_, nh_, robotName_);

  return true;
}

bool TaskCleaning::computeTrajectory(const std::vector<ROI::Pose>& waypoints) {
  std::vector<geometry_msgs::Pose> waypointsToPlan{};
  ROS_ERROR("[TaskCleaning] - Not implemented yet");

  return planner_->planTrajectory(waypointsToPlan);
}

bool TaskCleaning::execute() { return planner_->executeTrajectory(); }
