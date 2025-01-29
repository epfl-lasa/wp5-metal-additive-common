/**
 * @file PlannerCleaning.cpp
 * @brief Declaration of the PlannerCleaning class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */
#include "PlannerCleaning.h"

using namespace std;

PlannerCleaning::PlannerCleaning(ROSVersion rosVersion, ros::NodeHandle& nh, string robotName, double workingSpeed) :
    IPlannerBase(rosVersion, nh, robotName, workingSpeed) {}

bool PlannerCleaning::planTrajectoryTask_(const std::vector<geometry_msgs::Pose>& waypoints) {
  ROS_ERROR("[PlannerCleaning] - Not implemented yet");
  return false;
}