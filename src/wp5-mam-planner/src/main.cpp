/**
 * @file main.cpp
 * @brief Temporary file to test implementation WIP
 * @author [Louis Munier]
 * @date 2024-09-04
 */

#include <ros/ros.h>

#include "MAMPlanner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wp5_mam_planner_node");

  // Create an instance of MAMPlanner
  MAMPlanner planner(ROSVersion::ROS1_NOETIC);

  // Plan a trajectory
  planner.planTrajectory();

  // Execute the planned trajectory
  planner.executeTrajectory();

  return 0;
}