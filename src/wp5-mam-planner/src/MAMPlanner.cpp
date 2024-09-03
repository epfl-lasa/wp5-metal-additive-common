/**
 * @file MAMPlanner.cpp
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-03
 */

#include "MAMPlanner.h"

//TODO(lmunier): Implement the MAMPlanner class

MAMPlanner::MAMPlanner() : spinner_(1) {
  //   try {
  //     initMoveit_();
  //     getInformation();

  //     // Add obstacles
  //     // addStaticObstacles();

  //     // Setup move group options
  //     move_group->setGoalPositionTolerance(0.005);
  //     move_group->setGoalOrientationTolerance(0.01);

  //     ik_solver = new trac_ik::TRAC_IK(robot_base, virtual_target, "Distance", ros::Duration(0.01));

  //     // TODO: Implement tracIK solutions
  //     // for each solution, start a thread and compute path planning
  //     // if path planning not successful, reconfigure platform to a new position
  //     // if path planning successful, execute the trajectory

  //   } catch (const ros::Exception& e) {
  //     ROS_ERROR("ROS Exception: %s", e.what());
  //   } catch (const std::exception& e) {
  //     ROS_ERROR("Exception: %s", e.what());
  //   } catch (...) {
  //     ROS_ERROR("Unknown exception");
  //   }
}

void MAMPlanner::planTrajectory() {}

void MAMPlanner::executeTrajectory() {}

void MAMPlanner::initMoveit_() {
  std::string robot_type, robot_group, robot_base, virtual_target;
  nh_.getParam("/robot_type", robot_type);
  nh_.getParam("group", robot_group);
  nh_.getParam("base", robot_base);
  nh_.getParam("virtual_target", virtual_target);

  //   robot = ik_geo::Robot::ur5();
  //   scene = new moveit::planning_interface::PlanningSceneInterface();
  //   move_group = new moveit::planning_interface::MoveGroupInterface(robot_group);
  //   move_group->setPoseReferenceFrame(robot_base);

  //   pub_welding_state = nh.advertise<std_msgs::Bool>("welding_state", 1);
  //   pub_display_trajectory = nh.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 20);

  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}