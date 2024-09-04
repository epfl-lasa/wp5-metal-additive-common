/**
 * @file MAMPlanner.cpp
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-03
 */
#include "MAMPlanner.h"

#include <moveit_msgs/DisplayTrajectory.h>
#include <std_msgs/Bool.h>

using namespace std;

//TODO(lmunier): Implement the MAMPlanner class

MAMPlanner::MAMPlanner() : spinner_(1) {
  try {
    robot_ = make_unique<RoboticArmUr5>();
    robot_->printInfo();
    initMoveit_();

    pub_welding_state_ = nh_.advertise<std_msgs::Bool>("welding_state", 1);
    pub_display_trajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 20);

    // Add obstacles
    // addStaticObstacles();

    // ik_solver_ = make_unique<TRAC_IK::TRAC_IK>(robot_base, virtual_target, "Distance", ros::Duration(0.01));

    // TODO: Implement tracIK solutions
    // for each solution, start a thread and compute path planning
    // if path planning not successful, reconfigure platform to a new position
    // if path planning successful, execute the trajectory

  } catch (const ros::Exception& e) {
    ROS_ERROR("ROS Exception: %s", e.what());
  } catch (const exception& e) {
    ROS_ERROR("Exception: %s", e.what());
  } catch (...) {
    ROS_ERROR("Unknown exception");
  }
}

void MAMPlanner::planTrajectory() {
  //TODO(lmunier): Implement the planTrajectory method
  cout << "Planning trajectory" << endl;
}

void MAMPlanner::executeTrajectory() {
  //TODO(lmunier): Implement the executeTrajectory method
  cout << "Executing trajectory" << endl;
}

void MAMPlanner::initMoveit_() {
  string robot_group = "manipulator";
  string robot_base = "base_link_inertia";

  scene_ = make_unique<moveit::planning_interface::PlanningSceneInterface>();
  move_group_ = make_unique<moveit::planning_interface::MoveGroupInterface>(robot_group);
  move_group_->setPoseReferenceFrame(robot_base);
  move_group_->setPlannerId("RRTConnectkConfigDefault");
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(10);
  move_group_->setGoalPositionTolerance(0.005);
  move_group_->setGoalOrientationTolerance(0.01);

  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}