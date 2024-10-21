/**
 * @file MAMPlanner.cpp
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-12
 */
#include "MAMPlanner.h"

#include <std_msgs/Bool.h>

#include "RoboticArmFactory.h"
#include "convertion_tools.h"
#include "math_tools.h"
#include "yaml_tools.h"

using namespace std;

MAMPlanner::MAMPlanner(ROSVersion rosVersion, ros::NodeHandle& nh) :
    spinner_(1), nh_(nh), tfListener_(tfBuffer_), subtask_(make_unique<Subtask>(nh_)) {
  RoboticArmFactory armFactory = RoboticArmFactory();
  robot_ = armFactory.createRoboticArm("ur10e_robot", rosVersion); //TODO(lmunier): Change to not hardcode value
  robot_->printInfo();

  initMoveit_();

  pubWeldingState_ = nh_.advertise<std_msgs::Bool>("welding_state", 1);
  pubDisplayTrajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 20);
  pubWaypointRviz_ = nh_.advertise<geometry_msgs::PoseStamped>("debug_waypoint", 10);
  pubWaypointCoppeliasim_ = nh_.advertise<geometry_msgs::PoseArray>("/visualisation/waypoints", 10);

  // Add obstacles
  obstacles_ = make_unique<ObstaclesManagement>(ObstaclesManagement(moveGroup_->getPlanningFrame()));
  // obstacles_->addStaticObstacles();
}

bool MAMPlanner::planTrajectory() {
  cout << "Planning trajectory" << endl;

  currentWPointID_ = 0;
  bestPlan_.clear();
  geometry_msgs::Pose currentPose = moveGroup_->getCurrentPose().pose;

  publishWaypointRviz_(currentPose, "base_link_inertia");

  bool pathFound = false;
  while (subtask_->empty()) {
    float TIME_WAIT = 0.2;
    ROS_WARN("Waiting for Waypoints");
    ros::Duration(TIME_WAIT).sleep();
  }

  while (!subtask_->empty()) {
    auto waypoint = subtask_->getROI();
    geometry_msgs::Pose startTaskPose = ConvertionTools::vectorToPose(waypoint.getPoseVector("start"));
    geometry_msgs::Pose endTaskPose = ConvertionTools::vectorToPose(waypoint.getPoseVector("end"));
    publishWaypointRviz_(currentPose, "base_link_inertia");
    publishWaypointRviz_(startTaskPose, "base_link_inertia");

    // Robot goes to start pose
    pathFound = computeTrajectory_(currentPose, startTaskPose, false);
    if (!pathFound) {
      return pathFound;
    }

    // Robot welding task
    pathFound = computeTrajectory_(startTaskPose, endTaskPose, true);
    if (!pathFound) {
      return pathFound;
    }
  }

  return true;
}

bool MAMPlanner::computeTrajectory_(const geometry_msgs::Pose currentPose,
                                    const geometry_msgs::Pose nextPose,
                                    const bool welding) {
  vector<vector<double>> ikSolutions{};
  bool ikSuccess = false;
  bool isPathFound = false;

  ikSuccess = robot_->getIKGeo(ConvertionTools::geometryToEigen(currentPose.orientation),
                               ConvertionTools::geometryToEigen(currentPose.position),
                               ikSolutions);

  if (ikSuccess) {
    ROS_INFO("IK solutions found");

    for (const auto& ikSol : ikSolutions) {
      vector<double> startConfig = ikSol;
      isPathFound += computePath_(startConfig, currentPose, nextPose, welding);
    }

    if (!isPathFound) {
      ROS_WARN("No path found to go to start waypoint");
      return false;
    }
  } else {
    ROS_WARN("No IK solutions found");
    return false;
  }

  return true;
}

void MAMPlanner::executeTrajectory() {
  bool success = false;
  vector<double> firstJointConfig{};
  cout << "Executing trajectory" << endl;

  if (bestPlan_.empty()) {
    ROS_ERROR("No trajectory to execute");
    return;
  }

  for (const auto& trajectory : bestPlan_) {
    firstJointConfig = trajectory.joint_trajectory.points[0].positions;

    if (!robot_->isAtJointPosition(firstJointConfig)) {
      moveGroup_->setJointValueTarget(firstJointConfig);
      success = (moveGroup_->move() == moveit::core::MoveItErrorCode::SUCCESS);

      if (!success) {
        ROS_ERROR("Failed to move to the starting joint configuration.");
        return;
      }
    }

    moveGroup_->stop();
    moveGroup_->clearPoseTargets();
    success = moveGroup_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;

    if (success) {
      ROS_INFO("Trajectory executed successfully");
    } else {
      ROS_ERROR("Failed to execute trajectory");
    }

    moveGroup_->stop();
    moveGroup_->clearPoseTargets();
  }
}

void MAMPlanner::initMoveit_() {
  const string robotGroup = "manipulator";
  ros::Duration timeout(2.0);

  try {
    moveGroup_ = make_unique<moveit::planning_interface::MoveGroupInterface>(
        robotGroup, make_shared<tf2_ros::Buffer>(), timeout);
  } catch (const runtime_error& e) {
    ROS_ERROR("Failed to initialize MoveGroupInterface: %s", e.what());
    return;
  }

  setupMovegroup_();
  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

void MAMPlanner::setupMovegroup_() {
  moveGroup_->setPoseReferenceFrame(robot_->getReferenceFrame());
  moveGroup_->setPlannerId("RRTConnect");
  moveGroup_->setPlanningTime(2.0);
  moveGroup_->setNumPlanningAttempts(10);
  moveGroup_->setGoalPositionTolerance(0.005);
  moveGroup_->setGoalOrientationTolerance(0.01);
}

bool MAMPlanner::computePath_(const vector<double>& startConfig,
                              const geometry_msgs::Pose& currentPose,
                              const geometry_msgs::Pose& targetPose,
                              const bool isWeldging) {
  bool success = false;
  const string robotGroup = "manipulator";
  moveit_msgs::RobotTrajectory planCartesianTrajectory{};
  moveGroup_->clearPoseTargets();

  // Create a RobotState object and set it to the desired starting joint configuration
  moveit::core::RobotState startState(*moveGroup_->getCurrentState());
  startState.setJointGroupPositions(robotGroup, startConfig);

  // Set the starting state in the planning scene
  moveGroup_->setStartState(startState);

  if (isWeldging) {
    // Compute Cartesian path
    vector<geometry_msgs::Pose> waypoints{};
    waypoints.push_back(currentPose);
    waypoints.push_back(targetPose);

    double fraction = moveGroup_->computeCartesianPath(waypoints, 0.01, planCartesianTrajectory, true);
    success = fraction == 1.0; // Path fully computed
    cout << "Fraction: " << fraction << endl;
  } else {
    // Set the target pose
    moveGroup_->setPoseTarget(targetPose);

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = moveGroup_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    planCartesianTrajectory = plan.trajectory_;
  }

  if (success) {
    if (currentWPointID_ >= bestPlan_.size()) {
      bestPlan_.push_back(planCartesianTrajectory);
    } else {
      double currentPlanSize = planCartesianTrajectory.joint_trajectory.points.size();
      double bestPlanSize = bestPlan_[currentWPointID_].joint_trajectory.points.size();

      if (currentPlanSize < bestPlanSize) {
        bestPlan_[currentWPointID_] = planCartesianTrajectory;
      }
    }
  }

  return success;
}

geometry_msgs::Pose MAMPlanner::projectPose_(const geometry_msgs::Pose& pose,
                                             const string& fromFrame,
                                             const string& toFrame) {
  geometry_msgs::PoseStamped inputPose;
  inputPose.pose = pose;
  inputPose.header.frame_id = fromFrame;
  inputPose.header.stamp = ros::Time::now();

  try {
    geometry_msgs::TransformStamped transformStamped =
        tfBuffer_.lookupTransform(toFrame, fromFrame, ros::Time(0), ros::Duration(1.0));
    geometry_msgs::PoseStamped outputPose;

    tf2::doTransform(inputPose, outputPose, transformStamped);
    return outputPose.pose;
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Could NOT transform: %s", ex.what());
    return pose;
  }
}

void MAMPlanner::createNewFrame_(const string& parentFrame,
                                 const string& newFrame,
                                 const geometry_msgs::Transform& transform) {
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parentFrame;
  transformStamped.child_frame_id = newFrame;
  transformStamped.transform = transform;

  br_.sendTransform(transformStamped);
}

void MAMPlanner::publishWaypointRviz_(const geometry_msgs::Pose& pose, const std::string& frameId) {
  float TIME_WAIT = 0.2;
  size_t NB_PUBLISH = 3;

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = frameId;
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.pose = pose;

  for (size_t i = 0; i < NB_PUBLISH; ++i) {
    pubWaypointRviz_.publish(poseStamped);
    ros::Duration(TIME_WAIT).sleep();
  }
}

// Publish the geometry::PoseArry waypoint to CoppeliaSim
void MAMPlanner::publishWaypointCoppeliasim_(const geometry_msgs::Pose& pose, const std::string& frameId) {
  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = frameId;
  poseArray.header.stamp = ros::Time::now();
  poseArray.poses.push_back(pose);

  pubWaypointCoppeliasim_.publish(poseArray);
}
