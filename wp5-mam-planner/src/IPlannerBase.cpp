/**
 * @file IPlannerBase.cpp
 * @brief Declaration of the IPlannerBase class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.3
 * @date 2025-01-31
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */
#include "IPlannerBase.h"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <filesystem>
#include <fstream>

#include "RoboticArmFactory.h"
#include "conversion_tools.h"
#include "debug_tools.h"
#include "laser_service/ManageLaser.h"
#include "math_tools.h"
#include "yaml_tools.h"

using namespace std;
namespace fs = filesystem;

/**
 * TODO(lmunier) : Clean store / read / plan configuration
 * - implement safety on the store / read behavior folder, filename and so on ...
 * - take care about the namespaces
 */

IPlannerBase::IPlannerBase(ROSVersion rosVersion, ros::NodeHandle& nh, string robotName, double workingSpeed) :
    nh_(nh), workingSpeed_(workingSpeed) {
  robot_ = RoboticArmFactory::createRoboticArm(robotName, rosVersion);
  initMoveit_();

  laserClient_ = nh.serviceClient<laser_service::ManageLaser>("/wp5_laser_service_node/manage");

#ifdef DEBUG_MODE
  pubTrajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("debug_trajectory", 1000);
#endif

  // Get trajectories strategy
  string tmpTrajFilename = "";
  nh_.getParam("/trajectories_strategy", trajectoriesStrat_);
  nh_.getParam("/trajectories_filename", tmpTrajFilename);

  trajectoriesDirectory_ = string(WP5_MAM_PLANNER_DIR) + "/trajectories/";
  trajectoriesFilename_ = trajectoriesDirectory_ + tmpTrajFilename;
  ROS_INFO_STREAM("Trajectory to store: " << trajectoriesStrat_ << " in " << trajectoriesFilename_);

  // Add obstacles
  obstacles_ = make_unique<ObstaclesManagement>(ObstaclesManagement(nh_, moveGroup_->getPlanningFrame()));
  obstacles_->addStaticObstacles();
}

bool IPlannerBase::planTrajectory(const vector<geometry_msgs::Pose>& waypoints) {
  bool success = false;
  string weldingState = "";

  if (trajectoriesStrat_ != "read") {
    success = planTrajectoryTask_(waypoints);

    if (trajectoriesStrat_ == "store" && success) {
      int i = 0;
      for (const auto& trajTask : trajTaskToExecute_) {
        i++;

        weldingState = trajTask.second ? "_welding" : "";
        success = saveTrajectory_(trajTask.first, trajectoriesFilename_ + to_string(i) + weldingState + ".yaml");

        if (!success) {
          ROS_ERROR_STREAM("[IPlannerBase] - Failed to save trajectory " << i);
        }
      }
    }
  } else {
    success = loadAllTrajectories_(trajectoriesDirectory_);
  }

  return success;
}

bool IPlannerBase::executeTrajectory() {
  bool success = false;
  int currentStep = 0;
  vector<double> firstJointConfig{};

  if (trajTaskToExecute_.empty()) {
    ROS_ERROR("[IPlannerBase] - No trajectory to execute");
    return success;
  }

  cleanMoveGroup_();
  for (const auto& trajTask : trajTaskToExecute_) {
    currentStep++;

#ifdef DEBUG_MODE
    ROS_INFO_STREAM("[IPlannerBase] - Welding state at step " << currentStep << " : " << trajTask.second);
#endif

    // Call laser service disabling before waiting on user feedback or robot positionning for the next step
    if (!trajTask.second) {
      if (!manageLaser_(trajTask.second)) {
        return false;
      }
    }

    // Check the first joint configuration to be able to begin trajectory execution
    firstJointConfig = trajTask.first.joint_trajectory.points.front().positions;

    if (!robot_->isAtJointPosition(firstJointConfig)) {
      // Disable laser welding first
      if (!manageLaser_(false)) {
        return false;
      }

      success = goToJointConfig(firstJointConfig);

      if (!success) {
        ROS_ERROR("[IPlannerBase] - Failed to move to the first joint configuration");
        return success;
      }
    }

#ifdef DEBUG_MODE
    string userMsg =
        "[IPlannerBase] - Execute trajectory at step " + to_string(currentStep) + ". Press Enter to continue.";

    DebugTools::publishTrajectory(*moveGroup_->getCurrentState(), trajTask.first, pubTrajectory_);
    DebugTools::waitOnUser(userMsg);
#endif

    // Call laser service enabling after waiting on user feedback or robot positionning for the next step
    if (trajTask.second) {
      if (!manageLaser_(trajTask.second)) {
        return false;
      }
    }

    // Execute the trajectory
    success = (moveGroup_->execute(trajTask.first) == moveit::core::MoveItErrorCode::SUCCESS);
    cleanMoveGroup_();

    if (!success) {
      ROS_ERROR_STREAM("[IPlannerBase] - Failed to execute trajectory at step " << currentStep);
      return success;
    }

    ROS_INFO("[IPlannerBase] - Trajectory executed successfully");
  }

  return success;
}

bool IPlannerBase::planCartesianFromJointConfig(const vector<double>& startJointConfig,
                                                const vector<geometry_msgs::Pose>& waypoints,
                                                vector<moveit_msgs::RobotTrajectory>& pathPlanned) {
  bool success = false;

  // Reset the move group interface
  cleanMoveGroup_();

  const double eef_step = 0.01;
  const string robotGroup = "manipulator";
  moveit_msgs::RobotTrajectory planTrajectory;

  // Create a RobotState object and set it to the desired starting joint configuration
  moveit::core::RobotState startState(*moveGroup_->getCurrentState());
  const moveit::core::JointModelGroup* jointModelGroup = startState.getJointModelGroup(robotGroup);
  startState.setJointGroupPositions(jointModelGroup, startJointConfig);
  moveGroup_->setStartState(startState);

  // Compute Cartesian path
  moveit_msgs::MoveItErrorCodes error_code;
  success = (moveGroup_->computeCartesianPath(waypoints, eef_step, planTrajectory, true) == 1.0);

  if (success) {
    ROS_INFO("[IPlannerBase] - Cartesian path computed successfully.");
    pathPlanned.push_back(planTrajectory);
  } else {
    ROS_ERROR_STREAM("[IPlannerBase] - Failed to compute the Cartesian path. Error code: " << error_code.val);
  }

  return success;
}

bool IPlannerBase::goToScanArea(const vector<double>& eePoseScan) {
  bool success = false;
  vector<vector<double>> ikSolutions{};
  pair<Eigen::Quaterniond, Eigen::Vector3d> scanEigenPair = ConversionTools::vectorToEigenQuatPose(eePoseScan);
  success = robot_->getIKGeo(scanEigenPair.first, scanEigenPair.second, ikSolutions);

  if (!success) {
    ROS_ERROR("[IPlannerBase] - Failed to get IK solutions for scanning pose.");
    return false;
  }

  // Loop over the IK solutions and find the one that is not colliding with obstacles and does not perform a big move
  bool foundValidConfig = false;
  const vector<double> currentJointPos = get<0>(robot_->getState());

  removeHighJointMoves(ikSolutions);
  for (auto& sol : ikSolutions) {
    success = goToJointConfig(sol);

    if (success) {
      break;
    }
  }

  return success;
}

bool IPlannerBase::goToJointConfig(const vector<double>& jointConfig) {
  cleanMoveGroup_();

  moveGroup_->setJointValueTarget(jointConfig);
  bool success = move_();

  if (!success) {
    string jointConfigStr = DebugTools::getVecString<double>(jointConfig);

    ROS_ERROR_STREAM("[IPlannerBase] - Failed to move to the joint configuration " << jointConfigStr);
  }

  return success;
}

bool IPlannerBase::goToPose(const geometry_msgs::Pose& targetPose) {
  cleanMoveGroup_();

  moveGroup_->setPoseTarget(targetPose);
  bool success = move_();

  if (!success) {
    string targetPoseStr = DebugTools::getPoseString(targetPose);

    ROS_ERROR_STREAM("[IPlannerBase] - Failed to move to the target pose " << targetPoseStr);
  }

  return success;
}

void IPlannerBase::removeHighJointMoves(vector<vector<double>>& jointPos, const double limitJointMove) {
  vector<double> currentConfig = get<0>(robot_->getState());

  for (auto it = jointPos.begin(); it != jointPos.end();) {
    bool remove = false;
    for (size_t i = 0; i < it->size(); ++i) {
      if (abs((*it)[i] - currentConfig[i]) > limitJointMove) {
        remove = true;
        break;
      }
    }
    if (remove) {
      it = jointPos.erase(it); // Remove the element and get the next iterator
    } else {
      ++it; // Move to the next element
    }
  }
}

bool IPlannerBase::saveTrajectory_(const moveit_msgs::RobotTrajectory& trajectory, const string& filename) {
  YAML::Emitter out;
  out << YAML::BeginSeq;
  for (const auto& point : trajectory.joint_trajectory.points) {
    out << YAML::BeginMap;
    out << YAML::Key << "positions" << YAML::Value << YAML::Flow << point.positions;
    out << YAML::Key << "velocities" << YAML::Value << YAML::Flow << point.velocities;
    out << YAML::Key << "accelerations" << YAML::Value << YAML::Flow << point.accelerations;
    out << YAML::Key << "effort" << YAML::Value << YAML::Flow << point.effort;
    out << YAML::Key << "time_from_start" << YAML::Value << point.time_from_start.toSec();
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  ofstream file(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("[IPlannerBase] - Failed to open file: " << filename);
    return false;
  }

  file << out.c_str();
  file.close();
  ROS_INFO_STREAM("[IPlannerBase] - Trajectory saved to: " << filename);
  return true;
}

bool IPlannerBase::loadTrajectory_(moveit_msgs::RobotTrajectory& trajectory, const string& filename) {
  ifstream file(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("[IPlannerBase] - Failed to open file: " << filename);
    return false;
  }

  YAML::Node node = YAML::Load(file);
  file.close();

  trajectory.joint_trajectory.header.frame_id = robot_->getReferenceFrame();
  trajectory.joint_trajectory.joint_names = moveGroup_->getVariableNames();

  for (const auto& j_point : node) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = j_point["positions"].as<vector<double>>();
    point.velocities = j_point["velocities"].as<vector<double>>();
    point.accelerations = j_point["accelerations"].as<vector<double>>();
    point.effort = j_point["effort"].as<vector<double>>();
    point.time_from_start = ros::Duration(j_point["time_from_start"].as<double>());

    trajectory.joint_trajectory.points.push_back(point);
  }

  ROS_INFO_STREAM("[IPlannerBase] - Trajectory loaded from: " << filename);
  return true;
}

bool IPlannerBase::loadAllTrajectories_(const string& directory) {
  bool success = false;
  bool weldingState = false;

  for (const auto& entry : fs::directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
      moveit_msgs::RobotTrajectory trajectory;
      success = loadTrajectory_(trajectory, entry.path().string());

      // If the trajectory contains "welding", put the welding state to true
      weldingState = entry.path().string().find("welding") != string::npos;

      if (success) {
        trajTaskToExecute_.emplace_back(trajectory, weldingState);
      } else {
        return success;
      }
    }
  }

  return success;
}

bool IPlannerBase::extractJointConfig_(const moveit_msgs::RobotTrajectory& trajectory,
                                       vector<double>& jointConfig,
                                       const ConfigPosition position) {
  jointConfig.clear();
  if (trajectory.joint_trajectory.points.empty()) {
    ROS_WARN("[IPlannerBase] - The trajectory is empty.");
    return false;
  }

  const trajectory_msgs::JointTrajectoryPoint& point = (position == ConfigPosition::FIRST)
                                                           ? trajectory.joint_trajectory.points.front()
                                                           : trajectory.joint_trajectory.points.back();
  jointConfig = point.positions;
  return true;
}

double IPlannerBase::computeTotalJerk_(const moveit_msgs::RobotTrajectory& trajectory) {
  const int MIN_POINTS = 3;
  double totalJerk = 0.0;

  const vector<trajectory_msgs::JointTrajectoryPoint>& points = trajectory.joint_trajectory.points;
  if (points.size() < MIN_POINTS) {
    ROS_WARN_STREAM("[IPlannerBase] - Not enough points to compute jerk : current size of " << points.size());
    return totalJerk;
  }

  for (size_t i = 1; i < points.size() - 1; ++i) {
    const trajectory_msgs::JointTrajectoryPoint& prev = points[i - 1];
    const trajectory_msgs::JointTrajectoryPoint& curr = points[i];
    const trajectory_msgs::JointTrajectoryPoint& next = points[i + 1];

    Eigen::VectorXd velPrev = Eigen::VectorXd::Map(prev.velocities.data(), prev.velocities.size());
    Eigen::VectorXd velCurr = Eigen::VectorXd::Map(curr.velocities.data(), curr.velocities.size());
    Eigen::VectorXd velNext = Eigen::VectorXd::Map(next.velocities.data(), next.velocities.size());

    double timePrev = prev.time_from_start.toSec();
    double timeCurr = curr.time_from_start.toSec();
    double timeNext = next.time_from_start.toSec();

    Eigen::VectorXd accPrev = (velCurr - velPrev) / (timeCurr - timePrev);
    Eigen::VectorXd accNext = (velNext - velCurr) / (timeNext - timeCurr);

    Eigen::VectorXd jerk = (accNext - accPrev) / (timeNext - timePrev);

    totalJerk += jerk.norm();
  }

  return totalJerk;
}

void IPlannerBase::sortTrajectoriesByJerk_(vector<moveit_msgs::RobotTrajectory>& trajectories) {
  sort(sortedWeldingPaths_.begin(),
       sortedWeldingPaths_.end(),
       [this](const moveit_msgs::RobotTrajectory& a, const moveit_msgs::RobotTrajectory& b) {
         return computeTotalJerk_(a) < computeTotalJerk_(b);
       });
}

void IPlannerBase::reverseTrajectory_(moveit_msgs::RobotTrajectory& trajectory) {
  // Reverse the joint trajectory points
  vector<trajectory_msgs::JointTrajectoryPoint>& points = trajectory.joint_trajectory.points;

  // Copy the time_from_start values
  vector<ros::Duration> time_from_start_values;
  time_from_start_values.reserve(points.size());
  for (const auto& point : points) {
    time_from_start_values.push_back(point.time_from_start);
  }

  // Reverse the points
  reverse(points.begin(), points.end());

  // Reassign the time_from_start values in ascending order
  for (size_t i = 0; i < points.size(); ++i) {
    points[i].time_from_start = time_from_start_values[i];
  }
}

bool IPlannerBase::retimeTrajectory_(moveit_msgs::RobotTrajectory& trajectory, const double cartesianSpeed) {
  const string PLANNING_GROUP = "manipulator";

  // Ensure the trajectory has points
  if (trajectory.joint_trajectory.points.empty()) {
    ROS_WARN("[IPlannerBase] - Trajectory has no points to retime.");
    return false;
  }

  // Scale the Cartesian speed
  robot_trajectory::RobotTrajectory rt(moveGroup_->getCurrentState()->getRobotModel(), PLANNING_GROUP);
  rt.setRobotTrajectoryMsg(*moveGroup_->getCurrentState(), trajectory);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(rt, cartesianSpeed);

  rt.getRobotTrajectoryMsg(trajectory);

  return true;
}

void IPlannerBase::initMoveit_() {
  const string robotGroup = "manipulator";
  ros::Duration timeout(10.0);

  try {
    moveGroup_ = make_unique<moveit::planning_interface::MoveGroupInterface>(robotGroup);
  } catch (const runtime_error& e) {
    ROS_ERROR_STREAM("[IPlannerBase] - Failed to initialize MoveGroupInterface: " << e.what());
    return;
  }

  setupMoveGroup_();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

void IPlannerBase::setupMoveGroup_() {
  moveGroup_->setPoseReferenceFrame(robot_->getReferenceFrame());
  moveGroup_->setPlannerId("RRTConnect");
  moveGroup_->setPlanningTime(5.0);
  moveGroup_->setNumPlanningAttempts(20);
  moveGroup_->setGoalPositionTolerance(0.001);
  moveGroup_->setGoalOrientationTolerance(0.01);
}

void IPlannerBase::cleanMoveGroup_() {
  moveGroup_->stop();
  moveGroup_->clearPoseTargets();
  moveGroup_->setStartStateToCurrentState();
}

bool IPlannerBase::move_() {
  moveit::planning_interface::MoveGroupInterface::Plan currentPlan;
  bool success = (moveGroup_->plan(currentPlan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
#ifdef DEBUG_MODE
    string userMsg = "[IPlannerBase] - Path succes : " + to_string(success) + " Press Enter to continue.";
    const moveit_msgs::RobotTrajectory& robotTrajectory = currentPlan.trajectory_;

    DebugTools::publishTrajectory(*moveGroup_->getCurrentState(), robotTrajectory, pubTrajectory_);
    DebugTools::waitOnUser(userMsg);
#endif

    success = (moveGroup_->execute(currentPlan) == moveit::core::MoveItErrorCode::SUCCESS);
    cleanMoveGroup_();
  }

  return success;
}

bool IPlannerBase::manageLaser_(bool enable) {
  laser_service::ManageLaser msgLaser;
  msgLaser.request.enable = enable;

  ros::Duration timeout(20.0); // Add safety timeout

  ros::Time startTime = ros::Time::now();
  while (!laserClient_.call(msgLaser)) {
    if (ros::Time::now() - startTime > timeout) {
      ROS_ERROR("[IPlannerBase] - Timeout while waiting for laser service");
      return false;
    }

    ros::Duration(0.1).sleep();
  }

  return msgLaser.response.success;
}
