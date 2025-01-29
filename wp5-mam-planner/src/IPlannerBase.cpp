/**
 * @file IPlannerBase.cpp
 * @brief Declaration of the IPlannerBase class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2025-01-24
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */
#include "IPlannerBase.h"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <filesystem>
#include <fstream>

#include "RoboticArmFactory.h"
#include "conversion_tools.h"
#include "debug_tools.h"
#include "laser_service/ManageLaser.h"
#include "math_tools.h"
#include "yaml_tools.h"

using namespace std;
namespace fs = std::filesystem;

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

  // TODO(lmunier): Check if the orientation constraints are necessary
  // setOrientationConstraints_();

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
  std::sort(sortedWeldingPaths_.begin(),
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
  std::reverse(points.begin(), points.end());

  // Reassign the time_from_start values in ascending order
  for (size_t i = 0; i < points.size(); ++i) {
    points[i].time_from_start = time_from_start_values[i];
  }
}

vector<trajectory_msgs::JointTrajectoryPoint> IPlannerBase::interpolatePoints_(
    const trajectory_msgs::JointTrajectoryPoint& prevPoint,
    const trajectory_msgs::JointTrajectoryPoint& currPoint,
    double timeStep,
    double timeInterval) {
  vector<trajectory_msgs::JointTrajectoryPoint> interpolatedPoints;

  for (double t = timeStep; t < timeInterval; t += timeStep) {
    const double alpha = t / timeInterval;
    trajectory_msgs::JointTrajectoryPoint interpolatedPoint;
    interpolatedPoint.positions.resize(prevPoint.positions.size());

    std::transform(prevPoint.positions.begin(),
                   prevPoint.positions.end(),
                   currPoint.positions.begin(),
                   interpolatedPoint.positions.begin(),
                   [alpha](double prev, double curr) { return prev + alpha * (curr - prev); });

    interpolatedPoint.time_from_start = ros::Duration(t);
    interpolatedPoints.push_back(std::move(interpolatedPoint));
  }

  return interpolatedPoints;
}

bool IPlannerBase::retimeTrajectory_(moveit_msgs::RobotTrajectory& trajectory,
                                     double cartesianSpeed,
                                     double robotFrequency) {
  // Ensure the trajectory has points
  if (trajectory.joint_trajectory.points.empty()) {
    ROS_WARN("[IPlannerBase] - Trajectory has no points to retime.");
    return false;
  }

  // Calculate the desired time step based on the robot's frequency
  const double timeStep = 1.0 / robotFrequency; // TODO(lmunier) : Check if needed or if Moveit! already does it

  // Create a new trajectory to store the interpolated points
  moveit_msgs::RobotTrajectory newTrajectory;
  newTrajectory.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;

  // Add the first point to the new trajectory
  newTrajectory.joint_trajectory.points.push_back(trajectory.joint_trajectory.points.front());

  // Linearly interpolate between each pair of points
  for (size_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i) {
    const auto& prevPoint = trajectory.joint_trajectory.points[i - 1];
    const auto& currPoint = trajectory.joint_trajectory.points[i];

    // Calculate the Euclidean distance between waypoints
    tf2::Vector3 prevPosition(prevPoint.positions[0], prevPoint.positions[1], prevPoint.positions[2]);
    tf2::Vector3 currPosition(currPoint.positions[0], currPoint.positions[1], currPoint.positions[2]);
    const double distance = prevPosition.distance(currPosition);

    // Calculate the time interval for the desired speed
    const double timeInterval = distance / cartesianSpeed;

    // Interpolate points at the desired time step
    auto interpolatedPoints = interpolatePoints_(prevPoint, currPoint, timeStep, timeInterval);
    for (auto& point : interpolatedPoints) {
      point.time_from_start += newTrajectory.joint_trajectory.points.back().time_from_start;
      newTrajectory.joint_trajectory.points.push_back(std::move(point));
    }

    // Add the current point to the new trajectory
    auto currPointCopy = currPoint;
    currPointCopy.time_from_start =
        newTrajectory.joint_trajectory.points.back().time_from_start + ros::Duration(timeInterval);
    newTrajectory.joint_trajectory.points.push_back(std::move(currPointCopy));
  }

  // Replace the original trajectory with the new trajectory
  trajectory = std::move(newTrajectory);

  ROS_INFO_STREAM("[IPlannerBase] - Trajectory retimed and interpolated successfully to match the robot frequency of "
                  << robotFrequency << " Hz and achieve a constant Cartesian speed of " << cartesianSpeed << " m/s.");

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

// TODO(lmunier): Check if the orientation constraints are necessary
void IPlannerBase::setOrientationConstraints_() {
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "tool0";                           // The link to which the constraint is applied
  ocm.header.frame_id = robot_->getReferenceFrame(); // The reference frame for the orientation

  ocm.orientation.x = -0.707;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 0.707;

  ocm.absolute_x_axis_tolerance = 2.8798; // Allowed deviation in radians for x-axis
  ocm.absolute_y_axis_tolerance = 2.8798; // Allowed deviation in radians for y-axis
  ocm.absolute_z_axis_tolerance = 2.8798; // Allowed deviation in radians for z-axis
  ocm.weight = 1.0;                       // Importance of this constraint

  moveit_msgs::Constraints constraints;
  constraints.orientation_constraints.push_back(ocm);

  moveGroup_->setPathConstraints(constraints);
}

bool IPlannerBase::move_() {
  moveit::planning_interface::MoveGroupInterface::Plan currentPlan;

  // TODO(lmunier): Check if the orientation constraints are necessary
  // setOrientationConstraints_();
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

//TODO(lmunier) to implement the following function
// void publishContinuously(ros::Publisher& publisher, std::atomic<bool>& running) {
//   ros::Rate rate(10.0); // 10 Hz
//   std_msgs::Bool msg;
//   msg.data = true;

//   while (running.load() && ros::ok()) {
//     publisher.publish(msg);
//     rate.sleep();
//   }
// }
