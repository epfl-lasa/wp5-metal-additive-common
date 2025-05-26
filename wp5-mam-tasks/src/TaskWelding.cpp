/**
 * @file TaskWelding.cpp
 * @brief Declaration of the TaskWelding class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.3
 * @date 2025-01-31
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#include "TaskWelding.h"

#include "PlannerWelding.h"
#include "conversion_tools.h"
#include "debug_tools.h"
#include "yaml_tools.h"

using namespace std;

TaskWelding::TaskWelding(ros::NodeHandle& nh, string configFilename) :
    ITaskBase(nh, YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_TASKS_DIR)))["welding"]) {}

bool TaskWelding::initialize() {
  planner_ = make_unique<PlannerWelding>(rosVersion_, nh_, robotName_, workingSpeed_);
  return true;
}

bool TaskWelding::computeTrajectory(const std::vector<ROI::Pose>& waypoints) {
  std::vector<geometry_msgs::Pose> waypointsToPlan{};

  // Compute needed vectors normal, waypoint and offset direction
  const Eigen::Vector3d wpVector = (waypoints[1].getPosition() - waypoints[0].getPosition()).normalized();

  // Add moving to welding pose
  waypointsToPlan.push_back(getPoseOffset_(waypoints.front(), wpVector, eePosWorkOffset_));

  // Add welding waypoints
  for (size_t i = 0; i < waypoints.size(); ++i) {
    waypointsToPlan.push_back(getPoseOffset_(waypoints[i], wpVector, eePosOffset_));
  }

  // Add moving away from welding pose
  waypointsToPlan.push_back(getPoseOffset_(waypoints.back(), wpVector, eePosWorkOffset_));

#ifdef DEBUG_MODE
  DebugTools::publishPath("base_link", waypointsToPlan, pathPub_);
#endif

  return planner_->planTrajectory(waypointsToPlan);
}

bool TaskWelding::execute() { return planner_->executeTrajectory(); }

// TODO(lmunier) - Double check the angle offset orientation with ANiMA to ensure the correct end effector orientation
const geometry_msgs::Pose TaskWelding::getPoseOffset_(const ROI::Pose waypoint,
                                                      const Eigen::Vector3d wpVector,
                                                      const Eigen::Vector3d offset) {
  const Eigen::Vector3d offsetDir = waypoint.getNormal().normalized();

  // Define orientation matrix to face the welding direction
  // const Eigen::Vector3d vy = wpVector.normalized();
  // here set vy always is (0,0,-1)
  const Eigen::Vector3d vy = Eigen::Vector3d(0, 0, -1);

  const Eigen::Vector3d vz = -offsetDir;
  const Eigen::Vector3d vx = vy.cross(vz);

  Eigen::Matrix3d faceMatrix;
  faceMatrix << vx, vy, vz;

  // Extract quaternions from current settings
  const Eigen::Quaterniond faceQuaternion(faceMatrix);
  const Eigen::Quaterniond rotation = MathTools::getQuatFromNormalTheta(vx, workingAngle_);

  // Add offsets from both tools size and welding needs
  pair<Eigen::Quaterniond, Eigen::Vector3d> offsetPoseQuatVec = MathTools::addOffset(
      pair<Eigen::Quaterniond, Eigen::Vector3d>(Eigen::Quaterniond::Identity(), waypoint.getPosition()),
      pair<Eigen::Quaterniond, Eigen::Vector3d>(faceQuaternion, offset));

  offsetPoseQuatVec = MathTools::addOffset(
      offsetPoseQuatVec,
      pair<Eigen::Quaterniond, Eigen::Vector3d>(rotation, ConversionTools::extractVector(toolTransform_)));

  return ConversionTools::eigenToGeometry(offsetPoseQuatVec.first, offsetPoseQuatVec.second);
}