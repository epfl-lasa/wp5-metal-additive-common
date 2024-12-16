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
#include "debug_tools.h"
#include "yaml_tools.h"

using namespace std;

TaskWelding::TaskWelding(ros::NodeHandle& nh, string configFilename) :
    ITaskBase(nh, YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_TASKS_DIR)))["welding"]) {}

bool TaskWelding::initialize() {
  planner_ = make_unique<PlannerWelding>(rosVersion_, nh_, robotName_);
  return true;
}

bool TaskWelding::computeTrajectory(const std::vector<geometry_msgs::Pose>& waypoints) {
  std::vector<geometry_msgs::Pose> waypointsToPlan{};
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> poseOffset{};

  // Put waypoints in an array for convenience
  array<Eigen::Vector3d, 3> pointsArray;
  pointsArray[0] = ConversionTools::geometryToEigen(waypoints[0].position);
  pointsArray[1] = Eigen::Vector3d::Zero();
  pointsArray[2] = ConversionTools::geometryToEigen(waypoints[1].position);

  // Compute needed vectors normal, waypoint and offset direction
  const Eigen::Vector3d normalVector = MathTools::getNormalFromPlan(pointsArray);
  const Eigen::Vector3d wpVector = (pointsArray[2] - pointsArray[0]).normalized();
  const Eigen::Vector3d offsetVector = wpVector.cross(normalVector).normalized();

  const Eigen::Quaterniond rotation = MathTools::getQuatFromNormalTheta(-normalVector, workingAngle_);
  const Eigen::Quaterniond rotQuaternion = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), offsetVector);

  const Eigen::Quaterniond quatTransform = rotation * rotQuaternion;

  // Compute the different quaternion to find the offset orientation
  const geometry_msgs::Pose offsetVectorPose = ConversionTools::eigenToGeometry(rotQuaternion, Eigen::Vector3d::Zero());

  // Add offsets from both tools size and welding needs
  geometry_msgs::Pose offsetPose = MathTools::addOffset(
      offsetVectorPose, ConversionTools::eigenToGeometry(Eigen::Quaterniond::Identity(), eePosOffset_));
  geometry_msgs::Pose offsetPoseWork = MathTools::addOffset(
      offsetVectorPose, ConversionTools::eigenToGeometry(Eigen::Quaterniond::Identity(), eePosWorkOffset_));

  offsetPoseWork.orientation = ConversionTools::eigenToGeometry(quatTransform);
  offsetPose.orientation = ConversionTools::eigenToGeometry(quatTransform);

  // Add moving to welding pose
  waypointsToPlan.push_back(MathTools::addOffset(waypoints.front(), offsetPoseWork));

  // Add welding waypoints
  for (size_t i = 0; i < waypoints.size(); ++i) {
    waypointsToPlan.push_back(MathTools::addOffset(waypoints[i], offsetPose));
  }

  // Add moving away from welding pose
  waypointsToPlan.push_back(MathTools::addOffset(waypoints.back(), offsetPoseWork));

  return planner_->planTrajectory(waypointsToPlan);
}

bool TaskWelding::execute() { return planner_->executeTrajectory(); }
