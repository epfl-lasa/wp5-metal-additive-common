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

//TODO(lmunier) - Solve adding the offset to the waypoints, with the orientation
bool TaskWelding::computeTrajectory(const std::vector<geometry_msgs::Pose>& waypoints) {
  std::vector<geometry_msgs::Pose> waypointsToPlan{};
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> poseOffset{};

  // Apply the offset to the waypoints
  array<Eigen::Vector3d, 3> pointsArray;

  pointsArray[0] = ConversionTools::geometryToEigen(waypoints[0].position);
  pointsArray[1] = Eigen::Vector3d::Zero();
  pointsArray[2] = ConversionTools::geometryToEigen(waypoints[1].position);

  const Eigen::Vector3d normalVector = MathTools::getNormalFromPlan(pointsArray);
  //TODO(lmunier) - Change angle to params
  const Eigen::Quaterniond quatTransform = MathTools::getQuatFromNormalTheta(normalVector, MathTools::degToRad(20));

  ROS_INFO("Orientation QuatTransform: %s", DebugTools::getEigenString(quatTransform).c_str());

  const Eigen::Vector3d wpVector = pointsArray[2] - pointsArray[0];
  const Eigen::Vector3d offsetVecNormalized = normalVector.cross(wpVector).normalized();

  const Eigen::Vector3d offsetVecWork = offsetVecNormalized * *(eePoseWorkOffset_.end() - 1);
  const Eigen::Vector3d offsetVec = offsetVecNormalized * *(eePoseOffset_.end() - 1);

  const geometry_msgs::Pose offsetPoseWork = ConversionTools::eigenToGeometry(quatTransform, offsetVecWork);
  const geometry_msgs::Pose offsetPose = ConversionTools::eigenToGeometry(quatTransform, offsetVec);

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
