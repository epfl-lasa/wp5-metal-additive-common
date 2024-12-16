/**
 * @file TaskCleaning.h
 * @brief Declaration of the TaskCleaning class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */

#include "ITaskBase.h"

#include "RoboticArmFactory.h"
#include "conversion_tools.h"
#include "math_tools.h"
#include "yaml_tools.h"

using namespace std;

ITaskBase::ITaskBase(ros::NodeHandle& nh, const YAML::Node& config) :
    nh_(nh),
    robotName_(YamlTools::loadYamlValue<string>(config, "robot_name")),
    rosVersion_(IRosInterfaceBase::rosVersionsMap.at(YamlTools::loadYamlValue<string>(config, "ros_version"))),
    homeConfig_(YamlTools::loadYamlValue<vector<double>>(config, "home_configuration")),
    workingAngle_(YamlTools::loadYamlValue<double>(config, "working_angle")),
    eePosOffset_(YamlTools::loadYamlValue<Eigen::Vector3d>(config, "ee_pos_offset")),
    eePosWorkOffset_(YamlTools::loadYamlValue<Eigen::Vector3d>(config, "ee_pos_work_offset")),
    eePoseScan_(YamlTools::loadYamlValue<vector<double>>(config, "ee_pose_scan")),
    transform_(getTransform_("ee_tool", "virtual_link")) {}

bool ITaskBase::scanArea() {
  geometry_msgs::Pose scanPose = ConversionTools::vectorToGeometryPose(eePoseScan_);
  bool success = planner_->goToPose(scanPose);

  //TODO(lmunier) - Add scanning logic
  // Open camera lid
  // Scan area if any scan pattern
  // Close camera lid
  // for now, just a simple pause to publish the toy data
  ros::Duration(5.0).sleep();

  return success;
}

bool ITaskBase::goHomingConfiguration() { return planner_->goToJointConfig(homeConfig_); }

bool ITaskBase::goWorkingPosition() {
  cout << "Go Working Position" << endl;
  return true;
}

const geometry_msgs::Transform ITaskBase::getTransform_(const std::string& sourceFrame,
                                                        const std::string& targetFrame) {
  geometry_msgs::TransformStamped transformStamped;
  bool success = MathTools::getTransform(sourceFrame, targetFrame, transformStamped);

  if (!success) {
    ROS_ERROR_STREAM("[ITaskBase] - Failed to get transform from " << sourceFrame << " to " << targetFrame);
    return geometry_msgs::Transform();
  }

#ifdef DEBUG_MODE
  ROS_INFO_STREAM("[ITaskBase] - Got transform from " << sourceFrame << " to " << targetFrame);
  ROS_INFO_STREAM("[ITaskBase] - " << DebugTools::getTransformString(transformStamped));
#endif

  return transformStamped.transform;
}
