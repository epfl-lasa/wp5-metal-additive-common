/**
 * @file TaskCleaning.h
 * @brief Declaration of the TaskCleaning class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.3
 * @date 2025-01-30
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#include "ITaskBase.h"

#include <cstdlib>
#include <thread>

#include "RoboticArmFactory.h"
#include "conversion_tools.h"
#include "math_tools.h"
#include "std_srvs/Trigger.h"
#include "yaml_tools.h"

using namespace std;

ITaskBase::ITaskBase(ros::NodeHandle& nh, const YAML::Node& config) :
    nh_(nh),
    robotName_(YamlTools::loadYamlValue<string>(config, "robot_name")),
    rosVersion_(IRosInterfaceBase::rosVersionsMap.at(YamlTools::loadYamlValue<string>(config, "ros_version"))),
    homeConfig_(YamlTools::loadYamlValue<vector<double>>(config, "home_configuration")),
    workingAngle_(YamlTools::loadYamlValue<double>(config, "working_angle")),
    workingSpeed_(YamlTools::loadYamlValue<double>(config, "working_speed")),
    eePosOffset_(YamlTools::loadYamlValue<Eigen::Vector3d>(config, "ee_pos_offset")),
    eePosWorkOffset_(YamlTools::loadYamlValue<Eigen::Vector3d>(config, "ee_pos_work_offset")),
    eePoseScan_(YamlTools::loadYamlValue<vector<double>>(config, "ee_pose_scan")),
    toolTransform_(getTransform_("ee_tool", "virtual_link")) {}

bool ITaskBase::scanArea() {
  string robotType = "";
  string waypointsFile = "";
  bool toyWaypoints = false;
  bool success = planner_->goToScanArea(eePoseScan_);

  // Get ROS param
  nh_.getParam("/robot_type", robotType);
  nh_.getParam("/" + robotType + "/wp5_task_node/toy_waypoints", toyWaypoints);
  nh_.getParam("/" + robotType + "/wp5_task_node/waypoints_filename", waypointsFile);

  //TODO(lmunier) - Add scanning logic
  // Open camera lid
  // Scan area if any scan pattern
  // Close camera lid

  // Publish waypoints from handwritten file or call DTU ros service to detect them
  if (toyWaypoints) {
    ROS_INFO_STREAM("[ITaskBase] - Using toy data waypoints");

    // Start the toy data publisher in a separate thread and wait for it to finish
    std::string command = "roslaunch wp5_mam_tasks publish_waypoints.launch yaml_file:=" + waypointsFile;
    std::thread launch_thread([this, command]() { this->launchRosLaunchFile_(command); });
    launch_thread.join();
  } else {
    ROS_INFO_STREAM("[ITaskBase] - Using waypoints from DTU detection");

    // Call DTU service to detect waypoints
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("/ransac_node/execution/enable");
    std_srvs::Trigger srv;

    success = client.call(srv);
    if (!success) {
      ROS_ERROR_STREAM("[ITaskBase] - Failed to call service to detect waypoints");
    }
  }

  ROS_INFO_STREAM("[ITaskBase] - Scanning area DONE");

  return success;
}

bool ITaskBase::goHomingConfiguration() { return planner_->goToJointConfig(homeConfig_); }

bool ITaskBase::launchRosLaunchFile_(const std::string command) {
  bool success = std::system(command.c_str()) == 0;
  if (!success) {
    ROS_ERROR_STREAM("[ITaskBase] - Failed to publish waypoints");
  }

  return success;
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
