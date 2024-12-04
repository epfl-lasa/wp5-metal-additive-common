#include "ITaskBase.h"

#include "RoboticArmFactory.h"
#include "conversion_tools.h"
#include "yaml_tools.h"

using namespace std;

ITaskBase::ITaskBase(ros::NodeHandle& nh, const YAML::Node& config) :
    nh_(nh),
    robotName_(YamlTools::loadYamlValue<string>(config, "robot_name")),
    rosVersion_(IRosInterfaceBase::rosVersionsMap.at(YamlTools::loadYamlValue<string>(config, "ros_version"))),
    homeConfig_(YamlTools::loadYamlValue<vector<double>>(config, "home_configuration")),
    eePoseScan_(YamlTools::loadYamlValue<vector<double>>(config, "ee_pose_scan")),
    eePoseWorkOffset_(YamlTools::loadYamlValue<vector<double>>(config, "ee_pose_work_offset")),
    eePoseOffset_(YamlTools::loadYamlValue<vector<double>>(config, "ee_pose_offset")) {}

bool ITaskBase::initialize() {
  planner_ = make_unique<MAMPlanner>(rosVersion_, nh_, robotName_);

  return true;
}

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
