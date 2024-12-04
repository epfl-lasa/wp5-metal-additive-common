#include "ITaskBase.h"

#include "RoboticArmFactory.h"
#include "yaml_tools.h"

using namespace std;

ITaskBase::ITaskBase(ros::NodeHandle& nh, const YAML::Node& config) :
    nh_(nh),
    robotName_(YamlTools::loadYamlValue<string>(config, "robot_name")),
    rosVersion_(IRosInterfaceBase::rosVersionsMap.at(YamlTools::loadYamlValue<string>(config, "ros_version"))),
    homeConfig_(YamlTools::loadYamlValue<vector<double>>(config, "home_configuration")),
    eePoseScan(YamlTools::loadYamlValue<vector<double>>(config, "ee_pose_scan")),
    eePoseWorkOffset(YamlTools::loadYamlValue<vector<double>>(config, "ee_pose_work_offset")),
    eePoseOffset(YamlTools::loadYamlValue<vector<double>>(config, "ee_pose_offset")) {}

bool ITaskBase::initialize() {
  planner_ = make_unique<MAMPlanner>(rosVersion_, nh_, robotName_);

  return true;
}

bool ITaskBase::scanArea() {
  cout << "Scan Area" << endl;
  return true;
}

bool ITaskBase::goHomingPosition() {
  cout << "Go Homing Position" << endl;
  return true;
}

bool ITaskBase::goWorkingPosition() {
  cout << "Go Working Position" << endl;
  return true;
}
