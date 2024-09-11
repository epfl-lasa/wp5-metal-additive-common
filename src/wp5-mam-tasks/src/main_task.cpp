#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <fstream>
#include <string>
#include <tuple>

#include "IRoboticArmBase.h"
#include "RoboticArmUr5.h"
#include "RosInterfaceNoetic.h"
#include "TaskCleaning.h"
#include "TaskFSM.h"
#include "TaskFactory.h"
#include "TaskWelding.h"

using namespace std;

int main(int argc, char** argv) {
  bool valid = false;
  double rosFreq = 300;
  TaskFactory taskFactory;

  // Init ros
  ros::init(argc, argv, "wp5_mam_main_task_node");
  ros::NodeHandle nh;
  ros::Rate loopRate(rosFreq);

  string taskType;
  if (ros::param::get("~taskType", taskType)) {
    vector<string> allowed_values = taskFactory.getTaskTypes();

    if (find(allowed_values.begin(), allowed_values.end(), taskType) == allowed_values.end()) {
      ostringstream oss;
      copy(allowed_values.begin(), allowed_values.end() - 1, ostream_iterator<string>(oss, ", "));
      oss << allowed_values.back();

      ROS_ERROR("Invalid taskType: %s. Allowed values are %s.", taskType.c_str(), oss.str().c_str());
      return 1;
    }
  } else {
    ROS_ERROR("No taskType argument received");
    return 1;
  }

  // Load parameters from YAML file
  string alternativeYamlPath = string(WP5_TASKS_DIR) + "/config/robot_task_config.yaml";
  string yamlPath = string(WP5_TASKS_DIR) + "/../../config/robot_task_config.yaml";

  // Check if the alternative YAML file exists
  ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    cout << "Using general YAML file: " << yamlPath << endl;
  } else {
    yamlPath = alternativeYamlPath;
    cout << "Using local YAML file: " << yamlPath << endl;
  }

  // Load parameters from YAML file
  YAML::Node config = YAML::LoadFile(yamlPath);

  string robotName = config[taskType]["robot_name"].as<string>();

  // Create an unique pointer for the instance of TaskFSM
  ROS_INFO("Creating Task - %s", taskType.c_str());
  shared_ptr<ITaskBase> task = taskFactory.createTask(taskType, nh, rosFreq, robotName);

  taskFsm_ internalFSM_(task);

  // Initialize and test the FSM
  internalFSM_.start();
  internalFSM_.process_event(Start());
  internalFSM_.stop();
  cout << "TaskFSM is stopped" << endl;

  return 0;
}
