#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <string>

#include "TaskFSM.h"
#include "TaskFactory.h"
#include "yaml_tools.h"

using namespace std;

int main(int argc, char** argv) {
  bool valid = false;
  TaskFactory taskFactory{};

  // Init ros and check for the needed parameter taskType
  ros::init(argc, argv, "wp5_mam_main_task_node");
  ros::NodeHandle nh;

  string taskType;
  if (!ros::param::get("~taskType", taskType)) {
    ROS_ERROR("No taskType argument received");
    return 1;
  }

  // Create an unique pointer for the instance of TaskFSM
  ROS_INFO("Creating Task - %s", taskType.c_str());
  shared_ptr<ITaskBase> task = taskFactory.createTask(taskType, nh, string("robot_task.yaml"));

  taskFsm_ internalFSM_(task);

  // Initialize and test the FSM
  internalFSM_.start();
  internalFSM_.process_event(Start());
  internalFSM_.stop();
  cout << "TaskFSM is stopped" << endl;

  return 0;
}
