#include <ros/ros.h>

#include <string>

#include "TaskManager.hpp"

using namespace std;

int main(int argc, char** argv) {
  // Init ros
  ros::init(argc, argv, "task_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  bool valid = false;
  double rosFreq = 500;
  ros::Rate loopRate(rosFreq);

  // Check for the task_type parameter to not start the node if it is not set
  string taskType = "";
  if (!ros::param::get("~task_type", taskType)) {
    ROS_ERROR("[MainTask] - No task_type argument received");
    return 1;
  }

  TaskManager TaskManager(nh, nh_private);
  ros::spin();

  return 0;
}
