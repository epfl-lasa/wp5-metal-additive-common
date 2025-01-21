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
  double rosFreq = 125;
  ros::Rate loopRate(rosFreq);

  // Check for the taskType parameter
  string taskType = "";
  if (!ros::param::get("~taskType", taskType)) {
    ROS_ERROR("[MainTask] - No taskType argument received");
    return 1;
  }

  TaskManager TaskManager(nh, nh_private);
  ros::spin();

  return 0;
}
