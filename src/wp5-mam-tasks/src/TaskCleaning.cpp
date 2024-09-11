#include "TaskCleaning.h"

using namespace std;

TaskCleaning::TaskCleaning(ros::NodeHandle& nh, ROSVersion rosVersion, double freq, string robotName) :
    ITaskBase(nh, rosVersion, freq, robotName) {
  ros::NodeHandle nodeHandle = getRosNodehandle_();
}

bool TaskCleaning::computePath() {
  planner_->planTrajectory();
  return true;
}

bool TaskCleaning::execute() {
  planner_->executeTrajectory();
  return true;
}
