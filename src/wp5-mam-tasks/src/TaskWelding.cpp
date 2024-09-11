#include "TaskWelding.h"

using namespace std;

TaskWelding::TaskWelding(ros::NodeHandle& nh, ROSVersion rosVersion, double freq, string robotName) :
    ITaskBase(nh, rosVersion, freq, robotName) {
  ros::NodeHandle nodeHandle = getRosNodehandle_();
}

bool TaskWelding::computePath() {
  planner_->planTrajectory();
  return true;
}

bool TaskWelding::execute() {
  planner_->executeTrajectory();
  return true;
}
