#include "TaskCleaning.h"

using namespace std;
using namespace Eigen;

TaskCleaning::TaskCleaning(ros::NodeHandle& nh, double freq, string robotName) : ITaskBase(nh, freq, robotName) {
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
