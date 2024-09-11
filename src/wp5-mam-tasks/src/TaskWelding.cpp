#include "TaskWelding.h"

using namespace std;
using namespace Eigen;

TaskWelding::TaskWelding(ros::NodeHandle& nh, double freq, string robotName) : ITaskBase(nh, freq, robotName) {
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
