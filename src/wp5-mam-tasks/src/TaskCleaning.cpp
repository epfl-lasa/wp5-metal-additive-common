#include "TaskCleaning.h"

using namespace std;
using namespace Eigen;

TaskCleaning::TaskCleaning(ros::NodeHandle& nh, double freq, string robotName) : ITaskBase(nh, freq, robotName) {
  ros::NodeHandle nodeHandle = getRosNodehandle_();

  // Create an unique pointer for the instance of PathPlanner
  pathPlanner_ = make_unique<PathPlanner>(nodeHandle);
}

bool TaskCleaning::computePath() {
  cout << "computing path ..." << endl;
  return true;
}

bool TaskCleaning::execute() {
  cout << "preforming cleaning ..." << endl;
  return true;
}
