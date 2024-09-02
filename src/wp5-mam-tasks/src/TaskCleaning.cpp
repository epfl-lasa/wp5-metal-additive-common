#include "TaskCleaning.h"

using namespace std;
using namespace Eigen;

TaskCleaning::TaskCleaning(ros::NodeHandle& nh, double freq, string robotName) : ITaskBase(nh, freq, robotName) {
  ros::NodeHandle nodeHandle = getRosNodehandle_();
}

bool TaskCleaning::computePath() {
  cout << "computing path ..." << endl;
  return true;
}

bool TaskCleaning::execute() {
  cout << "preforming cleaning ..." << endl;
  return true;
}
