#include "TaskWelding.h"

using namespace std;
using namespace Eigen;

TaskWelding::TaskWelding(ros::NodeHandle& nh, double freq, string robotName) : ITaskBase(nh, freq, robotName) {
  ros::NodeHandle nodeHandle = getRosNodehandle_();

  // Create an unique pointer for the instance of PathPlanner
  pathPlanner_ = make_unique<PathPlanner>(nodeHandle);
}

bool TaskWelding::computePath() {
  cout << "computing path ..." << endl;
  return true;
}

bool TaskWelding::execute() {
  cout << "preforming welding ..." << endl;
  return true;
}
