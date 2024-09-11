// clang off
#include "ITaskBase.h"
// clang on

#include "RoboticArmFactory.h"

using namespace std;

ITaskBase::ITaskBase(ros::NodeHandle& nh, ROSVersion rosVersion, double freq, string robotName) :
    nh_(nh), rosVersion_(rosVersion), rosFreq_(freq), loopRate_(freq) {}

bool ITaskBase::initialize() {
  planner_ = make_unique<MAMPlanner>(rosVersion_);

  return true;
}

bool ITaskBase::goHomingPosition() {
  cout << "Go Homing Position" << endl;
  return true;
}

bool ITaskBase::goWorkingPosition() {
  cout << "Go Working Position" << endl;
  return true;
}
