// clang off
#include "ITaskBase.h"
// clang on

#include "RoboticArmFactory.h"

using namespace std;
using namespace Eigen;

ITaskBase::ITaskBase(ros::NodeHandle& nh, double freq, string robotName) : nh_(nh), rosFreq_(freq), loopRate_(freq) {
  // Create an unique pointer for the instance of RosInterfaceNoetic
  rosInterface_ = make_unique<RosInterfaceNoetic>(robotName);

  // Create an unique pointer for the instance of RosInterfaceNoetic
  RoboticArmFactory armFactory = RoboticArmFactory();
  roboticArm_ = armFactory.createRoboticArm(robotName, ROSVersion::ROS1_NOETIC);
}

bool ITaskBase::initialize() {
  planner_ = make_unique<MAMPlanner>();

  return true;
}

double ITaskBase::getRosFrequency_() const { return rosFreq_; }

ros::Rate* ITaskBase::getRosLoopRate_() { return &loopRate_; }

ros::NodeHandle ITaskBase::getRosNodehandle_() const { return nh_; }

vector<double> ITaskBase::getHomeJoint_() const { return homeJoint_; }

void ITaskBase::setHomeJoint_(vector<double> desiredJoint) { homeJoint_ = desiredJoint; }

bool ITaskBase::goHomingPosition() {
  cout << "Go Homing Position" << endl;
  return true;
}

bool ITaskBase::goWorkingPosition() {
  cout << "Go Working Position" << endl;
  return true;
}
