#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

#include "IRoboticArmBase.h"
#include "ITaskBase.h"
#include "RosInterfaceNoetic.h"

class TaskCleaning : public ITaskBase {
public:
  TaskCleaning(ros::NodeHandle& n, double freq, std::string robotName);

  bool computePath();
  bool execute();

  std::vector<double> homeJoint_;

private:
};
