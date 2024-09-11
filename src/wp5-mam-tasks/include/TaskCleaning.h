#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

#include "IRoboticArmBase.h"
#include "IRosInterfaceBase.h"
#include "ITaskBase.h"

class TaskCleaning : public ITaskBase {
public:
  TaskCleaning(ros::NodeHandle& n, ROSVersion rosVersion, double freq, std::string robotName);

  bool computePath();
  bool execute();

  std::vector<double> homeJoint_;

private:
};
