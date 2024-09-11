#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

#include "IRoboticArmBase.h"
#include "IRosInterfaceBase.h"
#include "ITaskBase.h"

class TaskWelding : public ITaskBase {
public:
  TaskWelding(ros::NodeHandle& n, ROSVersion rosVersion, double freq, std::string robotName);

  bool computePath();
  bool execute();

private:
};
