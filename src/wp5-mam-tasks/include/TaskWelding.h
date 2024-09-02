#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

#include "IRoboticArmBase.h"
#include "ITaskBase.h"
#include "RosInterfaceNoetic.h"

class TaskWelding : public ITaskBase {
public:
  TaskWelding(ros::NodeHandle& n, double freq, std::string robotName);

  bool computePath();
  bool execute();

private:
};
