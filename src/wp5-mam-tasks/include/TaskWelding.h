#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

#include "IRoboticArmBase.h"
#include "IRosInterfaceBase.h"
#include "ITaskBase.h"

class TaskWelding : public ITaskBase {
public:
  TaskWelding(ros::NodeHandle& nh, std::string configFilename);

  bool computePath();
  bool execute();

private:
  const TaskType taskType_ = WELDING; ///< Task type.
};
