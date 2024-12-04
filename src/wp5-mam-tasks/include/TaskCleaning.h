#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

#include "IRoboticArmBase.h"
#include "IRosInterfaceBase.h"
#include "ITaskBase.h"

class TaskCleaning : public ITaskBase {
public:
  TaskCleaning(ros::NodeHandle& nh, std::string configFilename);

  bool computeTrajectory(std::vector<geometry_msgs::Pose> waypoints);
  bool execute();
};
