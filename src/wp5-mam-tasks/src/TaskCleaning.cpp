#include "TaskCleaning.h"

#include "yaml_tools.h"

using namespace std;

TaskCleaning::TaskCleaning(ros::NodeHandle& nh, string configFilename) :
    ITaskBase(nh, YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_TASKS_DIR)))["cleaning"]) {}

bool TaskCleaning::computePath() {
  planner_->planTrajectory();
  return true;
}

bool TaskCleaning::execute() {
  planner_->executeTrajectory();
  return true;
}
