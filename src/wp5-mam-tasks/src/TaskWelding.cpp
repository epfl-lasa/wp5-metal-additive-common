#include "TaskWelding.h"

#include "yaml_tools.h"

using namespace std;

TaskWelding::TaskWelding(ros::NodeHandle& nh, string configFilename) :
    ITaskBase(nh, YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_TASKS_DIR)))["welding"]) {}

bool TaskWelding::computePath() {
  planner_->planTrajectory();
  return true;
}

bool TaskWelding::execute() {
  planner_->executeTrajectory();
  return true;
}
