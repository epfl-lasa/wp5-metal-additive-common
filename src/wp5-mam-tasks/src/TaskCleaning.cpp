#include "TaskCleaning.h"

#include "yaml_tools.h"

using namespace std;

TaskCleaning::TaskCleaning(ros::NodeHandle& nh, string configFilename) :
    ITaskBase(nh, YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_TASKS_DIR)))["cleaning"]) {}

bool TaskCleaning::computeTrajectory(std::vector<geometry_msgs::Pose> waypoints) { return planner_->planTrajectory(); }

bool TaskCleaning::execute() { return planner_->executeTrajectory(); }
