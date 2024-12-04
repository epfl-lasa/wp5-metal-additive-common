#include "TaskWelding.h"

#include "yaml_tools.h"

using namespace std;

TaskWelding::TaskWelding(ros::NodeHandle& nh, string configFilename) :
    ITaskBase(nh, YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_TASKS_DIR)))["welding"]) {}

bool TaskWelding::computeTrajectory(std::vector<geometry_msgs::Pose> waypoints) { return planner_->planTrajectory(); }

bool TaskWelding::execute() { return planner_->executeTrajectory(); }
