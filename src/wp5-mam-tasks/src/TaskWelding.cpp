#include "TaskWelding.h"

#include "conversion_tools.h"
#include "yaml_tools.h"

using namespace std;

TaskWelding::TaskWelding(ros::NodeHandle& nh, string configFilename) :
    ITaskBase(nh, YAML::LoadFile(YamlTools::getYamlPath(configFilename, string(WP5_TASKS_DIR)))["welding"]) {}

bool TaskWelding::computeTrajectory(std::vector<geometry_msgs::Pose> waypoints) {
  std::vector<geometry_msgs::Pose> waypointsToPlan{};
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> poseOffset{};

  bool firstWaypoint = true;
  for (const auto& pose : waypoints) {
    if (firstWaypoint) {
      poseOffset = ConversionTools::vectorToEigenQuatPose(eePoseWorkOffset_);
      waypointsToPlan.push_back(MathTools::addOffset(pose, poseOffset));

      firstWaypoint = false;
    }

    poseOffset = ConversionTools::vectorToEigenQuatPose(eePoseOffset_);
    waypointsToPlan.push_back(MathTools::addOffset(pose, poseOffset));
  }

  return planner_->planTrajectory(waypointsToPlan);
}

bool TaskWelding::execute() { return planner_->executeTrajectory(); }
