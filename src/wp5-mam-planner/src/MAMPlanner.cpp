/**
 * @file MAMPlanner.cpp
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-12
 */
#include "MAMPlanner.h"

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/Bool.h>

#include <thread>

#include "RoboticArmFactory.h"

using namespace std;

const double MAMPlanner::TOLERANCE = 1e-5;

MAMPlanner::MAMPlanner(ROSVersion rosVersion) : spinner_(1), nh_("ur5") {
  RoboticArmFactory armFactory = RoboticArmFactory();
  robot_ = armFactory.createRoboticArm("ur5_robot", rosVersion);
  robot_->printInfo();

  initMoveit_();

  pubWeldingState_ = nh_.advertise<std_msgs::Bool>("welding_state", 1);
  pubDisplayTrajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 20);

  // Add obstacles
  addStaticObstacles_();
  getWaypoints_();

  // ikSolver_ = make_unique<TRAC_IK::TRAC_IK>(robotBase, virtualTarget, "Distance", ros::Duration(0.01));

  // TODO: Implement tracIK solutions
  // for each solution, start a thread and compute path planning
  // if path planning not successful, reconfigure platform to a new position
  // if path planning successful, execute the trajectory
}

void MAMPlanner::computePath_(const std::string& group,
                              const std::string& base_link,
                              const geometry_msgs::Pose& target_pose) {
  moveit::planning_interface::MoveGroupInterface moveGroup(group);
  moveGroup.setPoseReferenceFrame(base_link);
  moveGroup.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (moveGroup.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    std::unique_lock<std::mutex> lock(mtx_);
    bool isBetterPlan =
        plan.trajectory_.joint_trajectory.points.size() < bestPlan_.trajectory_.joint_trajectory.points.size();

    if (!pathFound_ || isBetterPlan) {
      bestPlan_ = plan;
      pathFound_ = true;
      cv_.notify_all();
    }
  }
}

void MAMPlanner::planTrajectory() {
  std::vector<geometry_msgs::Pose> ikSolutions; // Assume this is populated with IK solutions

  for (const auto& pose : ikSolutions) {
    threads.emplace_back(&MAMPlanner::computePath_, this, "manipulator", "base_link_inertia", pose);
  }

  {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] { return pathFound_; }); // Wait until a path is found
  }

  for (auto& thread : threads) {
    if (thread.joinable()) {
      thread.join(); // Join all threads
    }
  }

  if (pathFound_) {
    moveGroup_->execute(bestPlan_);
    moveGroup_->stop();
    moveGroup_->clearPoseTargets();
  } else {
    ROS_ERROR("No valid path found");
  }
}

void MAMPlanner::executeTrajectory() {
  //TODO(lmunier): Implement the executeTrajectory method
  cout << "Executing trajectory" << endl;
}

void MAMPlanner::initMoveit_() {
  string robotGroup = "manipulator";
  moveGroup_ = make_unique<moveit::planning_interface::MoveGroupInterface>(robotGroup);
  scene_ = make_unique<moveit::planning_interface::PlanningSceneInterface>();
  setupMovegroup_(moveGroup_.get());

  robotModelLoader_ = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
  planningScene_ = std::make_shared<planning_scene::PlanningScene>(robotModelLoader_->getModel());

  // Initialize the planning pipeline
  planningPipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(
      robotModelLoader_->getModel(), nh_, "planning_plugin", "request_adapters");

  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

void MAMPlanner::setupMovegroup_(moveit::planning_interface::MoveGroupInterface* mGroup) {
  mGroup->setPoseReferenceFrame(robot_->getReferenceFrame());
  mGroup->setPlannerId("RRTConnectkConfigDefault");
  mGroup->setPlanningTime(5.0);
  mGroup->setNumPlanningAttempts(10);
  mGroup->setGoalPositionTolerance(0.005);
  mGroup->setGoalOrientationTolerance(0.01);
}

geometry_msgs::Pose MAMPlanner::generatePose_(const vector<float>& pose) {
  if (pose.size() != 6 && pose.size() != 7) {
    ROS_ERROR("Invalid pose size it should be 6 or 7.");
    return geometry_msgs::Pose();
  }

  geometry_msgs::Pose newPose;
  newPose.position.x = pose[0];
  newPose.position.y = pose[1];
  newPose.position.z = pose[2];

  if (pose.size() == 6) {
    Eigen::Quaternionf q = eulerToQuaternion_<float>({pose[3], pose[4], pose[5]});

    newPose.orientation.x = q.x();
    newPose.orientation.y = q.y();
    newPose.orientation.z = q.z();
    newPose.orientation.w = q.w();
  } else if (pose.size() == 7) {
    newPose.orientation.x = pose[3];
    newPose.orientation.y = pose[4];
    newPose.orientation.z = pose[5];
    newPose.orientation.w = pose[6];
  }

  return newPose;
}

bool MAMPlanner::areQuaternionsEquivalent_(const Eigen::Quaterniond& q1,
                                           const Eigen::Quaterniond& q2,
                                           double tolerance) {
  Eigen::Matrix3d rot1 = q1.toRotationMatrix();
  Eigen::Matrix3d rot2 = q2.toRotationMatrix();

  return (rot1 - rot2).norm() < tolerance;
}

// Function to check if two positions are equivalent
bool MAMPlanner::arePositionsEquivalent_(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double tolerance) {
  return (p1 - p2).norm() < tolerance;
}

void MAMPlanner::getWaypoints_() {
  string yamlPath = string(WP5_MAM_PLANNER_DIR) + "/config/waypoints.yaml";
  YAML::Node config = YAML::LoadFile(yamlPath);

  Waypoint newWaypoint;
  array<double, 3> euler{};
  array<double, 3> position{};

  for (const auto& waypoint : config) {
    newWaypoint.clear();
    newWaypoint.frame = waypoint["frame"].as<string>();

    position = waypoint["pos"].as<array<double, 3>>();
    newWaypoint.pos = Eigen::Vector3d(position[0], position[1], position[2]);

    euler = waypoint["angle"].as<array<double, 3>>();
    newWaypoint.quat = eulerToQuaternion_<double>(euler);

    newWaypoint.speed = waypoint["speed"].as<double>();
    newWaypoint.welding = waypoint["welding"].as<bool>();

    waypoints_.push_back(newWaypoint);
  }
}

void MAMPlanner::addStaticObstacles_() {
  string name{}, type{};

  moveit_msgs::CollisionObject collisionObject;
  vector<moveit_msgs::CollisionObject> collisionObjects;
  collisionObject.header.frame_id = moveGroup_->getPlanningFrame();

  // Get obstacles from the config file
  string yamlPath = string(WP5_MAM_PLANNER_DIR) + "/config/obstacles.yaml";
  YAML::Node config = YAML::LoadFile(yamlPath);
  vector<YAML::Node> obstacles = config["obstacles"].as<vector<YAML::Node>>();

  for (const auto& obstacle : obstacles) {
    name = obstacle["name"].as<string>();
    type = obstacle["type"].as<string>();
    geometry_msgs::PoseStamped newObstacle;

    newObstacle.header.frame_id = collisionObject.header.frame_id;
    newObstacle.pose = generatePose_(obstacle["pose"].as<vector<float>>());

    // Create the collision object
    if (type == "box") {
      vector<double> size = obstacle["size"].as<vector<double>>();

      collisionObject.primitives.push_back(createBox_(name, size));
    } else if (type == "cylinder") {
      double height = obstacle["height"].as<double>();
      double radius = obstacle["radius"].as<double>();

      collisionObject.primitives.push_back(createCylinder_(name, height, radius));
    } else if (type == "sphere") {
      double radius = obstacle["radius"].as<double>();

      collisionObject.primitives.push_back(createSphere_(name, radius));
    } else if (type == "mesh") {
      string meshPath = obstacle["mesh_path"].as<string>();

      collisionObject.meshes.push_back(createMesh_(name, meshPath));
    } else {
      ROS_ERROR("No such obstacle type.");
      continue;
    }

    // Add the obstacle to the scene
    if (type == "mesh") {
      collisionObject.mesh_poses.push_back(newObstacle.pose);
    } else {
      collisionObject.primitive_poses.push_back(newObstacle.pose);
    }

    collisionObject.id = name;
    collisionObjects.push_back(collisionObject);
  }

  scene_->applyCollisionObjects(collisionObjects);
}

shape_msgs::SolidPrimitive MAMPlanner::MAMPlanner::createBox_(const string name, const vector<double>& size) const {
  ROS_INFO("Adding box %s...", name.c_str());
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;

  primitive.dimensions.resize(3);
  primitive.dimensions = size;

  return move(primitive);
}

shape_msgs::SolidPrimitive MAMPlanner::createCylinder_(const string name,
                                                       const double height,
                                                       const double radius) const {
  ROS_INFO("Adding cylinder %s...", name.c_str());
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;

  primitive.dimensions.resize(2);
  primitive.dimensions = {height, radius};

  return move(primitive);
}

shape_msgs::SolidPrimitive MAMPlanner::createSphere_(const string name, const double radius) const {
  ROS_INFO("Adding sphere %s...", name.c_str());
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;

  primitive.dimensions.resize(1);
  primitive.dimensions = {radius};

  return move(primitive);
}

shape_msgs::Mesh MAMPlanner::createMesh_(const string name, const string meshPath) const {
  ROS_INFO("Adding mesh %s...", name.c_str());
  shapes::Mesh* m = shapes::createMeshFromResource(meshPath);

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg meshMsg;
  shapes::constructMsgFromShape(m, meshMsg);
  mesh = boost::get<shape_msgs::Mesh>(meshMsg);

  return move(mesh);
}