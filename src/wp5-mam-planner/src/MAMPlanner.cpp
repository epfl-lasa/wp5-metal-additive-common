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

#include "RoboticArmFactory.h"

using namespace std;

MAMPlanner::MAMPlanner(ROSVersion rosVersion) : spinner_(1), nh_("ur5"), tfListener_(tfBuffer_) {
  RoboticArmFactory armFactory = RoboticArmFactory();
  robot_ = armFactory.createRoboticArm("ur5_robot", rosVersion);
  robot_->printInfo();

  initMoveit_();

  pubWeldingState_ = nh_.advertise<std_msgs::Bool>("welding_state", 1);
  pubDisplayTrajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 20);
  waypointPub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

  // Add obstacles
  addStaticObstacles_();
  getWaypoints_();
}

MAMPlanner::~MAMPlanner() { /*delete jointModelGroup_; */
}

bool MAMPlanner::computePath_(const vector<double>& startConfig, const geometry_msgs::Pose& targetPose) {
  // Set the starting configuration
  const string robotGroup = "manipulator";
  const moveit::core::JointModelGroup* jointModelGroup = moveGroup_->getCurrentState()->getJointModelGroup(robotGroup);

  moveGroup_->clearPoseTargets();
  // moveGroup_->getCurrentState()->setJointGroupPositions(jointModelGroup, startConfig);

  // Set the joint positions to the provided start configuration
  moveGroup_->setJointValueTarget(startConfig);

  // Plan and execute the motion to the starting configuration
  moveit::planning_interface::MoveGroupInterface::Plan startPlan;
  bool startSuccess = (moveGroup_->plan(startPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (startSuccess) {
    moveGroup_->move();
  } else {
    ROS_WARN("Failed to plan to the starting configuration");
    return false;
  }

  // moveGroup_->setStartState(*moveGroup_->getCurrentState());

  // Set the target pose
  moveGroup_->setPoseTarget(targetPose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (moveGroup_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    if (currentWpointID_ > bestPlan_.size()) {
      bestPlan_.push_back(plan);
    } else {
      bool isBetterPlan = plan.trajectory_.joint_trajectory.points.size()
                          < bestPlan_[currentWpointID_].trajectory_.joint_trajectory.points.size();

      if (isBetterPlan) {
        bestPlan_[currentWpointID_] = plan;
      }
    }
  }

  return success;
}

bool MAMPlanner::planTrajectory() {
  cout << "Planning trajectory" << endl;

  bool isPathFound = false;
  vector<vector<double>> ikSolutions{};

  // TODO(lmunier): Avoid using dynamic_cast
  RoboticArmUr5* robotUr5 = dynamic_cast<RoboticArmUr5*>(robot_.get());

  currentWpointID_ = 0;
  bestPlan_.clear();
  for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
    currentWpointID_ = i;
    const Waypoint currentWPoint = waypoints_[i];
    const Waypoint nextWPoint = waypoints_[i + 1];
    geometry_msgs::Pose nextTarget = generatePose_(nextWPoint.getPoseVector<double>());

    publishWaypoint_(nextTarget, "base_link_inertia", currentWpointID_);

    robotUr5->getIKGeo(currentWPoint.quat, currentWPoint.pos, ikSolutions);

    for (const auto& sol : ikSolutions) {
      cout << "Computing path for waypoint " << i << endl;
      isPathFound += computePath_(sol, nextTarget);
    }

    if (!isPathFound) {
      ROS_ERROR("No path found for waypoint %ld", i);
      return false;
    }
  }

  return true;
}

void MAMPlanner::executeTrajectory() {
  cout << "Executing trajectory" << endl;

  for (auto& plan : bestPlan_) {
    moveGroup_->execute(plan);
    moveGroup_->stop();
    moveGroup_->clearPoseTargets();
  }
}

void MAMPlanner::initMoveit_() {
  const string robotGroup = "manipulator";
  ros::Duration timeout(2.0);

  try {
    moveGroup_ = make_unique<moveit::planning_interface::MoveGroupInterface>(
        robotGroup, make_shared<tf2_ros::Buffer>(), timeout);
  } catch (const runtime_error& e) {
    ROS_ERROR("Failed to initialize MoveGroupInterface: %s", e.what());
    return;
  }

  planningScene_ = make_unique<moveit::planning_interface::PlanningSceneInterface>();
  setupMovegroup_();

  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

void MAMPlanner::setupMovegroup_() {
  moveGroup_->setPoseReferenceFrame(robot_->getReferenceFrame());
  moveGroup_->setPlannerId("RRTConnect");
  moveGroup_->setPlanningTime(5.0);
  moveGroup_->setNumPlanningAttempts(10);
  moveGroup_->setGoalPositionTolerance(0.005);
  moveGroup_->setGoalOrientationTolerance(0.01);
}

geometry_msgs::Pose MAMPlanner::generatePose_(const vector<double>& pose) {
  if (pose.size() != 6 && pose.size() != 7) {
    ROS_ERROR("Invalid pose size it should be 6 for Euler use or 7 for Quaternions.");
    return geometry_msgs::Pose();
  }

  geometry_msgs::Pose newPose;
  newPose.position.x = pose[0];
  newPose.position.y = pose[1];
  newPose.position.z = pose[2];

  if (pose.size() == 6) {
    Eigen::Quaterniond q = eulerToQuaternion_<double>({pose[3], pose[4], pose[5]});

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

geometry_msgs::Pose MAMPlanner::projectPose_(const geometry_msgs::Pose& pose,
                                             const string& fromFrame,
                                             const string& toFrame) {
  geometry_msgs::PoseStamped inputPose;
  inputPose.pose = pose;
  inputPose.header.frame_id = fromFrame;
  inputPose.header.stamp = ros::Time::now();

  try {
    geometry_msgs::TransformStamped transformStamped =
        tfBuffer_.lookupTransform(toFrame, fromFrame, ros::Time(0), ros::Duration(1.0));
    geometry_msgs::PoseStamped outputPose;

    tf2::doTransform(inputPose, outputPose, transformStamped);
    return outputPose.pose;
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Could NOT transform: %s", ex.what());
    return pose;
  }
}

void MAMPlanner::createNewFrame_(const string& parentFrame,
                                 const string& newFrame,
                                 const geometry_msgs::Transform& transform) {
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parentFrame;
  transformStamped.child_frame_id = newFrame;
  transformStamped.transform = transform;

  br_.sendTransform(transformStamped);
}

void MAMPlanner::getWaypoints_() {
  string yamlPath = string(WP5_MAM_PLANNER_DIR) + "/config/waypoints.yaml";
  YAML::Node config = YAML::LoadFile(yamlPath);

  Waypoint newWaypoint{};
  Eigen::Quaterniond tmpQuat{};
  vector<double> pose{};
  array<double, 3> tmpEuler{};
  geometry_msgs::Pose newPose{};

  for (const auto& waypoint : config) {
    newWaypoint.frame = waypoint["frame"].as<string>();

    // Get the pose and orientation, convert it and project it to the robot frame
    pose = waypoint["pos"].as<vector<double>>();
    tmpEuler = waypoint["angle"].as<array<double, 3>>();
    pose.insert(pose.end(), tmpEuler.begin(), tmpEuler.end());

    newPose = generatePose_(pose);
    newPose = projectPose_(newPose, newWaypoint.frame, robot_->getReferenceFrame());

    newWaypoint.pos = Eigen::Vector3d{newPose.position.x, newPose.position.y, newPose.position.z};
    newWaypoint.quat =
        Eigen::Quaterniond{newPose.orientation.w, newPose.orientation.x, newPose.orientation.y, newPose.orientation.z};
    newWaypoint.speed = waypoint["speed"].as<double>();
    newWaypoint.welding = waypoint["welding"].as<bool>();

    waypoints_.push_back(newWaypoint);
  }
}

void MAMPlanner::publishWaypoint_(const geometry_msgs::Pose& pose, const string& frameId, const int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.ns = "waypoints";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.lifetime = ros::Duration(30);

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  waypointPub_.publish(marker);
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
    newObstacle.pose = generatePose_(obstacle["pose"].as<vector<double>>());

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

  planningScene_->applyCollisionObjects(collisionObjects);
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