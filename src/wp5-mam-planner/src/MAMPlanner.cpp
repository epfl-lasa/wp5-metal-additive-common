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

  // Add obstacles
  addStaticObstacles_();
  getWaypoints_();

  // ikSolver_ = make_unique<TRAC_IK::TRAC_IK>(robotBase, virtualTarget, "Distance", ros::Duration(0.01));

  // TODO: Implement tracIK solutions
  // for each solution, start a thread and compute path planning
  // if path planning not successful, reconfigure platform to a new position
  // if path planning successful, execute the trajectory
}

bool MAMPlanner::computePath_(const vector<double>& startConfig, const geometry_msgs::Pose& targetPose) {
  bool isPathFound = false;

  // Set the starting configuration
  robotState_->setJointGroupPositions(jointModelGroup_, startConfig);
  moveGroup_->setStartState(*robotState_);

  // Set the target pose
  moveGroup_->setPoseTarget(targetPose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (moveGroup_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    bool isBetterPlan =
        plan.trajectory_.joint_trajectory.points.size() < bestPlan_.trajectory_.joint_trajectory.points.size();

    if (!isPathFound || isBetterPlan) {
      bestPlan_ = plan;
      isPathFound = true;
    }
  }

  return isPathFound;
}

void MAMPlanner::planTrajectory() {
  bool isPathFound = false;
  cout << "Planning trajectory" << endl;
  vector<vector<double>> ikSolutions{};
  RoboticArmUr5* robotUr5 = dynamic_cast<RoboticArmUr5*>(robot_.get());

  for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
    const Waypoint currentWPoint = waypoints_[i];
    const Waypoint nextWPoint = waypoints_[i + 1];
    geometry_msgs::Pose nextTarget = generatePose_(nextWPoint.getPoseVector<double>());

    robotUr5->getIKGeo(currentWPoint.quat, currentWPoint.pos, ikSolutions);

    for (const auto& sol : ikSolutions) {
      isPathFound = computePath_(sol, nextTarget);
    }

    if (isPathFound) {
      moveGroup_->execute(bestPlan_);
      moveGroup_->stop();
      moveGroup_->clearPoseTargets();
    } else {
      ROS_ERROR("No valid path found");
    }
  }
}

void MAMPlanner::executeTrajectory() {
  //TODO(lmunier): Implement the executeTrajectory method
  cout << "Executing trajectory" << endl;
}

void MAMPlanner::initMoveit_() {
  string robotGroup = "manipulator";
  moveGroup_ = make_unique<moveit::planning_interface::MoveGroupInterface>(robotGroup);
  planningScene_ = make_unique<moveit::planning_interface::PlanningSceneInterface>();
  setupMovegroup_();

  robotState_ = moveGroup_->getCurrentState();
  jointModelGroup_ = moveGroup_->getCurrentState()->getJointModelGroup(robotGroup);

  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

void MAMPlanner::setupMovegroup_() {
  moveGroup_->setPoseReferenceFrame(robot_->getReferenceFrame());
  moveGroup_->setPlannerId("RRTConnectkConfigDefault");
  moveGroup_->setPlanningTime(5.0);
  moveGroup_->setNumPlanningAttempts(10);
  moveGroup_->setGoalPositionTolerance(0.005);
  moveGroup_->setGoalOrientationTolerance(0.01);
}

geometry_msgs::Pose MAMPlanner::generatePose_(const vector<double>& pose) {
  if (pose.size() != 6 && pose.size() != 7) {
    ROS_ERROR("Invalid pose size it should be 6 for Euler use or 7 for quaternions.");
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
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.pose = pose;
  poseStamped.header.frame_id = fromFrame;
  poseStamped.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped transformedPoseStamped;
  try {
    tfBuffer_.transform(poseStamped, transformedPoseStamped, toFrame, ros::Duration(1.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Transform failed: %s", ex.what());
    return pose; // Return the original pose if the transform fails
  }

  return transformedPoseStamped.pose;
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
    tmpQuat = eulerToQuaternion_<double>(tmpEuler);
    pose.insert(pose.end(), {tmpQuat.x(), tmpQuat.y(), tmpQuat.z(), tmpQuat.w()});

    newPose = generatePose_(pose);
    newPose = projectPose_(newPose, newWaypoint.frame, robot_->getReferenceFrame());

    newWaypoint.pos = Eigen::Vector3d{newPose.position.x, newPose.position.y, newPose.position.z};
    newWaypoint.quat =
        Eigen::Quaterniond{newPose.orientation.x, newPose.orientation.y, newPose.orientation.z, newPose.orientation.w};
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