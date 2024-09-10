/**
 * @file MAMPlanner.cpp
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-03
 */
#include "MAMPlanner.h"

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/Bool.h>

using namespace std;

//TODO(lmunier): Implement the MAMPlanner class

MAMPlanner::MAMPlanner() : spinner_(1) {
  try {
    robot_ = make_unique<RoboticArmUr5>();
    robot_->printInfo();
    initMoveit_();

    pubWeldingState_ = nh_.advertise<std_msgs::Bool>("welding_state", 1);
    pubDisplayTrajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 20);

    // Add obstacles
    addStaticObstacles_();

    // ikSolver_ = make_unique<TRAC_IK::TRAC_IK>(robot_base, virtual_target, "Distance", ros::Duration(0.01));

    // TODO: Implement tracIK solutions
    // for each solution, start a thread and compute path planning
    // if path planning not successful, reconfigure platform to a new position
    // if path planning successful, execute the trajectory

  } catch (const ros::Exception& e) {
    ROS_ERROR("ROS Exception: %s", e.what());
  } catch (const exception& e) {
    ROS_ERROR("Exception: %s", e.what());
  } catch (...) {
    ROS_ERROR("Unknown exception");
  }
}

void MAMPlanner::planTrajectory() {
  //TODO(lmunier): Implement the planTrajectory method
  cout << "Planning trajectory" << endl;
}

void MAMPlanner::executeTrajectory() {
  //TODO(lmunier): Implement the executeTrajectory method
  cout << "Executing trajectory" << endl;
}

void MAMPlanner::initMoveit_() {
  string robot_group = "manipulator";
  string robot_base = "base_link_inertia";

  scene_ = make_unique<moveit::planning_interface::PlanningSceneInterface>();
  moveGroup_ = make_unique<moveit::planning_interface::MoveGroupInterface>(robot_group);
  moveGroup_->setPoseReferenceFrame(robot_base);
  moveGroup_->setPlannerId("RRTConnectkConfigDefault");
  moveGroup_->setPlanningTime(5.0);
  moveGroup_->setNumPlanningAttempts(10);
  moveGroup_->setGoalPositionTolerance(0.005);
  moveGroup_->setGoalOrientationTolerance(0.01);

  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

geometry_msgs::Pose MAMPlanner::generatePose_(const vector<double>& pose) {
  if (pose.size() != 6 && pose.size() != 7) {
    ROS_ERROR("Invalid pose size it should be 6 or 7.");
    return geometry_msgs::Pose();
  }

  geometry_msgs::Pose newPose;
  newPose.position.x = pose[0];
  newPose.position.y = pose[1];
  newPose.position.z = pose[2];

  if (pose.size() == 6) {
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(pose[3], Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pose[4], Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(pose[5], Eigen::Vector3f::UnitZ());

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