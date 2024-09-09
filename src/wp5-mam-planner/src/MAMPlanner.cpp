/**
 * @file MAMPlanner.cpp
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-03
 */
#include "MAMPlanner.h"

#include <moveit_msgs/DisplayTrajectory.h>
#include <std_msgs/Bool.h>
#include <yaml-cpp/yaml.h>

using namespace std;

//TODO(lmunier): Implement the MAMPlanner class

MAMPlanner::MAMPlanner() : spinner_(1) {
  try {
    robot_ = make_unique<RoboticArmUr5>();
    robot_->printInfo();
    initMoveit_();

    pub_welding_state_ = nh_.advertise<std_msgs::Bool>("welding_state", 1);
    pub_display_trajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 20);

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
  move_group_ = make_unique<moveit::planning_interface::MoveGroupInterface>(robot_group);
  move_group_->setPoseReferenceFrame(robot_base);
  move_group_->setPlannerId("RRTConnectkConfigDefault");
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(10);
  move_group_->setGoalPositionTolerance(0.005);
  move_group_->setGoalOrientationTolerance(0.01);

  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

void MAMPlanner::addStaticObstacles_() {
  vector<moveit_msgs::CollisionObject> collision_objects;
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_.getPlanningFrame();

  // Get obstacles from the config file
  YAML::Node config = YAML::LoadFile("config/obstacles.yaml");
  vector<YAML::Node> obstacles = config["obstacles"].as<vector<YAML::Node>>();

  for (const auto& obstacle : obstacles) {
    string name = obstacle["name"].as<string>();
    string type = obstacle["type"].as<string>();
    geometry_msgs::PoseStamped new_obstacle;

    new_obstacle.header.frame_id = collision_object.header.frame_id;
    new_obstacle.pose = generatePose_(obstacle["pose"].as<vector<double>>());
    collision_object.id = name;

    if (type == "box") {
      ROS_INFO("Adding box %s...", name);
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;

      primitive.dimensions.resize(3);
      primitive.dimensions = {obstacle["dimensions"][0].as<double>(),
                              obstacle["dimensions"][1].as<double>(),
                              obstacle["dimensions"][2].as<double>()};

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(new_obstacle.pose);
    } else if (type == "cylinder") {
      ROS_INFO("Adding cylinder %s...", name);
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.CYLINDER;

      primitive.dimensions.resize(2);
      primitive.dimensions = {obstacle["height"].as<double>(), obstacle["radius"].as<double>()};

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(new_obstacle.pose);
    } else if (type == "sphere") {
      ROS_INFO("Adding sphere %s...", name);
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.SPHERE;

      primitive.dimensions.resize(1);
      primitive.dimensions = {obstacle["radius"].as<double>()};

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(new_obstacle.pose);
    } else if (type == "mesh") {
      ROS_INFO("Adding mesh %s...", name);
      shapes::Mesh* m = shapes::createMeshFromResource(obstacle["path"].as<string>());

      shape_msgs::Mesh mesh;
      shapes::ShapeMsg mesh_msg;
      shapes::constructMsgFromShape(m, mesh_msg);
      mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

      collision_object.meshes.push_back(mesh);
      collision_object.mesh_poses.push_back(new_obstacle.pose);
    } else {
      ROS_ERROR("No such obstacle type.");
    }

    collision_objects.push_back(collision_object);
  }

  planning_scene_interface_.applyCollisionObjects(collision_objects);
}