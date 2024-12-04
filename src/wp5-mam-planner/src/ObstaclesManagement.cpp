/**
 * @file ObstaclesManagement.cpp
 * @author [Louis Munier] - lmunier@protonmail.com
 * @brief
 * @version 0.1
 * @date 2024-09-09-
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#include "ObstaclesManagement.h"

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>

#include "conversion_tools.h"
#include "yaml_tools.h"

using namespace std;

ObstaclesManagement::ObstaclesManagement(ros::NodeHandle nh, string frameID) : nh_(nh), frameID_(frameID) {
  planningSceneInterface_ = make_unique<moveit::planning_interface::PlanningSceneInterface>();
}

void ObstaclesManagement::addStaticObstacles() {
  string name{}, type{};

  moveit_msgs::CollisionObject collisionObject;
  vector<moveit_msgs::CollisionObject> collisionObjects;
  collisionObject.header.frame_id = frameID_;

  // Get obstacles from the config file
  string yamlPath = YamlTools::getYamlPath("obstacles.yaml", string(WP5_MAM_PLANNER_DIR));
  YAML::Node config = YAML::LoadFile(yamlPath);
  vector<YAML::Node> obstacles = config["obstacles"].as<vector<YAML::Node>>();

  for (const auto& obstacle : obstacles) {
    name = obstacle["name"].as<string>();
    type = obstacle["type"].as<string>();
    geometry_msgs::PoseStamped newObstacle;

    newObstacle.header.frame_id = collisionObject.header.frame_id;
    newObstacle.pose = ConversionTools::vectorToGeometryPose(obstacle["pose"].as<vector<double>>());

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
      ROS_ERROR("[ObstacleManagement] - No such obstacle type.");
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

  planningSceneInterface_->applyCollisionObjects(collisionObjects);
}

void ObstaclesManagement::removeObstacles(const vector<string>& obstacleIds) {
  planningSceneInterface_->removeCollisionObjects(obstacleIds);
}

void ObstaclesManagement::updatePlanningScene() {
  ros::ServiceClient planningSceneDiffClient = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES
                                      | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY
                                      | moveit_msgs::PlanningSceneComponents::ROBOT_STATE
                                      | moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
                                      | moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS
                                      | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

  if (planningSceneDiffClient.call(srv)) {
    planningSceneInterface_->applyPlanningScene(srv.response.scene);
  } else {
    ROS_ERROR("[ObstacleManagement] - Failed to call service get_planning_scene");
  }
}

const shape_msgs::SolidPrimitive ObstaclesManagement::createBox_(const string& name, const vector<double>& size) const {
  ROS_INFO_STREAM("[ObstacleManagement] - Adding box " << name << "...");
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;

  primitive.dimensions.resize(3);
  primitive.dimensions = size;

  return primitive;
}

const shape_msgs::SolidPrimitive ObstaclesManagement::createCylinder_(const string& name,
                                                                      const double height,
                                                                      const double radius) const {
  ROS_INFO_STREAM("[ObstacleManagement] - Adding cylinder " << name << "...");
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;

  primitive.dimensions.resize(2);
  primitive.dimensions = {height, radius};

  return primitive;
}

const shape_msgs::SolidPrimitive ObstaclesManagement::createSphere_(const string& name, const double radius) const {
  ROS_INFO_STREAM("[ObstacleManagement] - Adding sphere " << name << "...");
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;

  primitive.dimensions.resize(1);
  primitive.dimensions = {radius};

  return primitive;
}

const shape_msgs::Mesh ObstaclesManagement::createMesh_(const string& name, const string& meshPath) const {
  ROS_INFO_STREAM("[ObstacleManagement] - Adding mesh " << name << "...");
  shapes::Mesh* m = shapes::createMeshFromResource(meshPath);

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg meshMsg;
  shapes::constructMsgFromShape(m, meshMsg);
  mesh = boost::get<shape_msgs::Mesh>(meshMsg);

  return mesh;
}
