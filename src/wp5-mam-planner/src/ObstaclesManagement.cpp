/**
 * @file ObstaclesManagement.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-09-09-
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "ObstaclesManagement.h"

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "convertion_tools.h"
#include "yaml_tools.h"

using namespace std;

ObstaclesManagement::ObstaclesManagement(string frameID) : frameID_(frameID) {
  planningScene_ = make_unique<moveit::planning_interface::PlanningSceneInterface>();
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
    newObstacle.pose = ConvertionTools::vectorToPose(obstacle["pose"].as<vector<double>>());

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

  planningScene_->applyCollisionObjects(collisionObjects);
}

const shape_msgs::SolidPrimitive ObstaclesManagement::createBox_(const string& name, const vector<double>& size) const {
  ROS_INFO("[ObstacleManagement] - Adding box %s...", name.c_str());
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;

  primitive.dimensions.resize(3);
  primitive.dimensions = size;

  return move(primitive);
}

const shape_msgs::SolidPrimitive ObstaclesManagement::createCylinder_(const string& name,
                                                                      const double height,
                                                                      const double radius) const {
  ROS_INFO("[ObstacleManagement] - Adding cylinder %s...", name.c_str());
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;

  primitive.dimensions.resize(2);
  primitive.dimensions = {height, radius};

  return move(primitive);
}

const shape_msgs::SolidPrimitive ObstaclesManagement::createSphere_(const string& name, const double radius) const {
  ROS_INFO("[ObstacleManagement] - Adding sphere %s...", name.c_str());
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;

  primitive.dimensions.resize(1);
  primitive.dimensions = {radius};

  return move(primitive);
}

const shape_msgs::Mesh ObstaclesManagement::createMesh_(const string& name, const string& meshPath) const {
  ROS_INFO("[ObstacleManagement] - Adding mesh %s...", name.c_str());
  shapes::Mesh* m = shapes::createMeshFromResource(meshPath);

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg meshMsg;
  shapes::constructMsgFromShape(m, meshMsg);
  mesh = boost::get<shape_msgs::Mesh>(meshMsg);

  return move(mesh);
}