/**
 * @file ObstaclesManagement.h
 * @brief
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */
#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/SolidPrimitive.h>

#include <string>
#include <vector>

class ObstaclesManagement {
public:
  /**
   * @brief Constructor.
   */
  ObstaclesManagement(ros::NodeHandle nh, std::string frameID);

  void addStaticObstacles();
  void removeObstacles(const std::vector<std::string>& obstacleIds);
  void updatePlanningScene();

private:
  ros::NodeHandle nh_;       ///< ROS node handle
  std::string frameID_ = ""; ///< Frame ID

  // Planning scene interface
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planningSceneInterface_ = nullptr;

  // Create new obstacles shape
  const shape_msgs::SolidPrimitive createBox_(const std::string& name, const std::vector<double>& size) const;
  const shape_msgs::SolidPrimitive createCylinder_(const std::string& name, double height, double radius) const;
  const shape_msgs::SolidPrimitive createSphere_(const std::string& name, double radius) const;
  const shape_msgs::Mesh createMesh_(const std::string& name, const std::string& meshPath) const;
};