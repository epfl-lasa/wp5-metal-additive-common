/**
 * @file ObstaclesManagement.h
 * @brief Declaration of the ObstaclesManagement class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
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

  /**
   * @brief Add static obstacles to the planning scene
   */
  void addStaticObstacles();

  /**
   * @brief Remove obstacles from the planning scene
   *
   * @param obstacleIds The IDs of the obstacles to remove
   */
  void removeObstacles(const std::vector<std::string>& obstacleIds);

  /**
   * @brief Update the planning scene with the current state of the environment
   */
  void updatePlanningScene();

private:
  ros::NodeHandle nh_;       ///< ROS node handle
  std::string frameID_ = ""; ///< Frame ID

  // Planning scene interface
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planningSceneInterface_ = nullptr;

  // Create new obstacles shape, each of these function create a new obstacle, based on their needed parameters
  const shape_msgs::SolidPrimitive createBox_(const std::string& name, const std::vector<double>& size) const;
  const shape_msgs::SolidPrimitive createCylinder_(const std::string& name, double height, double radius) const;
  const shape_msgs::SolidPrimitive createSphere_(const std::string& name, double radius) const;
  const shape_msgs::Mesh createMesh_(const std::string& name, const std::string& meshPath) const;
};