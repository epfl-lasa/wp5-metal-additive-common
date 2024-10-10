/**
 * @file ObstaclesManagement.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-09-09-
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/SolidPrimitive.h>

#include <string>
#include <vector>

class ObstaclesManagement {
public:
  /**
   * @brief Constructor.
   */
  ObstaclesManagement(std::string frameID);

  void addStaticObstacles();

private:
  std::string frameID_ = "";                                                                    ///< Frame ID
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planningScene_ = nullptr; ///< Planning scene

  const shape_msgs::SolidPrimitive createBox_(const std::string& name, const std::vector<double>& size) const;
  const shape_msgs::SolidPrimitive createCylinder_(const std::string& name, double height, double radius) const;
  const shape_msgs::SolidPrimitive createSphere_(const std::string& name, double radius) const;
  const shape_msgs::Mesh createMesh_(const std::string& name, const std::string& meshPath) const;
};