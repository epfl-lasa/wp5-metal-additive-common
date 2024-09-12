/**
 * @file MAMPlanner.h
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-03
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <vector>

#include "RoboticArmCr7.h"
#include "RoboticArmUr5.h"

class MAMPlanner {
public:
  /**
   * @brief Constructor.
   */
  MAMPlanner(ROSVersion rosVersion);

  /**
   * @brief Plans the trajectory of the robot.
   */
  void planTrajectory();

  /**
   * @brief Executes the trajectory of the robot.
   */
  void executeTrajectory();

private:
  struct Waypoint {
    std::string frame = "";
    std::array<double, 3> pos = {};
    std::array<double, 4> quat = {};
    double speed = 0.0;
    bool welding = false;

    void clear() {
      frame.clear();
      pos.fill(0.0);
      quat.fill(0.0);
      speed = 0.0;
      welding = false;
    }

    void print() const {
      ROS_INFO("Frame: %s", frame.c_str());
      ROS_INFO("Position: %f %f %f", pos[0], pos[1], pos[2]);
      ROS_INFO("Quaternion: %f %f %f %f", quat[0], quat[1], quat[2], quat[3]);
      ROS_INFO("Speed: %f", speed);
      ROS_INFO("Welding: %s", welding ? "true" : "false");
    }
  };

  std::unique_ptr<IRoboticArmBase> robot_ = nullptr; ///< Robotic arm
  ros::NodeHandle nh_;                               ///< ROS node handle
  ros::AsyncSpinner spinner_;                        ///< ROS spinner to handle callbacks asynchronously
  std::vector<Waypoint> waypoints_;                  ///< Waypoints for the robot

  ros::Publisher pubWeldingState_;      ///< Publisher for the welding state
  ros::Publisher pubDisplayTrajectory_; ///< Publisher for the display trajectory

  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_ = nullptr; ///< Planning scene
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup_ = nullptr; ///< MoveGroup interface

  void initMoveit_();
  geometry_msgs::Pose generatePose_(const std::vector<double>& pose);
  Eigen::Quaternionf eulerToQuaternion_(const std::array<double, 3>& euler);

  void getWaypoints_();
  void addStaticObstacles_();

  shape_msgs::SolidPrimitive createBox_(const std::string name, const std::vector<double>& size) const;
  shape_msgs::SolidPrimitive createCylinder_(const std::string name, const double height, const double radius) const;
  shape_msgs::SolidPrimitive createSphere_(const std::string name, const double radius) const;
  shape_msgs::Mesh createMesh_(const std::string name, const std::string meshPath) const;
};