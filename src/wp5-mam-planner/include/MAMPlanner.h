/**
 * @file MAMPlanner.h
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-03
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
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
  static const double TOLERANCE;

  struct Waypoint {
    std::string frame = "";
    Eigen::Vector3d pos{};
    Eigen::Quaterniond quat{};
    double speed = 0.0;
    bool welding = false;

    void clear() {
      frame.clear();
      pos.setZero();
      quat.setIdentity();
      speed = 0.0;
      welding = false;
    }

    void print() const {
      ROS_INFO("Frame: %s", frame.c_str());
      ROS_INFO("Position: %f %f %f", pos.x(), pos.y(), pos.z());
      ROS_INFO("Quaternion: %f %f %f %f", quat.x(), quat.y(), quat.z(), quat.w());
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

  std::vector<std::thread> threads;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool pathFound_ = false;
  moveit::planning_interface::MoveGroupInterface::Plan bestPlan_;

  planning_pipeline::PlanningPipelinePtr planningPipeline_ = nullptr;
  planning_scene::PlanningScenePtr planningScene_ = nullptr;
  robot_model_loader::RobotModelLoaderPtr robotModelLoader_ = nullptr;

  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_ = nullptr; ///< Planning scene
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup_ = nullptr; ///< MoveGroup interface

  void initMoveit_();
  void setupMovegroup_(moveit::planning_interface::MoveGroupInterface* mGroup);
  geometry_msgs::Pose generatePose_(const std::vector<float>& pose);

  template <typename T>
  Eigen::Quaternion<T> eulerToQuaternion_(const std::array<T, 3>& euler) {
    Eigen::Quaternion<T> q;
    q = Eigen::AngleAxis<T>(euler[0], Eigen::Matrix<T, 3, 1>::UnitX())
        * Eigen::AngleAxis<T>(euler[1], Eigen::Matrix<T, 3, 1>::UnitY())
        * Eigen::AngleAxis<T>(euler[2], Eigen::Matrix<T, 3, 1>::UnitZ());

    return q;
  }

  bool areQuaternionsEquivalent_(const Eigen::Quaterniond& q1,
                                 const Eigen::Quaterniond& q2,
                                 double tolerance = TOLERANCE);

  bool arePositionsEquivalent_(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double tolerance = TOLERANCE);

  void getWaypoints_();
  void computePath_(const std::string& group, const std::string& base_link, const geometry_msgs::Pose& target_pose);
  void addStaticObstacles_();

  shape_msgs::SolidPrimitive createBox_(const std::string name, const std::vector<double>& size) const;
  shape_msgs::SolidPrimitive createCylinder_(const std::string name, const double height, const double radius) const;
  shape_msgs::SolidPrimitive createSphere_(const std::string name, const double radius) const;
  shape_msgs::Mesh createMesh_(const std::string name, const std::string meshPath) const;
};