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
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
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
  bool planTrajectory();

  /**
   * @brief Executes the trajectory of the robot.
   */
  void executeTrajectory();

private:
  struct Waypoint {
    std::string frame = "";
    Eigen::Vector3d pos{};
    Eigen::Quaterniond quat{};
    double speed = 0.0;
    bool welding = false;

    void clear() {
      pos.setZero();
      quat.setIdentity();
      speed = 0.0;
      welding = false;
    }

    template <typename T>
    std::vector<T> getPoseVector() const {
      // Safety type check to allow only float, int or double
      static_assert(std::is_same<T, float>::value || std::is_same<T, int>::value || std::is_same<T, double>::value,
                    "Invalid type for getPoseVector, should be either int, float or double.");

      std::vector<double> pose{pos.x(), pos.y(), pos.z(), quat.x(), quat.y(), quat.z(), quat.w()};
      return std::vector<T>(pose.begin(), pose.end());
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
  ros::AsyncSpinner spinner_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster br_; ///< ROS spinner to handle callbacks asynchronously

  ros::Publisher pubWeldingState_;      ///< Publisher for the welding state
  ros::Publisher pubDisplayTrajectory_; ///< Publisher for the display trajectory

  bool pathFound_ = false;
  int currentWpointID_ = 0;
  std::vector<Waypoint> waypoints_; ///< Waypoints for the robot
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> bestPlan_;

  moveit::core::RobotStatePtr robotState_ = nullptr;
  const moveit::core::JointModelGroup* jointModelGroup_ = nullptr;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planningScene_ = nullptr; ///< Planning scene
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup_ = nullptr;         ///< MoveGroup interface

  void initMoveit_();
  void setupMovegroup_();
  geometry_msgs::Pose generatePose_(const std::vector<double>& pose);
  geometry_msgs::Pose projectPose_(const geometry_msgs::Pose& pose,
                                   const std::string& fromFrame,
                                   const std::string& toFrame);
  void createNewFrame_(const std::string& parentFrame,
                       const std::string& newFrame,
                       const geometry_msgs::Transform& transform);

  template <typename T>
  Eigen::Quaternion<T> eulerToQuaternion_(const std::array<T, 3>& euler) {
    Eigen::Quaternion<T> q;
    q = Eigen::AngleAxis<T>(euler[0], Eigen::Matrix<T, 3, 1>::UnitX())
        * Eigen::AngleAxis<T>(euler[1], Eigen::Matrix<T, 3, 1>::UnitY())
        * Eigen::AngleAxis<T>(euler[2], Eigen::Matrix<T, 3, 1>::UnitZ());

    return q;
  }

  void getWaypoints_();
  bool computePath_(const std::vector<double>& startConfig, const geometry_msgs::Pose& targetPose);
  void addStaticObstacles_();

  shape_msgs::SolidPrimitive createBox_(const std::string name, const std::vector<double>& size) const;
  shape_msgs::SolidPrimitive createCylinder_(const std::string name, const double height, const double radius) const;
  shape_msgs::SolidPrimitive createSphere_(const std::string name, const double radius) const;
  shape_msgs::Mesh createMesh_(const std::string name, const std::string meshPath) const;
};