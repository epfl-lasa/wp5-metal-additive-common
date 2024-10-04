/**
 * @file Subtask.h
 * @brief Declaration of the Subtask class
 * @author [Elise Jeandupeux]
 * @date 2024-10-03
 */

#pragma once

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>

class Subtask {
public:
  /**
   * @brief Constructor.
   */
  Subtask(ros::NodeHandle& nh);

  /**
   * @brief Destructor.
   */
  ~Subtask() = default;

  /**
   * @brief Callback to get the region of interests.
   */
  void roiCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * @brief Delete all stored ROI
   */
  bool clearROI();

  /**
   * @brief True if no region of interest
   */
  bool isSubtaskEmpty();

  /**
   * @brief Get the first region of interest and deletes it
   */
  template <typename T>
  std::vector<std::vector<double>> getROI() {
    if (!roiMap_.empty()) {
      auto iterROI = roiMap_.begin();

      // Access the first element's value and remove it from the map
      ROI value = iterROI->second;
      roiMap_.erase(iterROI);

      // return value;
      std::vector<double> pose1{value.pos1.x(),
                                value.pos1.y(),
                                value.pos1.z(),
                                value.quat.x(),
                                value.quat.y(),
                                value.quat.z(),
                                value.quat.w()};
      std::vector<T> pose2{value.pos2.x(),
                           value.pos2.y(),
                           value.pos2.z(),
                           value.quat.x(),
                           value.quat.y(),
                           value.quat.z(),
                           value.quat.w()};
      return {pose1, pose2};
    } else {
      ROS_ERROR("[Subtask] - No stored region of interest ");
      return {{}, {}};
    }
  };

private:
  struct ROI {
    Eigen::Vector3d pos1{};
    Eigen::Vector3d pos2{};
    Eigen::Quaterniond quat{};
  };

  ros::NodeHandle nh_;
  ros::Subscriber subROI_;
  std::map<double, ROI> roiMap_;

  const Eigen::Vector3d robotPos_ = Eigen::Vector3d(0, 0, 0);
  const Eigen::Vector3d refVector_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  const double theta_ = -30 * M_PI / 180;

  bool parseROI_(const std::string& str);
  std::vector<double> splitString_(const std::string& str, const char delimiter);
  Eigen::Quaterniond rotateVectorInPlan_(const Eigen::Vector3d point1,
                                         const Eigen::Vector3d point2,
                                         const Eigen::Vector3d point3,
                                         const Eigen::Vector3d refVector,
                                         const double theta);
  // Debug
  ros::Publisher pubWaypoint1_;
  ros::Publisher pubWaypoint2_;
  ros::Publisher pubRobotBase_;
  ros::Publisher pubComputedQuat_;
  void publishPose_(const Eigen::Vector3d& pos, const Eigen::Quaterniond quat, ros::Publisher pub);
  void publishWaypoint_(const Eigen::Vector3d& pose, ros::Publisher pub);
};
