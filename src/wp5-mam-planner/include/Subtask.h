/**
 * @file Subtask.h
 * @brief Declaration of the Subtask class
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 * @date 2024-10-08
 */

#pragma once

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <array>
#include <deque>
#include <string>
#include <vector>

class Subtask {
private:
  struct ROI {
    std::string id{};
    Eigen::Vector3d posStart{};
    Eigen::Vector3d posEnd{};
    Eigen::Quaterniond quat{};

    // Function to check if the ROI is empty
    bool empty() const { return id.empty(); }

    // Function to clear the ROI
    void clear() {
      id.clear();
      posStart.setZero();
      posEnd.setZero();
      quat.setIdentity();
    }

    // Function to print the ROI
    void print() const {
      ROS_INFO_STREAM("ID: " << id.c_str());
      ROS_INFO_STREAM("Starting Position: " << posStart.x() << posStart.y() << posStart.z());
      ROS_INFO_STREAM("Ending Position: " << posEnd.x() << posEnd.y() << posEnd.z());
      ROS_INFO_STREAM("Quaternion: " << quat.x() << quat.y() << quat.z() << quat.w());
    }

    // Function to get the pose vector
    std::vector<double> getPoseVector(std::string posType) const {
      Eigen::Vector3d pos{};
      Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();

      if (posType == "start") {
        pos = posStart;
      } else if (posType == "end") {
        pos = posEnd;
      } else {
        ROS_ERROR("Invalid position type, should be either 'start' or 'end'.");
        return std::vector<double>();
      }

      return {pos.x(), pos.y(), pos.z(), identity.x(), identity.y(), identity.z(), identity.w()};
    }
  };

public:
  // Declare the test class as a friend to allow access to private members
  friend class SubtaskTest_TestSplitString_Test;

  /**
   * @brief Constructor.
   */
  Subtask(ros::NodeHandle& nh);

  /**
   * @brief Destructor.
   */
  ~Subtask() = default;

  /**
   * @brief Delete all stored ROI
   */
  void clearROI();

  /**
   * @brief True if no region of interest
   * @return bool True if no region of interest
   */
  const bool empty() const;

  /**
   * @brief Get the first region of interest and deletes it
   */
  const ROI getROI();

private:
  ros::NodeHandle nh_;
  ros::Subscriber subROI_;
  std::deque<ROI> dequeROI_;

  const Eigen::Vector3d robotPos_ = Eigen::Vector3d(0, 0, 0);
  const Eigen::Vector3d refVector_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  static constexpr double theta_ = -30 * M_PI / 180;

  void parseROI_(const std::string& str);
  const bool isIdStored_(const std::string& id) const;
  void splitString_(const std::string& str,
                    const char delimiter,
                    std::string& waypointID,
                    std::vector<double>& waypointsPos);
  const Eigen::Quaterniond rotateVectorInPlan_(const std::array<Eigen::Vector3d, 3>& pointsArray,
                                               const Eigen::Vector3d refVector,
                                               const double theta = theta_);
  // Debug
  ros::Publisher pubWaypoint1_;
  ros::Publisher pubWaypoint2_;
  ros::Publisher pubRobotBase_;
  ros::Publisher pubComputedQuat_;
  void publishPose_(const Eigen::Vector3d& pos, const Eigen::Quaterniond quat, ros::Publisher pub);
  void publishWaypoint_(const Eigen::Vector3d& pose, ros::Publisher pub);

  /**
   * @brief Callback to get the region of interests.
   */
  void cbkROI_(const std_msgs::String::ConstPtr& msg);
};
