/**
 * @file Subtask.h
 * @brief Declaration of the Subtask class
 *
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 *
 * @version 0.2
 * @date 2024-10-22
 * @copyright Copyright (c) 2024 - EPFL - LASA
 */

#pragma once

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <array>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "ROI.h"
#include "WaypointParser.h"
#include "math_tools.h"

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
   * @brief Delete all stored ROI
   */
  void clearROI() { dequeROI_.clear(); }

  /**
   * @brief True if no region of interest
   *
   * @return bool True if no region of interest
   */
  const bool empty() const { return dequeROI_.empty(); }

  /**
   * @brief Get the first region of interest and deletes it
   *
   * @return std::optional<ROI> The first region of interest
   */
  const std::optional<ROI> popROI();

private:
  ros::NodeHandle nh_;
  ros::Subscriber subROI_;
  std::deque<ROI> dequeROI_;

  const Eigen::Vector3d robotPos_ = Eigen::Vector3d::Zero();
  static constexpr double theta_ = MathTools::degToRad<int>(0);

  WaypointParser waypointParser_;

  void parseROI_(const std::string& str);
  const bool isROIStored_(const std::string& id) const;
  const Eigen::Quaterniond rotateVectorInPlan_(const std::array<Eigen::Vector3d, 3>& pointsArray,
                                               const double theta = theta_);
  // Debug
  ros::Publisher pubWaypoint_;
  /**
   * @brief Callback to get the region of interests.
   */
  void cbkROI_(const std_msgs::String::ConstPtr& msg);
};
