/**
 * @file Subtask.h
 * @brief Declaration of the Subtask class
 *
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 *
 * @version 0.2
 * @date 2024-10-22
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#pragma once

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

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
  ros::NodeHandle nh_;       ///< ROS node handle.
  ros::Subscriber subROI_;   ///< Subscriber to the region of interest topic.
  std::deque<ROI> dequeROI_; ///< Deque of region of interest.

  WaypointParser waypointParser_; ///< Waypoint parser.

  /**
   * @brief Parse the region of interest string.
   *
   * @param str The region of interest string.
   */
  void parseROI_(const std::string& str);

  /**
   * @brief Check if the region of interest is already stored to avoid duplicates.
   *
   * @param id The region of interest id.
   * @return bool True if the region of interest is already stored.
   */
  const bool isROIStored_(const std::string& id) const;

  /**
   * @brief Callback to get the region of interests.
   */
  void cbkROI_(const std_msgs::String::ConstPtr& msg);

#ifdef DEBUG_MODE
  // Publisher for the stored waypoints for debugging purposes
  ros::Publisher waypointsPub_{nh_.advertise<visualization_msgs::Marker>("/debug_waypoints", 10)};
#endif
};
