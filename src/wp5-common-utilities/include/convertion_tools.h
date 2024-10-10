/**
 * @file convertion_tools.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

namespace ConvertionTools {
/**
 * @brief Converts a geometry_msgs::Point to an Eigen::Vector3d.
 *
 * This function takes a point from the geometry_msgs library and converts it
 * to a 3D vector from the Eigen library.
 *
 * @param point The input point of type geometry_msgs::Point.
 * @return Eigen::Vector3d The converted 3D vector.
 */
Eigen::Vector3d geometryToEigen(const geometry_msgs::Point& point) {
  return Eigen::Vector3d{point.x, point.y, point.z};
}

/**
 * @brief Converts a geometry_msgs::Quaternion to an Eigen::Quaterniond.
 *
 * This function takes a quaternion from the geometry_msgs library and converts it
 * to a quaternion from the Eigen library. The conversion maintains the same
 * orientation by mapping the w, x, y, and z components appropriately.
 *
 * @param orientation The input quaternion from the geometry_msgs library.
 * @return An Eigen::Quaterniond representing the same orientation as the input.
 */
Eigen::Quaterniond geometryToEigen(const geometry_msgs::Quaternion& orientation) {
  return Eigen::Quaterniond{orientation.w, orientation.x, orientation.y, orientation.z};
}

/**
 * @brief Converts an Eigen::Vector3d to a geometry_msgs::Point.
 *
 * This function takes a 3D vector from the Eigen library and converts it into
 * a ROS geometry_msgs::Point message. The x, y, and z components of the Eigen
 * vector are directly mapped to the corresponding fields in the Point message.
 *
 * @param position The Eigen::Vector3d representing the position.
 * @return geometry_msgs::Point The converted ROS Point message.
 */
geometry_msgs::Point eigenToGeometry(const Eigen::Vector3d& position) {
  geometry_msgs::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();

  return move(point);
}

/**
 * @brief Converts an Eigen::Quaterniond to a geometry_msgs::Quaternion.
 *
 * This function takes a quaternion from the Eigen library and converts it to a
 * quaternion from the geometry_msgs library. The conversion maintains the same
 * orientation by mapping the x, y, z, and w components appropriately.
 *
 * @param orientation The input quaternion from the Eigen library.
 * @return geometry_msgs::Quaternion The converted ROS Quaternion message.
 */
geometry_msgs::Quaternion eigenToGeometry(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion quat;
  quat.x = orientation.x();
  quat.y = orientation.y();
  quat.z = orientation.z();
  quat.w = orientation.w();

  return move(quat);
}
} // namespace ConvertionTools