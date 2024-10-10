/**
 * @file convertion_tools.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.2
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

#include "math_tools.h"

/**
 * @namespace ConvertionTools
 * @brief A collection of utility functions for converting between different
 * representations of poses, points, and quaternions in ROS and Eigen.
 *
 * This namespace provides functions to convert between:
 * - std::vector<double> and geometry_msgs::Pose
 * - geometry_msgs::Point and Eigen::Vector3d
 * - geometry_msgs::Quaternion and Eigen::Quaterniond
 * - Eigen::Vector3d and geometry_msgs::Point
 * - Eigen::Quaterniond and geometry_msgs::Quaternion
 *
 * The conversions ensure that the data is accurately mapped between the
 * different representations, facilitating interoperability between ROS and
 * Eigen libraries.
 */
namespace ConvertionTools {
/**
 * @brief Converts a vector of doubles to a geometry_msgs::Pose.
 *
 * This function takes a vector of doubles representing a pose and converts it to a
 * geometry_msgs::Pose object. The input vector can either have 6 elements (representing
 * position and Euler angles) or 7 elements (representing position and quaternion).
 *
 * @param pose A vector of doubles representing the pose. The vector should have either
 * 6 elements (x, y, z, roll, pitch, yaw) or 7 elements (x, y, z, qx, qy, qz, qw).
 * @return A geometry_msgs::Pose object representing the pose. If the input vector does
 * not have 6 or 7 elements, an empty Pose object is returned and an error is logged.
 */
geometry_msgs::Pose vectorToPose(const std::vector<double>& pose);

/**
 * @brief Converts a geometry_msgs::Point to an Eigen::Vector3d.
 *
 * This function takes a point from the geometry_msgs library and converts it
 * to a 3D vector from the Eigen library.
 *
 * @param point The input point of type geometry_msgs::Point.
 * @return Eigen::Vector3d The converted 3D vector.
 */
Eigen::Vector3d geometryToEigen(const geometry_msgs::Point& point);

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
Eigen::Quaterniond geometryToEigen(const geometry_msgs::Quaternion& orientation);

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
geometry_msgs::Point eigenToGeometry(const Eigen::Vector3d& position);

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
geometry_msgs::Quaternion eigenToGeometry(const Eigen::Quaterniond& orientation);
} // namespace ConvertionTools