/**
 * @file convertion_tools.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
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
inline geometry_msgs::Pose vectorToPose(const std::vector<double>& pose) {
  if (pose.size() != 6 && pose.size() != 7) {
    ROS_ERROR("Invalid pose size it should be 6 for Euler use or 7 for Quaternions.");
    return geometry_msgs::Pose();
  }

  geometry_msgs::Pose newPose;
  newPose.position.x = pose[0];
  newPose.position.y = pose[1];
  newPose.position.z = pose[2];

  if (pose.size() == 6) {
    Eigen::Quaterniond q = MathTools::eulerToQuaternion<double>({pose[3], pose[4], pose[5]});

    newPose.orientation.x = q.x();
    newPose.orientation.y = q.y();
    newPose.orientation.z = q.z();
    newPose.orientation.w = q.w();
  } else if (pose.size() == 7) {
    newPose.orientation.x = pose[3];
    newPose.orientation.y = pose[4];
    newPose.orientation.z = pose[5];
    newPose.orientation.w = pose[6];
  }

  return newPose;
}

/**
 * @brief Converts a geometry_msgs::Point to an Eigen::Vector3d.
 *
 * This function takes a point from the geometry_msgs library and converts it
 * to a 3D vector from the Eigen library.
 *
 * @param point The input point of type geometry_msgs::Point.
 * @return Eigen::Vector3d The converted 3D vector.
 */
inline Eigen::Vector3d geometryToEigen(const geometry_msgs::Point& point) {
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
inline Eigen::Quaterniond geometryToEigen(const geometry_msgs::Quaternion& orientation) {
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
inline geometry_msgs::Point eigenToGeometry(const Eigen::Vector3d& position) {
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
inline geometry_msgs::Quaternion eigenToGeometry(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion quat;
  quat.x = orientation.x();
  quat.y = orientation.y();
  quat.z = orientation.z();
  quat.w = orientation.w();

  return move(quat);
}
} // namespace ConvertionTools