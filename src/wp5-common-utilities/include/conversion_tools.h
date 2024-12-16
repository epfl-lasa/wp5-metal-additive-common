/**
 * @file conversion_tools.h
 * @brief A collection of utility functions for converting between different
 * representations of poses, points, and quaternions in ROS and Eigen.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */

#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

/**
 * @namespace ConversionTools
 * @brief A collection of utility functions for converting between different
 * representations of poses, points, and quaternions in ROS and Eigen.
 *
 * The conversions ensure that the data is accurately mapped between the
 * different representations, facilitating interoperability between ROS and
 * Eigen libraries.
 */
namespace ConversionTools {

/**
 * @brief Converts a geometry_msgs::Point to a std::vector<double>.
 *
 * This function takes a geometry_msgs::Point object and converts its x, y, and z
 * coordinates into a std::vector<double> containing three elements.
 *
 * @param point The geometry_msgs::Point object to be converted.
 * @return A std::vector<double> containing the x, y, and z coordinates of the point.
 */
std::vector<double> geometryToVector(const geometry_msgs::Point& point);

/**
 * @brief Converts a geometry_msgs::Vector3 to a std::vector<double>.
 *
 * This function takes a geometry_msgs::Vector3 object and converts its
 * x, y, and z components into a std::vector<double> with three elements.
 *
 * @param vector The input geometry_msgs::Vector3 object.
 * @return A std::vector<double> containing the x, y, and z components of the input vector.
 */
std::vector<double> geometryToVector(const geometry_msgs::Vector3& vector);

/**
 * @brief Converts a quaternion to a vector of doubles.
 *
 * This function takes a quaternion representing orientation and converts it
 * into a vector of doubles. The resulting vector typically contains the
 * quaternion's components in a specific order.
 *
 * @param orientation The quaternion to be converted, represented as a
 * geometry_msgs::Quaternion.
 * @return A std::vector<double> containing the quaternion's components.
 */
std::vector<double> geometryToVector(const geometry_msgs::Quaternion& orientation);

/**
 * @brief Converts a geometry_msgs::Pose to a vector of doubles.
 *
 * This function takes a geometry_msgs::Pose object and converts it to a vector of doubles.
 * The output vector contains the position and orientation of the pose, as a quaternion, in
 * the order x, y, z, qx, qy, qz, qw.
 *
 * @param pose The input geometry_msgs::Pose object to be converted.
 * @return A vector of doubles representing the pose. The vector contains either 6 elements
 */
std::vector<double> geometryToVector(const geometry_msgs::Pose& pose);

/**
 * @brief Converts a vector of doubles to a geometry_msgs::Point.
 *
 * This function takes a vector of doubles representing a point and converts it to a
 * geometry_msgs::Point object. The input vector should have exactly 3 elements (x, y, z).
 *
 * @param point A vector of doubles representing the point. The vector should have 3 elements.
 * @return A geometry_msgs::Point object representing the point. If the input vector does
 * not have 3 elements, an empty Point object is returned and an error is logged.
 */
geometry_msgs::Point vectorToGeometryPoint(const std::vector<double>& point);

/**
 * @brief Converts a vector of orientation values to a geometry_msgs::Quaternion.
 *
 * This function takes a vector of double values representing orientation and converts it into a
 * geometry_msgs::Quaternion object.
 *
 * @param orientation A vector of double values representing the orientation.
 * @return geometry_msgs::Quaternion The resulting quaternion representing the orientation.
 */
geometry_msgs::Quaternion vectorToGeometryQuat(const std::vector<double>& orientation);

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
geometry_msgs::Pose vectorToGeometryPose(const std::vector<double>& pose);

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

std::pair<Eigen::Quaterniond, Eigen::Vector3d> geometryToEigen(const geometry_msgs::Pose& pose);

/**
 * @brief Converts an Eigen::Vector3d to a geometry_msgs::Point.
 *
 * This function takes a 3D vector and a quaternion from the Eigen library and
 * converts it into a ROS geometry_msgs::Pose message. The x, y, and z components
 * of the Eigen vector and x, y, z and w of the quaternion are directly mapped to
 * the corresponding fields in the Pose message.
 *
 * @param orientation The Eigen::Quaterniond representing the orientation (x, y, z, w).
 * @param position The Eigen::Vector3d representing the position (x, y, z).
 * @return geometry_msgs::Pose The converted ROS Pose message.
 */
geometry_msgs::Pose eigenToGeometry(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position);

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

/**
 * @brief Converts an Eigen::Vector3d to a std::vector<double>.
 *
 * This function takes an Eigen::Vector3d object representing a 3-dimensional vector
 * and converts it into a std::vector<double> containing the same elements.
 *
 * @param position The Eigen::Vector3d object to be converted.
 * @return A std::vector<double> containing the elements of the input Eigen::Vector3d.
 */
std::vector<double> eigenToVector(const Eigen::Vector3d& position);

/**
 * @brief Converts an Eigen::Quaterniond to a std::vector<double>.
 *
 * This function takes an Eigen::Quaterniond representing an orientation and
 * converts it into a std::vector<double> containing the quaternion's coefficients (x, y, z, w).
 *
 * @param orientation The Eigen::Quaterniond to be converted.
 * @return A std::vector<double> containing the coefficients of the quaternion.
 */
std::vector<double> eigenToVector(const Eigen::Quaterniond& orientation);

/**
 * @brief Converts an Eigen::Vector3d and an Eigen::Quaterniond to a std::vector<double>.
 *
 * This function takes an Eigen::Vector3d representing a position and an Eigen::Quaterniond
 * representing an orientation, and converts them into a single std::vector<double>.
 *
 * @param orientation The Eigen::Quaterniond representing the orientation (x, y, z, w).
 * @param position The Eigen::Vector3d representing the position (x, y, z).
 * @return A std::vector<double> containing the elements of the position followed by the elements of the orientation.
 */
std::vector<double> eigenToVector(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position);

/**
 * @brief Converts a standard vector of doubles to an Eigen::Vector3d.
 *
 * This function takes a std::vector<double> representing a 3D position and converts it to an Eigen::Vector3d.
 *
 * @param position A std::vector<double> containing three elements representing the x, y, and z coordinates.
 * @return Eigen::Vector3d The corresponding Eigen::Vector3d representation of the input position.
 */
Eigen::Vector3d vectorToEigenVec(const std::vector<double>& position);

/**
 * @brief Converts a vector of doubles representing an orientation to an Eigen::Quaterniond.
 *
 * This function takes a std::vector<double> that represents an orientation and converts it
 * into an Eigen::Quaterniond object. The input vector should contain four elements corresponding
 * to the quaternion components (x, y, z, w).
 *
 * @param orientation A std::vector<double> containing four elements representing the quaternion components.
 * @return An Eigen::Quaterniond object representing the orientation.
 */
Eigen::Quaterniond vectorToEigenQuat(const std::vector<double>& orientation);

/**
 * @brief Converts a vector of doubles representing a quaternion and position into an Eigen quaternion and vector.
 *
 * This function takes a std::vector<double> containing 7 elements, where the first 4 elements represent the quaternion
 * (w, x, y, z) and the last 3 elements represent the position (x, y, z). It returns a std::pair containing an
 * Eigen::Quaterniond and an Eigen::Vector3d.
 *
 * @param quatPos A std::vector<double> containing 7 elements: the first 4 for the quaternion and the last 3 for the position.
 * @return std::pair<Eigen::Quaterniond, Eigen::Vector3d> A pair containing the Eigen quaternion and position vector.
 */
std::pair<Eigen::Quaterniond, Eigen::Vector3d> vectorToEigenQuatPose(const std::vector<double>& quatPos);

/**
 * @brief Converts a geometry_msgs::Transform to a geometry_msgs::Pose.
 *
 * This function takes a Transform message and converts it into a Pose message.
 * The translation component of the Transform is directly mapped to the position
 * component of the Pose, and the rotation component of the Transform is directly
 * mapped to the orientation component of the Pose.
 *
 * @param transform The input Transform message to be converted.
 * @return geometry_msgs::Pose The resulting Pose message after conversion.
 */
geometry_msgs::Pose transformToPose(const geometry_msgs::Transform& transform);

/**
 * @brief Converts Euler angles to a quaternion.
 *
 * This function converts a set of Euler angles to a quaternion.
 *
 * @tparam T The type of the Euler angles.
 * @param euler The Euler angles to convert.
 * @return The quaternion representation of the Euler angles.
 */
template <typename T>
Eigen::Quaternion<T> eulerToQuaternion(const std::array<T, 3>& euler) {
  Eigen::Quaternion<T> q;
  q = Eigen::AngleAxis<T>(euler[0], Eigen::Matrix<T, 3, 1>::UnitX())
      * Eigen::AngleAxis<T>(euler[1], Eigen::Matrix<T, 3, 1>::UnitY())
      * Eigen::AngleAxis<T>(euler[2], Eigen::Matrix<T, 3, 1>::UnitZ());

  return q;
}
} // namespace ConversionTools