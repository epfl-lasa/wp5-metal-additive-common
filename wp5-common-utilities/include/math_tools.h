/**
 * @file math_tools.h
 * @brief A collection of mathematical utility functions to centralize common operations.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.4
 * @date 2024-11-20
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @namespace MathTools
 * @brief A collection of mathematical utility functions.
 *
 * This namespace contains functions for various mathematical checks and operations.
 */
namespace MathTools {
const double TOLERANCE = 2e-4;

class TfBufferSingleton {
public:
  static TfBufferSingleton& getInstance() {
    static TfBufferSingleton instance;
    return instance;
  }

  tf2_ros::Buffer& getBuffer() { return tfBuffer; }

private:
  TfBufferSingleton() : tfListener(tfBuffer) {}

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
};

/**
 * @brief Checks if a given string represents a number.
 *
 * This function verifies whether the provided string can be interpreted as a valid number.
 *
 * @param str The string to check.
 * @return true if the string is a number, false otherwise.
 */
const bool isNumber(const std::string& str);

/**
 * @brief Converts radians to degrees.
 *
 * This function takes an angle in radians and converts it to degrees.
 *
 * @param rad The angle in radians.
 * @return The angle in degrees.
 */
template <typename T>
constexpr double radToDeg(const T& rad) {
  return rad * 180.0 / M_PI;
}

/**
 * @brief Converts degrees to radians.
 *
 * This function takes an angle in degrees and converts it to radians.
 *
 * @param deg The angle in degrees.
 * @return The angle in radians.
 */
template <typename T>
constexpr double degToRad(const T& deg) {
  return deg * M_PI / 180.0;
}

/**
 * @brief Checks if two quaternions are equivalent within a given tolerance.
 *
 * This function compares two quaternions by converting them to rotation matrices and
 * checking if the difference between the matrices is within the specified tolerance.
 *
 * @param q1 The first quaternion.
 * @param q2 The second quaternion.
 * @param tolerance The tolerance within which the quaternions are considered equivalent.
 * @return true if the quaternions are equivalent within the given tolerance, false otherwise.
 */
const bool areQuatEquivalent(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double tolerance = TOLERANCE);

/**
 * @brief Checks if two positions are equivalent within a given tolerance.
 *
 * This function compares two 3D positions (vectors) and checks if the difference
 * between them is within the specified tolerance.
 *
 * @param p1 The first position vector.
 * @param p2 The second position vector.
 * @param tolerance The tolerance within which the positions are considered equivalent.
 * @return true if the positions are equivalent within the given tolerance, false otherwise.
 */
const bool arePosEquivalent(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double tolerance = TOLERANCE);

/**
 * @brief Checks if two poses are equivalent.
 *
 * This function compares two geometry_msgs::Pose objects to determine if they are equivalent.
 *
 * @param pose1 The first pose to compare.
 * @param pose2 The second pose to compare.
 * @return true if the poses are equivalent, false otherwise.
 */
const bool arePoseEquivalent(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);

/**
 * @brief Retrieves the transformation between two coordinate frames.
 *
 * This function attempts to get the transformation from the source frame to the target frame
 * and stores it in the provided transform object.
 *
 * @param sourceFrame The name of the source coordinate frame.
 * @param targetFrame The name of the target coordinate frame.
 * @param transform The TransformStamped object where the resulting transformation will be stored.
 * @return true if the transformation was successfully retrieved, false otherwise.
 */
bool getTransform(const std::string& sourceFrame,
                  const std::string& targetFrame,
                  geometry_msgs::TransformStamped& transform);

/**
 * @brief Transforms a pose from the source frame to the target frame using a TF listener.
 *
 * This function utilizes the tf::TransformListener to transform a pose from the specified
 * source frame to the target frame. It is essential for ensuring that the pose is correctly
 * transformed according to the latest available transform data.
 *
 * @param sourceFrame The name of the frame from which the pose is to be transformed.
 * @param targetFrame The name of the frame to which the pose is to be transformed.
 * @param pose The input geometry_msgs::Pose to be transformed.
 * @return The transformed geometry_msgs::Pose in the target frame.
 */
geometry_msgs::Pose transformPose(const std::string& sourceFrame,
                                  const std::string& targetFrame,
                                  const geometry_msgs::Pose& pose);

/**
 * @brief Adds an offset to a given quaternion and position pair.
 *
 * This function takes a pair of quaternion and position (without offset) and
 * adds the specified offset to it, returning the resulting quaternion and position pair.
 *
 * @param quatPosNoOffset A pair consisting of a quaternion (Eigen::Quaterniond) and a position (Eigen::Vector3d) without offset.
 * @param offset A pair consisting of a quaternion (Eigen::Quaterniond) and a position (Eigen::Vector3d) representing the offset to be added.
 * @return A pair consisting of the resulting quaternion (Eigen::Quaterniond) and position (Eigen::Vector3d) after adding the offset.
 */
std::pair<Eigen::Quaterniond, Eigen::Vector3d> addOffset(
    const std::pair<Eigen::Quaterniond, Eigen::Vector3d>& quatPosNoOffset,
    const std::pair<Eigen::Quaterniond, Eigen::Vector3d>& offset);

/**
 * @brief Adds an offset to a given pose.
 *
 * This function takes a pose without an offset and applies a specified offset
 * to it. The offset is provided as a pair consisting of a quaternion (for
 * rotational offset) and a vector (for translational offset).
 *
 * @param poseNoOffset The original pose without any offset.
 * @param offset The offset pose to add
 * @return geometry_msgs::Pose The pose with the applied offset.
 */
geometry_msgs::Pose addOffset(const geometry_msgs::Pose& poseNoOffset, const geometry_msgs::Pose& offset);

/**
 * @brief Computes the normal vector to a plane formed by three points.
 *
 * @param pointsArray An array of three Eigen::Vector3d points that define the plane.
 * @return Eigen::Vector3d The resulting normal vector to the plane.
 */
const Eigen::Vector3d getNormalFromPlan(const std::array<Eigen::Vector3d, 3>& pointsArray);

/**
 * @brief Computes a quaternion representing a rotation around a plane defined by a normal vector and an angle theta.
 *
 * This function calculates the quaternion that represents a rotation around a plane specified by the given normal vector.
 * The rotation is performed by the angle theta.
 *
 * @param normalVector The normal vector defining the plane of rotation.
 * @param theta The angle of rotation around the plane, in radians.
 * @return Eigen::Quaterniond The resulting quaternion representing the rotation.
 */
const Eigen::Quaterniond getQuatFromNormalTheta(const Eigen::Vector3d normalVector, const double theta);
} // namespace MathTools