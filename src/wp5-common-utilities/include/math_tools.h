/**
 * @file math_tools.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.2
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>

/**
 * @namespace MathTools
 * @brief A collection of mathematical utility functions.
 *
 * This namespace contains functions for various mathematical checks and operations.
 */
namespace MathTools {
const double TOLERANCE = 1e-5;

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

} // namespace MathTools