/**
 * @file math_tools.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-01
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
inline const bool isNumber(const std::string& str) {
  if (str.empty()) {
    return false;
  }

  std::istringstream iss(str);
  double value;
  iss >> std::noskipws >> value; // noskipws considers leading whitespace invalid

  return iss.eof() && !iss.fail();
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
inline const bool areQuatEquivalent(const Eigen::Quaterniond& q1,
                                    const Eigen::Quaterniond& q2,
                                    double tolerance = TOLERANCE) {
  Eigen::Matrix3d rot1 = q1.toRotationMatrix();
  Eigen::Matrix3d rot2 = q2.toRotationMatrix();

  return (rot1 - rot2).norm() < tolerance;
}

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
inline const bool arePosEquivalent(const Eigen::Vector3d& p1,
                                   const Eigen::Vector3d& p2,
                                   double tolerance = TOLERANCE) {
  return (p1 - p2).norm() < tolerance;
}

} // namespace MathTools