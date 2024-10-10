/**
 * @file math_tools.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.2
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "math_tools.h"

namespace MathTools {

const bool isNumber(const std::string& str) {
  if (str.empty()) {
    return false;
  }

  std::istringstream iss(str);
  double value;
  iss >> std::noskipws >> value; // noskipws considers leading whitespace invalid

  return iss.eof() && !iss.fail();
}

const bool areQuatEquivalent(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double tolerance) {
  Eigen::Matrix3d rot1 = q1.toRotationMatrix();
  Eigen::Matrix3d rot2 = q2.toRotationMatrix();

  return (rot1 - rot2).norm() < tolerance;
}

const bool arePosEquivalent(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double tolerance) {
  return (p1 - p2).norm() < tolerance;
}

} // namespace MathTools