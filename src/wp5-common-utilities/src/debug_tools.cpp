/**
 * @file debug_tools.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-29
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "debug_tools.h"

#include "conversion_tools.h"

namespace DebugTools {
std::string getPoseString(const geometry_msgs::Pose& pose) {
  std::string poseString = "";

  // Convert Position to string
  poseString += "Position - xyz " + getVecString<double>(ConversionTools::geometryToVector(pose.position)) + " ";

  // Convert Orientation to string
  poseString += "Orientation - xyzw " + getVecString<double>(ConversionTools::geometryToVector(pose.orientation));

  return poseString;
}
} // namespace DebugTools