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

#include "convertion_tools.h"

namespace DebugTools {
std::string getPoseString(const geometry_msgs::Pose& pose) {
  std::string poseString = "";
  std::vector<double> poseVector = ConvertionTools::poseToVector(pose);

  // Convert Position to string
  poseString += "Position - xyz [";
  for (size_t i = 0; i < 3; i++) {
    poseString += std::to_string(poseVector[i]);

    if (i < 2) {
      poseString += ", ";
    } else {
      poseString += "] ";
    }
  }

  // Convert Orientation to string
  poseString += "Orientation - xyzw [";
  for (size_t i = 3; i < 7; i++) {
    poseString += std::to_string(poseVector[i]);

    if (i < 6) {
      poseString += ", ";
    } else {
      poseString += "]";
    }
  }

  return poseString;
}
} // namespace DebugTools