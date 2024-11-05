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

#include <geometry_msgs/PoseStamped.h>

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

void publishPose(const geometry_msgs::Pose& pose, const std::string& frameId, ros::Publisher& pub) {
  float TIME_WAIT = 0.2;
  size_t NB_PUBLISH = 3;

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = frameId;
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.pose = pose;

  for (size_t i = 0; i < NB_PUBLISH; ++i) {
    pub.publish(poseStamped);
    ros::Duration(TIME_WAIT).sleep();
  }
}
} // namespace DebugTools