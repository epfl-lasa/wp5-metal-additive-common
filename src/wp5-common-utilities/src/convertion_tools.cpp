/**
 * @file convertion_tools.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.2
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#include "convertion_tools.h"

namespace ConvertionTools {

geometry_msgs::Pose vectorToPose(const std::vector<double>& pose) {
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

Eigen::Vector3d geometryToEigen(const geometry_msgs::Point& point) {
  return Eigen::Vector3d{point.x, point.y, point.z};
}

Eigen::Quaterniond geometryToEigen(const geometry_msgs::Quaternion& orientation) {
  return Eigen::Quaterniond{orientation.w, orientation.x, orientation.y, orientation.z};
}

geometry_msgs::Pose eigenToGeometry(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  geometry_msgs::Pose pose;
  pose.position = eigenToGeometry(position);
  pose.orientation = eigenToGeometry(orientation);

  return move(pose);
}

geometry_msgs::Point eigenToGeometry(const Eigen::Vector3d& position) {
  geometry_msgs::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();

  return move(point);
}

geometry_msgs::Quaternion eigenToGeometry(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion quat;
  quat.x = orientation.x();
  quat.y = orientation.y();
  quat.z = orientation.z();
  quat.w = orientation.w();

  return move(quat);
}

} // namespace ConvertionTools