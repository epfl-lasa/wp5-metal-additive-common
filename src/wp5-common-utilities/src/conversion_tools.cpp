/**
 * @file conversion_tools.cpp
 * @author [Louis Munier] - lmunier@protonmail.com
 * @brief
 * @version 0.2
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */
#include "conversion_tools.h"

namespace ConversionTools {

std::vector<double> geometryToVector(const geometry_msgs::Point& point) {
  std::vector<double> newPoint{point.x, point.y, point.z};

  return newPoint;
}

std::vector<double> geometryToVector(const geometry_msgs::Quaternion& orientation) {
  std::vector<double> newOrientation{orientation.x, orientation.y, orientation.z, orientation.w};

  return newOrientation;
}

std::vector<double> geometryToVector(const geometry_msgs::Pose& pose) {
  std::vector<double> newPoint = geometryToVector(pose.position);
  std::vector<double> newOrientation = geometryToVector(pose.orientation);

  newPoint.insert(newPoint.end(), newOrientation.begin(), newOrientation.end());
  return newPoint;
}

geometry_msgs::Point vectorToGeometryPoint(const std::vector<double>& point) {
  if (point.size() != 3) {
    ROS_ERROR("[ConversionTools] - Invalid point size it should be 3.");
    return geometry_msgs::Point();
  }

  geometry_msgs::Point newPoint;
  newPoint.x = point[0];
  newPoint.y = point[1];
  newPoint.z = point[2];

  return newPoint;
}

geometry_msgs::Quaternion vectorToGeometryQuat(const std::vector<double>& orientation) {
  if (orientation.size() != 3 && orientation.size() != 4) {
    ROS_ERROR("[ConversionTools] - Invalid orientation size it should be 3 for Euler use or 4 for Quaternions.");
    return geometry_msgs::Quaternion();
  }

  geometry_msgs::Quaternion newOrientation{};
  if (orientation.size() == 3) {
    std::array<double, 3> orientationArray = {orientation[0], orientation[1], orientation[2]};
    newOrientation = eigenToGeometry(eulerToQuaternion<double>(orientationArray));
  } else if (orientation.size() == 4) {
    newOrientation.x = orientation[0];
    newOrientation.y = orientation[1];
    newOrientation.z = orientation[2];
    newOrientation.w = orientation[3];
  }

  return newOrientation;
}

geometry_msgs::Pose vectorToGeometryPose(const std::vector<double>& pose) {
  geometry_msgs::Pose newPose;
  newPose.position = vectorToGeometryPoint(std::vector<double>(pose.begin(), pose.begin() + 3));
  newPose.orientation = vectorToGeometryQuat(std::vector<double>(pose.begin() + 3, pose.end()));

  return newPose;
}

Eigen::Vector3d geometryToEigen(const geometry_msgs::Point& point) {
  return Eigen::Vector3d{point.x, point.y, point.z};
}

Eigen::Quaterniond geometryToEigen(const geometry_msgs::Quaternion& orientation) {
  return Eigen::Quaterniond{orientation.w, orientation.x, orientation.y, orientation.z};
}

geometry_msgs::Pose eigenToGeometry(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position) {
  geometry_msgs::Pose pose;
  pose.position = eigenToGeometry(position);
  pose.orientation = eigenToGeometry(orientation);

  return pose;
}

geometry_msgs::Point eigenToGeometry(const Eigen::Vector3d& position) {
  geometry_msgs::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();

  return point;
}

geometry_msgs::Quaternion eigenToGeometry(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion quat;
  quat.x = orientation.x();
  quat.y = orientation.y();
  quat.z = orientation.z();
  quat.w = orientation.w();

  return quat;
}

std::vector<double> eigenToVector(const Eigen::Vector3d& position) {
  std::vector<double> newPosition{position.x(), position.y(), position.z()};

  return newPosition;
}

std::vector<double> eigenToVector(const Eigen::Quaterniond& orientation) {
  std::vector<double> newOrientation{orientation.x(), orientation.y(), orientation.z(), orientation.w()};

  return newOrientation;
}

std::vector<double> eigenToVector(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position) {
  std::vector<double> newPosition = eigenToVector(position);
  std::vector<double> newOrientation = eigenToVector(orientation);

  newPosition.insert(newPosition.end(), newOrientation.begin(), newOrientation.end());
  return newPosition;
}

Eigen::Vector3d vectorToEigenVec(const std::vector<double>& position) {
  if (position.size() != 3) {
    ROS_ERROR("[ConversionTools] - Invalid position size, it should be 3.");
  }

  return Eigen::Vector3d{position[0], position[1], position[2]};
}

Eigen::Quaterniond vectorToEigenQuat(const std::vector<double>& orientation) {
  if (orientation.size() != 4) {
    ROS_ERROR("[ConversionTools] - Invalid orientation size, it should be 4 in xyzw order.");
  }

  return Eigen::Quaterniond{orientation[0], orientation[1], orientation[2], orientation[3]};
}

} // namespace ConversionTools