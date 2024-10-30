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

geometry_msgs::Pose transformPose(const tf::TransformListener& listener,
                                  const string& source_frame,
                                  const string& target_frame,
                                  const geometry_msgs::Pose& pose) {
  if (source_frame == target_frame) {
    return pose;
  }

  // Define the target pose
  geometry_msgs::PoseStamped target_pose;

  // Define the pose in the source frame
  geometry_msgs::PoseStamped source_pose;
  source_pose.header.frame_id = source_frame;
  source_pose.header.stamp = ros::Time::now();
  source_pose.pose = pose;

  try {
    listener.transformPose(target_frame, source_pose, target_pose);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("[ConvertionTools] - Received an exception trying to transform a pose: %s", ex.what());
  }

  return target_pose.pose;
}

vector<double> poseToVector(const geometry_msgs::Pose& pose) {
  vector<double> newPose{pose.position.x,
                         pose.position.y,
                         pose.position.z,
                         pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w};
}

geometry_msgs::Pose vectorToPose(const vector<double>& pose) {
  if (pose.size() != 6 && pose.size() != 7) {
    ROS_ERROR("[ConvertionTools] - Invalid pose size it should be 6 for Euler use or 7 for Quaternions.");
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