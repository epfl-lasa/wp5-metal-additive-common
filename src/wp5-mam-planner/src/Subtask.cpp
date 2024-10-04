/**
 * @file Subtask.cpp
 * @brief Declaration of the Subtask class
 * @author [Elise Jeandupeux]
 * @date 2024-10-03
 */

#include "Subtask.h"

Subtask::Subtask(ros::NodeHandle& nh) : nh_(nh) {
  subROI_ = nh_.subscribe("/ur5/roi_topic", 1000, &Subtask::roiCallback, this);

  // debug waypoint
  pubWaypoint1_ = nh_.advertise<geometry_msgs::PointStamped>("debug_waypoint_1", 10);
  pubWaypoint2_ = nh_.advertise<geometry_msgs::PointStamped>("debug_waypoint_2", 10);
  pubRobotBase_ = nh_.advertise<geometry_msgs::PointStamped>("debug_robot_base", 10);
  pubComputedQuat_ = nh_.advertise<geometry_msgs::PoseStamped>("debug_computedQuat", 10);
}

void Subtask::publishWaypoint_(const Eigen::Vector3d& pos, ros::Publisher pub) {
  float TIME_WAIT = 0.05;
  size_t NB_PUBLISH = 3;
  const std::string frameID = "base_link";

  geometry_msgs::PointStamped pointStamped;
  pointStamped.header.stamp = ros::Time::now();
  pointStamped.header.frame_id = frameID;
  pointStamped.point.x = pos.x();
  pointStamped.point.y = pos.y();
  pointStamped.point.z = pos.z();

  for (size_t i = 0; i < NB_PUBLISH; ++i) {
    pub.publish(pointStamped);
    ros::Duration(TIME_WAIT).sleep();
  }
}

void Subtask::publishPose_(const Eigen::Vector3d& pos, const Eigen::Quaterniond quat, ros::Publisher pub) {
  float TIME_WAIT = 0.2;
  size_t NB_PUBLISH = 3;

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.header.frame_id = "base_link";
  poseStamped.pose.position.x = pos.x();
  poseStamped.pose.position.y = pos.y();
  poseStamped.pose.position.z = pos.z();

  poseStamped.pose.orientation.w = quat.w();
  poseStamped.pose.orientation.x = quat.x();
  poseStamped.pose.orientation.y = quat.y();
  poseStamped.pose.orientation.z = quat.z();

  for (size_t i = 0; i < NB_PUBLISH; ++i) {
    pub.publish(poseStamped);
    ros::Duration(TIME_WAIT).sleep();
  }
}

void Subtask::roiCallback(const std_msgs::String::ConstPtr& msg) { parseROI_(msg->data); }

bool Subtask::clearROI() {
  roiMap_.clear();
  return true;
}

bool Subtask::isSubtaskEmpty() { return roiMap_.empty(); }

bool Subtask::parseROI_(const std::string& str) {
  std::vector<double> splitMsg = splitString_(str, ',');

  if (splitMsg.size() != 7) {
    ROS_ERROR("[Subtask] - Waypoint ROS message doesn't have the correct size %s", str.c_str());
    return false;
  }

  // Store ROI only if it's not already done
  if (!(roiMap_.find(splitMsg[0]) != roiMap_.end())) {
    ROI roi;
    roi.pos1 = Eigen::Vector3d(splitMsg[1], splitMsg[2], splitMsg[3]);
    roi.pos2 = Eigen::Vector3d(splitMsg[4], splitMsg[5], splitMsg[6]);
    roi.quat = rotateVectorInPlan_(roi.pos1, roi.pos2, robotPos_, refVector_, theta_);
    roiMap_[splitMsg[0]] = roi;

    // Visualisation debug
    publishWaypoint_(roi.pos1, pubWaypoint1_);
    publishWaypoint_(roi.pos2, pubWaypoint2_);
    publishWaypoint_(robotPos_, pubRobotBase_);
    publishPose_(roi.pos1, roi.quat, pubComputedQuat_);
    return true;
  } else {
    std::cout << "[Subtask] - Waypoint received previously, not registered, key : " << splitMsg[0] << std::endl;
    return false;
  }
}

Eigen::Quaterniond Subtask::rotateVectorInPlan_(const Eigen::Vector3d point1,
                                                const Eigen::Vector3d point2,
                                                const Eigen::Vector3d point3,
                                                const Eigen::Vector3d refVector,
                                                const double theta) {

  // Compute vector normal to plan define by the 3 points
  const Eigen::Vector3d vectPlan1 = point1 - point2;
  const Eigen::Vector3d vectPlan2 = point1 - point3;
  const Eigen::Vector3d normalVect = vectPlan2.cross(vectPlan1).normalized();

  // Rotate vector defined by point1 and point2 by theta
  const Eigen::Vector3d axisRotationVectPlan = normalVect * sin(theta / 2);
  const Eigen::Quaterniond quatRotationVectPlan =
      Eigen::Quaterniond(cos(theta / 2), axisRotationVectPlan.x(), axisRotationVectPlan.y(), axisRotationVectPlan.z());
  const Eigen::Vector3d rotatedVectPlan1 = quatRotationVectPlan * vectPlan1;

  // Compute rotation in world frame
  Eigen::Vector3d axisFinalRot = refVector.cross(rotatedVectPlan1);
  axisFinalRot.normalize();
  const double angleFinalRot = acos(refVector.dot(rotatedVectPlan1) / (refVector.norm() * rotatedVectPlan1.norm()));
  const Eigen::Vector3d vectFinalRot = axisFinalRot * sin(angleFinalRot / 2.0);

  return Eigen::Quaterniond(cos(angleFinalRot / 2.0), vectFinalRot.x(), vectFinalRot.y(), vectFinalRot.z());
}

std::vector<double> Subtask::splitString_(const std::string& str, const char delimiter) {
  std::vector<double> stringArray;
  std::string token;
  std::stringstream ss(str);

  while (std::getline(ss, token, delimiter)) {
    stringArray.push_back(std::stod(token));
  }

  return stringArray;
}