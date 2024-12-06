/**
 * @file Subtask.cpp
 * @brief Declaration of the Subtask class
 *
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 *
 * @version 0.2
 * @date 2024-10-22
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 */

#include "Subtask.h"

#include "conversion_tools.h"
#include "debug_tools.h"

using namespace std;

Subtask::Subtask(ros::NodeHandle& nh) : nh_(nh) {
  subROI_ = nh_.subscribe("/damage_string", 100, &Subtask::cbkROI_, this);

  // debug waypoint
  pubWaypoint_ = nh_.advertise<geometry_msgs::PoseStamped>("debug_waypoint", 10);
}

const optional<ROI> Subtask::popROI() {
  if (!empty()) {
    ROI roi = dequeROI_.front();
    dequeROI_.pop_front();
    return roi;
  } else {
    return nullopt;
  }
}

void Subtask::parseROI_(const string& str) {
  const size_t MSG_POS_SIZE = 6;

  string waypointID = "";
  vector<double> waypointsPos{};
  bool unpackedSuccess = waypointParser_.unpackWaypoint(str, ',', waypointID, waypointsPos);

  if (unpackedSuccess && waypointsPos.size() != MSG_POS_SIZE) {
    ROS_ERROR_STREAM("[Subtask] - Waypoint ROS message " << str << " doesn't have the correct size, should be "
                                                         << MSG_POS_SIZE << " instead of " << waypointsPos.size());
  }

  // Store ROI only if it is not already done
  if (!isROIStored_(waypointID)) {
    ROI roi(waypointID);

    // Compute quaternion to restrain the orientation in the plan defined by the 3 points
    Eigen::Vector3d posStart(waypointsPos[0], waypointsPos[1], waypointsPos[2]);
    Eigen::Vector3d posEnd(waypointsPos[3], waypointsPos[4], waypointsPos[5]);
    // Eigen::Quaterniond quat(rotateVectorInPlan_({posStart, posEnd, robotPos_}));
    Eigen::Quaterniond quat(0, 1, 0, 0);

    roi.emplaceBackPose("base_link", quat, posStart);
    roi.emplaceBackPose("base_link", quat, posEnd);

    dequeROI_.push_back(roi);

    ROS_INFO_STREAM("[Subtask] - Waypoint registered, key : " << waypointID);
  } else {
    ROS_INFO_STREAM("[Subtask] - Waypoint received previously, already registered, key : " << waypointID);
  }
}

const bool Subtask::isROIStored_(const string& id) const {
  for (const auto& roi : dequeROI_) {
    if (roi.getID() == id) {
      return true;
    }
  }
  return false;
}

const Eigen::Quaterniond Subtask::rotateVectorInPlan_(const array<Eigen::Vector3d, 3>& pointsArray,
                                                      const double theta) {
  const Eigen::Vector3d refVector = Eigen::Vector3d::UnitX();

  // Compute vector normal to plan define by the 3 points
  const Eigen::Vector3d planVecStart = pointsArray[0] - pointsArray[1];
  const Eigen::Vector3d planVecEnd = pointsArray[0] - pointsArray[2];
  const Eigen::Vector3d normalVector = planVecStart.cross(planVecEnd).normalized();

  // Rotate vector defined by plan, forming by points : pointsArray, by theta
  const Eigen::Quaterniond quatRotation(cos(theta / 2),
                                        normalVector.x() * sin(theta / 2),
                                        normalVector.y() * sin(theta / 2),
                                        normalVector.z() * sin(theta / 2));
  const Eigen::Vector3d rotatedVectPlan = quatRotation * planVecEnd;

  return Eigen::Quaterniond::FromTwoVectors(refVector, -rotatedVectPlan);
}

void Subtask::cbkROI_(const std_msgs::String::ConstPtr& msg) { parseROI_(msg->data); }
