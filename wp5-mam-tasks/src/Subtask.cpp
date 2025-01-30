/**
 * @file Subtask.cpp
 * @brief Declaration of the Subtask class
 *
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 *
 * @version 0.2
 * @date 2024-10-22
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#include "Subtask.h"

#include "conversion_tools.h"
#include "debug_tools.h"

using namespace std;

Subtask::Subtask(ros::NodeHandle& nh) : nh_(nh) {
  subROI_ = nh_.subscribe("/damage_string", 100, &Subtask::cbkROI_, this);
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
  string refFrame = "";
  vector<double> waypointsPos{};
  vector<double> roiNormal{};
  bool unpackedSuccess = waypointParser_.unpackWaypoint(str, waypointID, refFrame, waypointsPos, roiNormal);

  if (unpackedSuccess && waypointsPos.size() != MSG_POS_SIZE) {
    ROS_ERROR_STREAM("[Subtask] - Waypoint ROS message " << str << " doesn't have the correct size, should be "
                                                         << MSG_POS_SIZE << " instead of " << waypointsPos.size());
  }

  // Store ROI only if it is not already done
  if (!isROIStored_(waypointID)) {
    ROI roi(waypointID);

    Eigen::Vector3d posStart(waypointsPos[0], waypointsPos[1], waypointsPos[2]);
    Eigen::Vector3d posEnd(waypointsPos[3], waypointsPos[4], waypointsPos[5]);
    Eigen::Vector3d normal(roiNormal[0], roiNormal[1], roiNormal[2]);

    roi.emplaceBackPose(refFrame, posStart, normal);
    roi.emplaceBackPose(refFrame, posEnd, normal);
    dequeROI_.push_back(roi);

    ROS_INFO_STREAM("[Subtask] - Waypoint registered");
    roi.print();
  } else {
    ROS_INFO_STREAM("[Subtask] - Waypoint received previously, already registered, key : " << waypointID);
  }

#ifdef DEBUG_MODE
  std::vector<Eigen::Vector3d> waypoints;
  std::array<float, 4> color = {1.0, 0.0, 0.0, 1.0};

  for (const auto& roi : dequeROI_) {
    const std::vector<Eigen::Vector3d> positions = roi.getPositions();

    if (!positions.empty()) {
      waypoints.insert(waypoints.end(), positions.begin(), positions.end());
    } else {
      ROS_WARN("[Subtask] - ROI has no positions.");
    }
  }

  DebugTools::publishWaypoints(refFrame, waypoints, waypointsPub_, color);
#endif
}

const bool Subtask::isROIStored_(const string& id) const {
  for (const auto& roi : dequeROI_) {
    if (roi.getID() == id) {
      return true;
    }
  }
  return false;
}

void Subtask::cbkROI_(const std_msgs::String::ConstPtr& msg) { parseROI_(msg->data); }
