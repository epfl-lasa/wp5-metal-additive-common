/**
 * @file WaypointParser.cpp
 * @brief Declaration of the WaypointParser class
 *
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 *
 * @version 0.2
 * @date 2024-10-22
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#include "WaypointParser.h"

#include <ros/ros.h>

#include <sstream>

#include "math_tools.h"

using namespace std;

void WaypointParser::packWaypoint(const string& waypointID,
                                  const string& refFrame,
                                  const vector<double>& waypointsPos,
                                  const vector<double>& normal,
                                  string& waypointPacked,
                                  const char delimiter) {
  waypointPacked = waypointID + "," + refFrame;

  for (const auto& pos : waypointsPos) {
    waypointPacked += "," + to_string(pos);
  }
}

bool WaypointParser::unpackWaypoint(const string& strToUnpack,
                                    string& waypointID,
                                    string& refFrame,
                                    vector<double>& waypointsPos,
                                    vector<double>& normal,
                                    const char delimiter) {
  bool waypointValid = true;
  ParseStatus idState = ParseStatus::NOT_STARTED;

  string token = "";
  stringstream ss(strToUnpack);

  // Clear previous values
  waypointID = "";
  refFrame = "";
  waypointsPos.clear();

  while (getline(ss, token, delimiter) && waypointValid) {
    switch (idState) {
      // Get the waypoint ID if it is well packed
      case ParseStatus::NOT_STARTED:
        if (MathTools::isNumber(token)) {
          ROS_ERROR_STREAM("[Subtask] - Invalid ID: " << token << " it should not only be a number.");
          waypointValid = false;
        } else {
          waypointID = token;
          idState = ParseStatus::ID_TAKEN;
        }

        break;

      // Get the reference frame if it is well packed
      case ParseStatus::ID_TAKEN:
        if (MathTools::isNumber(token)) {
          ROS_ERROR_STREAM("[Subtask] - Invalid refFrame: " << token << " it should be a string.");
          waypointValid = false;
        } else {
          refFrame = token;
          idState = ParseStatus::REF_FRAME_TAKEN;
        }

        break;

      // Get the positions if they are well packed
      case ParseStatus::REF_FRAME_TAKEN:
        if (MathTools::isNumber(token)) {
          waypointsPos.push_back(stod(token));
        } else {
          ROS_ERROR_STREAM("[Subtask] - Invalid position member argument: " << token << " it should be a number.");
          waypointValid = false;
        }

        if (waypointsPos.size() == 6) {
          idState = ParseStatus::POS_TAKEN;
        }

        break;

      // Get the normal if it is well packed
      case ParseStatus::POS_TAKEN:
        if (MathTools::isNumber(token)) {
          normal.push_back(stod(token));
        } else {
          ROS_ERROR_STREAM("[Subtask] - Invalid normal member argument: " << token << " it should be a number.");
          waypointValid = false;
        }

        if (normal.size() == 3) {
          idState = ParseStatus::NORMAL_TAKEN;
        }

        break;

      // Too many arguments
      case ParseStatus::NORMAL_TAKEN:
        ROS_ERROR_STREAM("[Subtask] - Too many arguments in the waypoint: " << token);
        waypointValid = false;
        break;

      default:
        break;
    }
  }

  // Check if the waypoint is valid
  if (idState != ParseStatus::NORMAL_TAKEN && waypointValid) {
    ROS_ERROR_STREAM("[Subtask] - Not enough arguments in the waypoint: " << strToUnpack);
    waypointValid = false;
  }

  // Clear values if the waypoint is not valid
  if (!waypointValid) {
    waypointID = "";
    refFrame = "";
    waypointsPos.clear();
    normal.clear();
  }

  return waypointValid;
}
