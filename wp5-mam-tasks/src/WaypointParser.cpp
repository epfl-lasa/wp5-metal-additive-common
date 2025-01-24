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

bool WaypointParser::packWaypoint(const vector<string>& strToPack,
                                  const char delimiter,
                                  string& waypointID,
                                  vector<double>& waypointsPos) {
  string token = "";

  // Clear previous values
  waypointID = "";
  waypointsPos.clear();

  for (const auto& str : strToPack) {
    if (waypointID.empty()) {
      if (MathTools::isNumber(str)) {
        ROS_ERROR_STREAM("[Subtask] - Invalid ID: " << str << " should not only be a number.");
        return false;
      }

      waypointID = str;
    } else {
      if (MathTools::isNumber(str)) {
        waypointsPos.push_back(stod(str));
      } else {
        ROS_ERROR_STREAM("[Subtask] - Invalid argument: " << str << " should be a number.");
        return false;
      }
    }
  }

  return true;
}

bool WaypointParser::unpackWaypoint(const string& strToUnpack,
                                    const char delimiter,
                                    string& waypointID,
                                    vector<double>& waypointsPos) {
  bool idTaken = false;
  string token = "";
  stringstream ss(strToUnpack);

  // Clear previous values
  waypointID = "";
  waypointsPos.clear();

  while (getline(ss, token, delimiter)) {
    if (idTaken) {
      if (MathTools::isNumber(token)) {
        waypointsPos.push_back(stod(token));
      } else {
        ROS_ERROR_STREAM("[Subtask] - Invalid argument: " << token << " it should be a number.");
      }
    } else {
      if (MathTools::isNumber(token)) {
        ROS_ERROR_STREAM("[Subtask] - Invalid ID: " << token << " it should not only be a number.");
        waypointID = "";
        waypointsPos.clear();
        return false;
      } else {
        waypointID = token;
        idTaken = true;
      }
    }
  }

  return true;
}
