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
#pragma once

#include <string>
#include <vector>

class WaypointParser {
public:
  enum class ParseStatus { NOT_STARTED = -1, ID_TAKEN, REF_FRAME_TAKEN, POS_TAKEN, NORMAL_TAKEN };

  void packWaypoint(const std::string& waypointID,
                    const std::string& refFrame,
                    const std::vector<double>& waypointsPos,
                    const std::vector<double>& normal,
                    std::string& waypointPacked,
                    const char delimiter = ',');

  bool unpackWaypoint(const std::string& strToUnpack,
                      std::string& waypointID,
                      std::string& refFrame,
                      std::vector<double>& waypointsPos,
                      std::vector<double>& normal,
                      const char delimiter = ',');
};