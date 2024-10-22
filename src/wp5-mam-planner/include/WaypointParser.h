/**
 * @file WaypointParser.cpp
 * @brief Declaration of the WaypointParser class
 *
 * @author [Elise Jeandupeux]
 * @author [Louis Munier] - lmunier@protonmail.com
 *
 * @version 0.2
 * @date 2024-10-22
 * @copyright Copyright (c) 2024 - EPFL
 */
#pragma once

#include <string>
#include <vector>

class WaypointParser {
public:
  bool packWaypoint(const std::vector<std::string>& strToPack,
                    const char delimiter,
                    std::string& waypointID,
                    std::vector<double>& waypointsPos);

  bool unpackWaypoint(const std::string& strToUnpack,
                      const char delimiter,
                      std::string& waypointID,
                      std::vector<double>& waypointsPos);
};