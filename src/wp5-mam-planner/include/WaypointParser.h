/**
 * @file WaypointParser.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
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

private:
  const bool isNumber_(const std::string& str);
};