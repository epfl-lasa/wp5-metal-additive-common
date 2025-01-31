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
  /**
   * @brief Enum class to represent the status of the unpacking process
   */
  enum class ParseStatus { NOT_STARTED = -1, ID_TAKEN, REF_FRAME_TAKEN, POS_TAKEN, NORMAL_TAKEN };

  /**
   * @brief Pack the waypoint into a determined string
   *
   * @param waypointID The ID of the waypoint to pack
   * @param refFrame The reference frame of the waypoint to pack
   * @param waypointsPos The position of the waypoint to pack
   * @param normal The normal vector of the waypoint to pack
   * @param waypointPacked The resulting packed waypoint string
   * @param delimiter (Optional) The delimiter to use to pack the waypoint [default: ',']
   */
  void packWaypoint(const std::string& waypointID,
                    const std::string& refFrame,
                    const std::vector<double>& waypointsPos,
                    const std::vector<double>& normal,
                    std::string& waypointPacked,
                    const char delimiter = ',');

  /**
   * @brief Unpack the waypoint and store the different values in given variables
   *
   * @param strToUnpack The string to unpack
   * @param waypointID The ID of the waypoint to unpack
   * @param refFrame The reference frame of the waypoint to unpack
   * @param waypointsPos The position of the waypoint to unpack
   * @param normal The normal vector of the waypoint to unpack
   * @param delimiter (Optional) The delimiter to use to unpack the waypoint [default: ',']
   */
  bool unpackWaypoint(const std::string& strToUnpack,
                      std::string& waypointID,
                      std::string& refFrame,
                      std::vector<double>& waypointsPos,
                      std::vector<double>& normal,
                      const char delimiter = ',');
};