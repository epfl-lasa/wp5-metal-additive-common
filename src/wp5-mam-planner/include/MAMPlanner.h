/**
 * @file MAMPlanner.h
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-03
 */

#pragma once

#include "ik_geo.h"

class MAMPlanner {
public:
  /**
   * @brief Constructor.
   */
  MAMPlanner();

  /**
   * @brief Plans the trajectory of the robot.
   */
  void planTrajectory();

  /**
   * @brief Executes the trajectory of the robot.
   */
  void executeTrajectory();
};