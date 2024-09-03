/**
 * @file MAMPlanner.cpp
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-03
 */

#include "MAMPlanner.h"

MAMPlanner::MAMPlanner() { ik_geo::Robot robot = ik_geo::Robot::ur5(); }

void MAMPlanner::planTrajectory() {}

void MAMPlanner::executeTrajectory() {}
