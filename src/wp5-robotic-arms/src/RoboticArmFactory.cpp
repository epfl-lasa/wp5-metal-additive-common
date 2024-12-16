/**
 * @file RoboticArmFactory.cpp
 * @brief Factory class to create instances of robotic arms.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.3
 * @date 2024-11-20
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#include "RoboticArmFactory.h"

// clang-format off
/**
   * @brief Robotic Arm H matrices.
   *
   * The H matrix defines the orientation of the joint axis, in the reference frame (identity frame),
   * in its home position.
   * => define the rotation axis for each joint, in the base frame.
   */

const std::array<double, 18> RoboticArmFactory::UR_H_MATRIX{
    0.0, 0.0, 1.0,
    1.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, -1.0,
    1.0, 0.0, 0.0
};

  /**
   * @brief Robotic Arm P matrices.
   *
   * The P matrix defines the position of the joint axis, in the reference frame (identity frame),
   * in its home position.
   * => define the position of each joint, with respect to the previous one, in the base frame.
   */

/**
 * @brief UR5 P matrix.
 */
const std::array<double, 21> RoboticArmFactory::UR5_P_MATRIX{
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0892,
    0.0, -0.425, 0.0,
    0.0, -0.3922, 0.0,
    0.1091, 0.0, 0.0,
    0.0, 0.0, -0.0946,
    0.1173, 0.0, 0.0 // Adding sensor link offset
};

/**
 * @brief UR10e P matrix.
 */
const std::array<double, 21> RoboticArmFactory::UR10E_P_MATRIX{
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.1807,
    0.0, -0.6127, 0.0,
    0.0, -0.57155, 0.0,
    0.17415, 0.0, 0.0,
    0.0, 0.0, -0.11985,
    0.11655, 0.0, 0.0
    // TODO(lmunier) - Test again for the offset.
    // 0.42555, -0.0226, 0.0 // Adding tool offset - setup for welding laser head
};
// clang-format on