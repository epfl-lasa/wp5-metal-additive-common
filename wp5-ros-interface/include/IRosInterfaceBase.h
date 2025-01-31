/**
 * @file IRosInterfaceBase.h
 * @brief Create a ROS interface with respect to the ROS version to communicate with the robotic arm
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @author [Tristan Bonato] - tristan_bonato@hotmail.com
 * @version 0.1
 * @date 2024-03-07
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 */

#pragma once

#include <map>
#include <tuple>
#include <vector>

/**
 * @brief Enum representing different types of ROS versions.
 */
enum ROSVersion : int8_t {
  VERSION_UNDEFINED = -1, /**< Undefined ROS Version. */
  ROS1_NOETIC,            /**< ROS 1 Noetic. */
  ROS2_HUMBLE             /**< ROS 2 Humble. */
};

class IRosInterfaceBase {
public:
  /**
   * @brief Map of task names to task_type enums.
   */
  inline static const std::map<std::string, ROSVersion> rosVersionsMap{{"noetic", ROS1_NOETIC},
                                                                       {"humble", ROS2_HUMBLE}};

  /**
   * @brief Constructor for IRosInterfaceBase.
   */
  IRosInterfaceBase(ROSVersion rosVersion) : rosVersion_(rosVersion) {};

  /**
   * @brief Destructor.
   */
  virtual ~IRosInterfaceBase() = default;

  /**
   * @brief Get the ROS version.
   * @return ROS version.
   */
  const ROSVersion getROSVersion() const { return rosVersion_; }

  /**
   * @brief Get the current state of the robot.
   * @return Tuple containing joint positions, velocities, and torques.
   */
  virtual std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getState() const = 0;

  /**
   * @brief Set the state of the robot.
   * @param data Vector containing the state data.
   */
  virtual void setState(const std::vector<double>& data) = 0;

private:
  ROSVersion rosVersion_; /**< ROS version. */
};