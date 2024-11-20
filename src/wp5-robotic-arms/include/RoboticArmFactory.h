/**
 * @file RoboticArmFactory.h
 * @author [Louis Munier] - lmunier@protonmail.com
 * @brief
 * @version 0.3
 * @date 2024-11-11
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */
#pragma once

#include <array>
#include <functional>
#include <iostream>
#include <map>
#include <memory>

#include "IRoboticArmBase.h"
#include "IRosInterfaceBase.h"
#include "RoboticArmUr.h"

class RoboticArmFactory {
public:
  // Prevent instantiation
  RoboticArmFactory() = delete;

  // Register a robotic arm
  static void registerRobotArm(const std::string& name,
                               std::function<std::unique_ptr<RoboticArmUr>(ROSVersion)> creator) {
    getRegistry()[name] = creator;
  }

  /**
   * @brief Creates an instance of a robotic arm based on its name.
   *
   * @param name The name of the robotic arm type.
   * @param rosVersion The ROS version to use.
   * @return A unique pointer to the created robotic arm instance.
   * @throws std::runtime_error if the specified robotic arm name is invalid.
   */
  static std::unique_ptr<IRoboticArmBase> createRoboticArm(const std::string& name, ROSVersion rosVersion) {
    auto it = getRegistry().find(name);

    if (it != getRegistry().end()) {
      return it->second(rosVersion);
    } else {
      std::ostringstream errorMsg{};
      errorMsg << "Invalid robotic arm type: " << name << ". Allowed values are [";

      for (const auto& type : getRoboticArmTypes()) {
        errorMsg << type << ", ";
      }
      errorMsg << "]";

      std::string errorMsgStr = errorMsg.str();
      errorMsgStr = errorMsgStr.substr(0, errorMsgStr.size() - 2) + ".";

      throw std::runtime_error(errorMsg.str());
    }
  }

  /**
   * @brief Retrieves the list of registered robotic arm types.
   *
   * @return A vector containing the names of the registered robotic arm types.
   */
  static std::vector<std::string> getRoboticArmTypes() {
    std::vector<std::string> keys;

    for (const auto& pair : getRegistry()) {
      keys.push_back(pair.first);
    }

    return keys;
  }

private:
  // Get the registry of robotic arms
  static std::unordered_map<std::string, std::function<std::unique_ptr<RoboticArmUr>(ROSVersion)>>& getRegistry() {
    static std::unordered_map<std::string, std::function<std::unique_ptr<RoboticArmUr>(ROSVersion)>> registry;
    return registry;
  }

  // Static initialization block
  static bool initialize() {
    registerRobotArm("ur5_robot", [](ROSVersion rosVersion) {
      return std::make_unique<RoboticArmUr>(
          rosVersion, "ur5_robot", std::string("robotic_arm.yaml"), UR_H_MATRIX, UR5_P_MATRIX);
    });
    registerRobotArm("ur10e_robot", [](ROSVersion rosVersion) {
      return std::make_unique<RoboticArmUr>(
          rosVersion, "ur10e_robot", std::string("robotic_arm.yaml"), UR_H_MATRIX, UR10E_P_MATRIX);
    });
    return true;
  }

  // Static member to trigger initialization only once
  static inline bool initialized = initialize();

  /**
   * @brief Robotic Arm H matrices.
   */
  static const std::array<double, 18> UR_H_MATRIX;

  /**
   * @brief Robotic Arm P matrices.
   */
  static const std::array<double, 21> UR5_P_MATRIX;
  static const std::array<double, 21> UR10E_P_MATRIX;
};