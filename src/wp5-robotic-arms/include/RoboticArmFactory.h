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
#include "RoboticArmCr7.h"
#include "RoboticArmUr.h"

class RoboticArmFactory {
public:
  using FactoryFunction = std::function<std::unique_ptr<IRoboticArmBase>(ROSVersion rosVersion)>;

  RoboticArmFactory() {
    registerRobotArm("ur5_robot", [this](ROSVersion rosVersion) {
      return std::make_unique<RoboticArmUr>(
          rosVersion, "ur5_robot", std::string("robotic_arm.yaml"), UR_H_MATRIX, UR5_P_MATRIX);
    });
    registerRobotArm("ur10e_robot", [this](ROSVersion rosVersion) {
      return std::make_unique<RoboticArmUr>(
          rosVersion, "ur10e_robot", std::string("robotic_arm.yaml"), UR_H_MATRIX, UR10E_P_MATRIX);
    });
    registerRobotArm("xMateCR7", [](ROSVersion rosVersion) {
      return std::make_unique<RoboticArmCr7>(rosVersion, std::string("robotic_arm.yaml"));
    });
  }

  /**
   * @brief Registers a robotic arm with its corresponding factory function.
   *
   * @param name The name of the robotic arm type.
   * @param function The factory function that creates instances of the robotic arm type.
   */
  void registerRobotArm(std::string name, FactoryFunction function) { factoryFunctionRegistry[name] = function; }

  /**
   * @brief Creates an instance of a robotic arm based on its name.
   *
   * @param name The name of the robotic arm type.
   * @param rosVersion The ROS version to use.
   * @return A unique pointer to the created robotic arm instance.
   * @throws std::runtime_error if the specified robotic arm name is invalid.
   */
  std::unique_ptr<IRoboticArmBase> createRoboticArm(std::string name, ROSVersion rosVersion) {
    auto it = factoryFunctionRegistry.find(name);

    if (it != factoryFunctionRegistry.end()) {
      return it->second(rosVersion);
    } else {
      std::ostringstream errorMsg{}, oss{};

      std::vector<std::string> allowedValues = getRoboticArmTypes();
      std::copy(allowedValues.begin(), allowedValues.end() - 1, std::ostream_iterator<std::string>(oss, ", "));

      oss << allowedValues.back();
      errorMsg << "Invalid robotic arm type: " << name << ". Allowed values are " << oss.str() << ".";

      throw std::runtime_error(errorMsg.str());
    }
  }

  /**
   * @brief Retrieves the list of registered robotic arm types.
   *
   * @return A vector containing the names of the registered robotic arm types.
   */
  std::vector<std::string> getRoboticArmTypes() {
    std::vector<std::string> keys;

    for (const auto& pair : factoryFunctionRegistry) {
      keys.push_back(pair.first);
    }

    return keys;
  }

private:
  std::map<std::string, FactoryFunction> factoryFunctionRegistry;

  // clang-format off
  /**
   * @brief generic UR H matrix.
   *
   * The H matrix defines the orientation of the joint axis, in the reference frame (identity frame),
   * in its home position.
   * => define the rotation axis for each joint, in the base frame.
   */

  const std::array<double, 18> UR_H_MATRIX{
      0.0, 0.0, 1.0,
      1.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, -1.0,
      1.0, 0.0, 0.0
  };

  /**
   * The P matrix defines the position of the joint axis, in the reference frame (identity frame),
   * in its home position.
   * => define the position of each joint, with respect to the previous one, in the base frame.
   */

  /**
   * @brief UR5 P matrix.
   */
  const std::array<double, 21> UR5_P_MATRIX{
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
  const std::array<double, 21> UR10E_P_MATRIX{
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.1807,
      0.0, -0.6127, 0.0,
      0.0, -0.57155, 0.0,
      0.17415, 0.0, 0.0,
      0.0, 0.0, -0.11985,
      0.11655, 0.0, 0.0
  };
  // clang-format on
};