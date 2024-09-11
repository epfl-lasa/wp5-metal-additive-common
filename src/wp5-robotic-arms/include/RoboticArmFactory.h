#include <functional>
#include <iostream>
#include <map>
#include <memory>

#include "IRoboticArmBase.h"
#include "IRosInterfaceBase.h"
#include "RoboticArmCr7.h"
#include "RoboticArmUr5.h"

class RoboticArmFactory {
public:
  using FactoryFunction = std::function<std::unique_ptr<IRoboticArmBase>(ROSVersion rosVersion)>;

  RoboticArmFactory() {
    registerRobotArm("ur5_robot", [](ROSVersion rosVersion) { return std::make_unique<RoboticArmUr5>(rosVersion); });
    registerRobotArm("xMateCR7", [](ROSVersion rosVersion) { return std::make_unique<RoboticArmCr7>(rosVersion); });
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
      throw std::runtime_error("Invalid name");
    }
  }

private:
  std::map<std::string, FactoryFunction> factoryFunctionRegistry;
};