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

    return move(keys);
  }

private:
  std::map<std::string, FactoryFunction> factoryFunctionRegistry;
};