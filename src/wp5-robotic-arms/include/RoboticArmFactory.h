#include <functional>
#include <iostream>
#include <map>
#include <memory>

#include "IRoboticArmBase.h"
#include "RoboticArmCr7.h"
#include "RoboticArmUr5.h"

class RoboticArmFactory {
public:
  using FactoryFunction = std::function<std::unique_ptr<IRoboticArmBase>()>;

  //TODO(lmunier) : Add compatibility with the other robotic arms
  RoboticArmFactory() {
    registerRobotArm("ur5_robot", []() { return std::make_unique<RoboticArmUr5>(); });
    registerRobotArm("xMateCR7", []() { return std::make_unique<RoboticArmCr7>(); });
  }

  void registerRobotArm(std::string name, FactoryFunction function) { factoryFunctionRegistry[name] = function; }

  std::unique_ptr<IRoboticArmBase> createRoboticArm(std::string name) {
    auto it = factoryFunctionRegistry.find(name);

    if (it != factoryFunctionRegistry.end()) {
      return it->second();
    } else {
      throw std::runtime_error("Invalid name");
    }
  }

private:
  std::map<std::string, FactoryFunction> factoryFunctionRegistry;
};