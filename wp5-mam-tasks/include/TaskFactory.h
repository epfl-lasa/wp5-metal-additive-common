// clang off
#include "ITaskBase.h"
// clang on

#include <ros/ros.h>

#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include "TaskCleaning.h"
#include "TaskWelding.h"

/**
 * @brief The TaskFactory class is responsible for creating instances of tasks based on their names.
 *
 * The TaskFactory class provides a mechanism to register different task types and their corresponding factory functions.
 * It allows creating instances of tasks by specifying their names and providing the necessary parameters.
 */
class TaskFactory {
public:
  using FactoryFunction = std::function<std::unique_ptr<ITaskBase>(ros::NodeHandle& nh, std::string configFilename)>;

  /**
   * @brief Constructs a TaskFactory object.
   *
   * The constructor initializes the factoryFunctionRegistry and registers the default task types.
   */
  TaskFactory() {
    registerTask("welding", [](ros::NodeHandle& nh, std::string configFilename) {
      return std::make_unique<TaskWelding>(nh, configFilename);
    });
    registerTask("cleaning", [](ros::NodeHandle& nh, std::string configFilename) {
      return std::make_unique<TaskCleaning>(nh, configFilename);
    });
  }

  /**
   * @brief Registers a task type with its corresponding factory function.
   *
   * @param name Name of the task type.
   * @param function Factory function that creates instances of the task type.
   */
  void registerTask(std::string name, FactoryFunction function) { factoryFunctionRegistry[name] = function; }

  /**
   * @brief Creates an instance of a task based on its name.
   *
   * @param name Name of the task type.
   * @param nh ROS NodeHandle object.
   * @param configFilename Name of the YAML configuration file.
   * @return A unique pointer to the created task instance.
   * @throws std::runtime_error if the specified task name is invalid.
   */
  std::unique_ptr<ITaskBase> createTask(std::string name, ros::NodeHandle& nh, std::string configFilename) {
    auto it = factoryFunctionRegistry.find(name);

    if (it != factoryFunctionRegistry.end()) {
      return it->second(nh, configFilename);
    } else {
      std::ostringstream errorMsg{}, oss{};

      std::vector<std::string> allowedValues = getTaskTypes();
      std::copy(allowedValues.begin(), allowedValues.end() - 1, std::ostream_iterator<std::string>(oss, ", "));

      oss << allowedValues.back();
      errorMsg << "Invalid taskType: " << name << ". Allowed values are " << oss.str() << ".";

      throw std::runtime_error(errorMsg.str());
    }
  }

  /**
   * @brief Retrieves the list of registered task types.
   *
   * @return A vector containing the names of the registered task types.
   */
  std::vector<std::string> getTaskTypes() {
    std::vector<std::string> keys;

    for (const auto& pair : factoryFunctionRegistry) {
      keys.push_back(pair.first);
    }

    return keys;
  }

private:
  std::map<std::string, FactoryFunction> factoryFunctionRegistry;
};