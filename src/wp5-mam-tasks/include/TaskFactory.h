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
  using FactoryFunction = std::function<std::unique_ptr<ITaskBase>(
      ros::NodeHandle& nh, ROSVersion rosVersion, double freq, std::string robotName)>;

  /**
   * @brief Constructs a TaskFactory object.
   *
   * The constructor initializes the factoryFunctionRegistry and registers the default task types.
   */
  TaskFactory() {
    registerTask("welding", [](ros::NodeHandle& nh, ROSVersion rosVersion, double freq, std::string robotName) {
      return std::make_unique<TaskWelding>(nh, rosVersion, freq, robotName);
    });
    registerTask("cleaning", [](ros::NodeHandle& nh, ROSVersion rosVersion, double freq, std::string robotName) {
      return std::make_unique<TaskCleaning>(nh, rosVersion, freq, robotName);
    });
  }

  /**
   * @brief Registers a task type with its corresponding factory function.
   *
   * @param name The name of the task type.
   * @param function The factory function that creates instances of the task type.
   */
  void registerTask(std::string name, FactoryFunction function) { factoryFunctionRegistry[name] = function; }

  /**
   * @brief Creates an instance of a task based on its name.
   *
   * @param name The name of the task type.
   * @param nh The ROS NodeHandle object.
   * @param rosVersion The ROS version.
   * @param freq The frequency of the task.
   * @param robotName The name of the robot associated with the task.
   * @return A unique pointer to the created task instance.
   * @throws std::runtime_error if the specified task name is invalid.
   */
  std::unique_ptr<ITaskBase> createTask(
      std::string name, ros::NodeHandle& nh, ROSVersion rosVersion, double freq, std::string robotName) {
    auto it = factoryFunctionRegistry.find(name);

    if (it != factoryFunctionRegistry.end()) {
      return it->second(nh, rosVersion, freq, robotName);
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

    return move(keys);
  }

private:
  std::map<std::string, FactoryFunction> factoryFunctionRegistry;
};