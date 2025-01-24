/**
 * @file TaskFactory.h
 * @brief Factory class to create instances of task.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.3
 * @date 2025-01-21
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
 *
 */
#pragma once

#include <ros/ros.h>

#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ITaskBase.h"
#include "TaskCleaning.h"
#include "TaskWelding.h"
#include "debug_tools.h"

/**
 * @brief The TaskFactory class is responsible for creating instances of tasks based on their names.
 *
 * The TaskFactory class provides a mechanism to register different task types and their corresponding factory functions.
 * It allows creating instances of tasks by specifying their names and providing the necessary parameters.
 */
class TaskFactory {
public:
  // Prevent instantiation
  TaskFactory() = delete;

  /**
   * @brief Registers a task type with its corresponding factory function.
   *
   * @param name Name of the task type.
   * @param function Factory function that creates instances of the task type.
   */
  static void registerTask(const std::string& name,
                           std::function<std::shared_ptr<ITaskBase>(ros::NodeHandle&, std::string)> creator) {
    getRegistry()[name] = creator;
  }

  /**
   * @brief Creates an instance of a task based on its name.
   *
   * @param name Name of the task type.
   * @param nh ROS NodeHandle object.
   * @param configFilename Name of the YAML configuration file.
   * @return A unique pointer to the created task instance.
   * @throws std::runtime_error if the specified task name is invalid.
   */
  static std::shared_ptr<ITaskBase> createTask(const std::string& name,
                                               ros::NodeHandle& nh,
                                               std::string configFilename) {
    auto it = getRegistry().find(name);

    if (it != getRegistry().end()) {
      return it->second(nh, configFilename);
    } else {
      std::ostringstream errorMsg{};
      errorMsg << "Invalid task type: " << name << ". Allowed values are ";
      errorMsg << DebugTools::getVecString<std::string>(getTaskTypes());

      throw std::runtime_error(errorMsg.str());
    }
  }

  /**
   * @brief Retrieves the list of registered task types.
   *
   * @return A vector containing the names of the registered task types.
   */
  static std::vector<std::string> getTaskTypes() {
    std::vector<std::string> keys;

    for (const auto& pair : getRegistry()) {
      keys.push_back(pair.first);
    }

    return keys;
  }

private:
  // Get the registry of robotic arms
  static std::unordered_map<std::string, std::function<std::shared_ptr<ITaskBase>(ros::NodeHandle&, std::string)>>&
  getRegistry() {
    static std::unordered_map<std::string, std::function<std::shared_ptr<ITaskBase>(ros::NodeHandle&, std::string)>>
        registry;
    return registry;
  }

  // Static initialization block
  static bool initialize() {
    registerTask("welding", [](ros::NodeHandle& nh, std::string configFilename) {
      return std::make_shared<TaskWelding>(nh, configFilename);
    });
    registerTask("cleaning", [](ros::NodeHandle& nh, std::string configFilename) {
      return std::make_shared<TaskCleaning>(nh, configFilename);
    });
    return true;
  }

  // Static member to trigger initialization only once
  static inline bool initialized = initialize();
};