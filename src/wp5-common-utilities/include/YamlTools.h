/**
 * @file yaml_tools.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace YamlTools {
template <typename T>
const T loadYamlValue_(const std::string yamlPath, const std::string rootName, const std::string key) {
  YAML::Node config = YAML::LoadFile(yamlPath)[rootName];
  return config[key].as<T>();
}

const std::string getYamlPath_(const std::string fileName, const std::string rootPath) {
  std::string yamlFile = "general";
  std::string yamlPath = rootPath + "/../config/" + fileName;

  if (!std::filesystem::exists(yamlPath)) {
    yamlFile = "local";
    yamlPath = rootPath + "/config/" + fileName;
  }

  // Check if the file is valid
  std::ifstream file(yamlPath);
  if (!file.good()) {
    throw std::runtime_error("Failed to open " + yamlFile + " YAML file: " + yamlPath);
  } else {
    ROS_INFO_STREAM("Using " << yamlFile << " YAML file: " << yamlPath);
  }

  return yamlPath;
}

const void yamlFileValidation_(const std::string yamlPath, const std::string yamlFile) {
  std::ifstream file(yamlPath);
  if (!file.good()) {
    throw std::runtime_error("Failed to open " + yamlFile + " YAML file: " + yamlPath);
  } else {
    ROS_INFO_STREAM("Using " << yamlFile << " YAML file: " << yamlPath);
  }
}
} // namespace YamlTools