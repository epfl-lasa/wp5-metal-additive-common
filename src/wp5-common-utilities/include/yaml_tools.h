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
inline const void fileValidation(const std::string filePath, const std::string fileType) {
  std::ifstream file(filePath);
  if (!file.good()) {
    throw std::runtime_error("Failed to open " + fileType + " YAML file: " + filePath);
  } else {
    ROS_INFO_STREAM("Using " << fileType << " YAML file: " << filePath);
  }
}

inline const std::vector<std::string> parseYamlKey(std::string key) {
  std::string token = "";
  std::string delimiter = "/";
  std::vector<std::string> keys;

  size_t pos = 0;
  while ((pos = key.find(delimiter)) != std::string::npos) {
    token = key.substr(0, pos);

    if (!token.empty()) {
      keys.push_back(token);
    }
    key.erase(0, pos + delimiter.length());
  }

  if (!key.empty()) {
    keys.push_back(key);
  }

  return keys;
}

inline void printYamlKeys(const YAML::Node& node) {
  if (node.Type() == YAML::NodeType::Map) {
    for (const auto& it : node) {
      std::cout << it.first.as<std::string>() << std::endl;
    }
  } else {
    throw std::runtime_error("Expected a map node.");
  }
}

template <typename T>
inline const T loadYamlValue(const YAML::Node& config, const std::string& key) {
  YAML::Node node = YAML::Clone(config);
  std::vector<std::string> vectorKeys = parseYamlKey(key);

  for (const auto& key : vectorKeys) {
    node = node[key];
  }

  return node.as<T>();
}

template <typename T>
inline const T loadYamlValue(const std::string& yamlPath, const std::string& rootName, const std::string& key) {
  YAML::Node config = YAML::LoadFile(yamlPath)[rootName];
  return loadYamlValue<T>(config, key);
}

inline const std::string getYamlPath(const std::string fileName, const std::string rootPath) {
  std::string fileType = "general";
  std::string filePath = rootPath + "/../config/" + fileName;

  if (!std::filesystem::exists(filePath)) {
    fileType = "local";
    filePath = rootPath + "/config/" + fileName;
  }

  // Check if the file is valid and teturn the path
  fileValidation(filePath, fileType);
  return filePath;
}
} // namespace YamlTools