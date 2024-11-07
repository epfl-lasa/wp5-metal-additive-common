/**
 * @file yaml_tools.cpp
 * @author [Louis Munier] - lmunier@protonmail.com
 * @brief
 * @version 0.2
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#include "yaml_tools.h"

namespace YamlTools {

const void fileValidation(const std::string filePath, const std::string fileType) {
  std::ifstream file(filePath);
  if (!file.good()) {
    throw std::runtime_error("[YamlTools] - Failed to open " + fileType + " YAML file: " + filePath);
  } else {
    ROS_INFO_STREAM("[YamlTools] - Using " << fileType << " YAML file: " << filePath);
  }
}

const std::vector<std::string> parseYamlKey(std::string key) {
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

void printYamlKeys(const YAML::Node& node) {
  if (node.Type() == YAML::NodeType::Map) {
    for (const auto& it : node) {
      std::cout << it.first.as<std::string>() << std::endl;
    }
  } else {
    throw std::runtime_error("[YamlTools] - Expected a map node.");
  }
}

const std::string getYamlPath(const std::string fileName, const std::string rootPath) {
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