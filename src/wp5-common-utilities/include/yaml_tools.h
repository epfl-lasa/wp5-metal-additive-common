/**
 * @file yaml_tools.h
 * @brief A collection of YAML utility functions to centralize common operations.
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.2
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */

#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <string>

/**
 * @namespace YamlTools
 * @brief A collection of yaml utility functions.
 *
 * This namespace contains functions for various yaml file checks and operations.
 */
namespace YamlTools {
/**
 * @brief Validates the specified file based on its type.
 *
 * This function checks if the file at the given file path is valid according to the specified file type.
 *
 * @param filePath The path to the file that needs to be validated.
 * @param fileType The type of the file to validate against.
 */
const void fileValidation(const std::string filePath, const std::string fileType);

/**
 * @brief Parses a YAML key into its constituent parts.
 *
 * This function takes a YAML key as input and splits it into its individual components.
 * The key is expected to be in a dot-separated format (e.g., "parent.child.subchild").
 *
 * @param key The YAML key to be parsed.
 * @return A vector of strings, each representing a part of the key.
 */
const std::vector<std::string> parseYamlKey(std::string key);

/**
 * @brief Prints the keys of a given YAML node.
 *
 * This function iterates through the provided YAML node and prints out all the keys
 * it contains. It is useful for debugging and inspecting the structure of a YAML file.
 *
 * @param node The YAML node whose keys are to be printed.
 */
void printYamlKeys(const YAML::Node& node);

/**
 * @brief Loads a value from a YAML configuration node using a specified key.
 *
 * This function takes a YAML configuration node and a key, which can be a
 * hierarchical key separated by dots (e.g., "parent.child.key"). It traverses
 * the YAML node according to the key and returns the value as the specified type.
 *
 * @tparam T The type to which the YAML value should be converted.
 * @param config The YAML configuration node from which to load the value.
 * @param key The key used to locate the value within the YAML node. It can be
 *            a hierarchical key separated by dots.
 * @return The value from the YAML node converted to the specified type.
 */
template <typename T>
const T loadYamlValue(const YAML::Node& config, const std::string& key) {
  YAML::Node node = YAML::Clone(config);
  std::vector<std::string> vectorKeys = parseYamlKey(key);

  for (const auto& key : vectorKeys) {
    node = node[key];
  }

  return node.as<T>();
}

/**
 * @brief Loads a value of type T from a YAML file.
 *
 * This function reads a YAML file from the specified path, navigates to the specified root node,
 * and retrieves the value associated with the given key.
 *
 * @tparam T The type of the value to be loaded.
 * @param yamlPath The path to the YAML file.
 * @param rootName The name of the root node in the YAML file.
 * @param key The key within the root node whose value is to be retrieved.
 * @return The value of type T associated with the specified key.
 */
template <typename T>
const T loadYamlValue(const std::string& yamlPath, const std::string& rootName, const std::string& key) {
  YAML::Node config = YAML::LoadFile(yamlPath)[rootName];
  return loadYamlValue<T>(config, key);
}

/**
 * @brief Retrieves the full path to a YAML file.
 *
 * This function constructs the full path to a YAML file by combining the provided
 * file name with the specified root path.
 *
 * @param fileName The name of the YAML file.
 * @param rootPath The root directory path where the YAML file is located.
 * @return The full path to the YAML file as a std::string.
 */
const std::string getYamlPath(const std::string fileName, const std::string rootPath);
} // namespace YamlTools