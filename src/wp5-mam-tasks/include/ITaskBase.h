#pragma once

/**
 * @file
 * @brief This file contains the declaration of the ITaskBase class and its associated enums and dependencies.
 */

#include <ros/ros.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "IRoboticArmBase.h"
#include "IRosInterfaceBase.h"
#include "MAMPlanner.h"
#include "RoboticArmUr5.h"

/**
 * @brief Enum representing different types of tasks.
 */
enum TaskType : int8_t {
  TASK_UNDEFINED = -1, /**< Undefined task type. */
  WELDING,             /**< Welding task type. */
  CLEANING,            /**< Cleaning task type. */
  NB_TASKS             /**< Number of task types. Keep at the end of enum. */
};

/**
 * @brief Base class for tasks.
 */
class ITaskBase {
public:
  /**
   * @brief Map of task names to TaskType enums.
   */
  inline static const std::map<std::string, TaskType> taskTypesMap{{"welding", WELDING}, {"cleaning", CLEANING}};

  /**
   * @brief Flag indicating if initialization has been checked.
   */
  bool checkInitialization = false;

  /**
   * @brief Flag indicating if task completion has been checked.
   */
  bool checkFinish = false;

  /**
   * @brief Flag indicating if path computation has been checked.
   */
  bool checkPath = false;

  /**
   * @brief Flag indicating if homing position has been checked.
   */
  bool checkHomingPosition = false;

  /**
   * @brief Flag indicating if working position has been checked.
   */
  bool checkWorkingPosition = false;

  /**
   * @brief Constructor.
   * @param nh Node handle for ROS.
   * @param config YAML node configuration for the task.
   */
  ITaskBase(ros::NodeHandle& nh, const YAML::Node& config);

  /**
   * @brief Initializes the task.
   * @return True if initialization is successful, false otherwise.
   */
  bool initialize();

  /**
   * @brief Computes the path for the task.
   * @return True if path computation is successful, false otherwise.
   */
  virtual bool computePath() = 0;

  /**
   * @brief Executes the task.
   * @return True if execution is successful, false otherwise.
   */
  virtual bool execute() = 0;

  /**
   * @brief Moves to the homing position.
   * @return True if successful, false otherwise.
   */
  virtual bool goHomingPosition();

  /**
   * @brief Moves to the working position.
   * @return True if successful, false otherwise.
   */
  virtual bool goWorkingPosition();

protected:
  std::unique_ptr<MAMPlanner> planner_ = nullptr; ///< Pointer to MAMPlanner instance.

  /**
   * @brief Get the ROS node handle.
   * @return ROS node handle.
   */
  ros::NodeHandle getRosNodehandle_() const { return nh_; }

  /**
   * @brief Get the ROS version of the task.
   * @return ROS version.
   */
  const ROSVersion getRosVersion_() const { return rosVersion_; }

  /**
   * @brief Get the home joint configuration.
   * @return Home joint configuration.
   */
  std::vector<double> getHomeJoint_() const { return homeJoint_; }

  /**
   * @brief Set the home joint configuration.
   * @param desiredJoint Desired joint configuration.
   */
  void setHomeJoint_(std::vector<double> desiredJoint) { homeJoint_ = desiredJoint; }

private:
  ros::NodeHandle nh_{};                      ///< ROS node handle.
  const std::string robotName_{};             ///< Robot name.
  const ROSVersion rosVersion_{};             ///< ROS version.
  const std::vector<double> eePosOffset_{};   ///< End effector position offset.
  const std::vector<double> eeAngleOffset_{}; ///< End effector angle offset.

  std::vector<double> homeJoint_{}; ///< Home joint configuration.
};
