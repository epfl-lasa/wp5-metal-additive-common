#pragma once

// clang-format off
#include "Tasks.h"
// clang-format on

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <string>

class ExecuteTaskAction {
public:
  explicit ExecuteTaskAction(std::string name) :
      as_(nh_, name, boost::bind(&ExecuteTaskAction::executeCB, this, _1), false), action_name_(name) {
    as_.start();
  }

  void executeCB(const wp5_tasks_action::ExecuteTaskGoalConstPtr& goal) {
    current_task_type_ = static_cast<Tasks::Type>(goal->task_id);
  }

private:
  ros::NodeHandle nh_;
  const std::string action_name_ = "";
  Tasks::Type current_task_type_ = Tasks::Type::UNDEFINED;

  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<wp5_tasks_action::ExecuteTaskAction> as_;

  // create messages that are used to published feedback/result
  wp5_tasks_action::ExecuteTaskFeedback feedback_;
  wp5_tasks_action::ExecuteTaskResult result_;
};
