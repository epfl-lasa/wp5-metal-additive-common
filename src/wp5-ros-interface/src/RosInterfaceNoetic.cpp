/**
 * @file RosInterfaceNoetic.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief Create a ROS interface with respect to the ROS version to communicate with the robotic arm
 * @version 0.1
 * @date 2024-03-07
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "RosInterfaceNoetic.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace std;

RosInterfaceNoetic::RosInterfaceNoetic(ros::NodeHandle& n, string robotName) : nh_(n), robotName_(robotName) {
  // Try to load parameters from YAML file
  try {

      // Load parameters from YAML file
    string alternativeYamlPath = string(WP5_ROS_INTERFACE_DIR) + "/config/ros_interface_config.yaml";
    string yamlPath = string(WP5_ROS_INTERFACE_DIR) + "/../../config/ros_interface_config.yaml";

    // Check if the alternative YAML file exists
    ifstream originalFile(yamlPath);
    if (originalFile.good()) {
      cout << "Using general YAML file: " << yamlPath << endl;
    } else {
      yamlPath = alternativeYamlPath;
      cout << "Using local YAML file: " << yamlPath << endl;
    }

    // Load parameters from YAML file
    YAML::Node config = YAML::LoadFile(yamlPath);

    // Print information about robotName_ field
    YAML::Node robotNode = config[robotName_];

    // Attempt to access the "njoint" field within the robot
    nJoint_ = robotNode["number_joint"].as<int>();
    string FTTopic = robotNode["ft_topic"].as<string>();
    string actualStateTopic = robotNode["joint_topic"].as<string>();
    string commandStateTopic = robotNode["joint_command"].as<string>();

    // Initialization
    jointsPosition_.assign(nJoint_, 0.0);
    jointsSpeed_.assign(nJoint_, 0.0);
    jointsTorque_.assign(nJoint_, 0.0);
    initJoint_ = false;

    // ROS init
    subFTsensor_ = nh_.subscribe(FTTopic, 10, &RosInterfaceNoetic::FTCallback, this);
    subState_ = nh_.subscribe(actualStateTopic, 10, &RosInterfaceNoetic::jointStateCallback, this);
    pubState_ = nh_.advertise<std_msgs::Float64MultiArray>(commandStateTopic, 1000);


    pubStateDS_ = nh_.advertise<std_msgs::Float64MultiArray>("desiredDsTwist", 1000);
    pubStateCartesianTwistEEF_ = nh_.advertise<std_msgs::Float64MultiArray>("actualCartesianTwistEEF", 1000);
    pubStateCartesianPoseEEF_ = nh_.advertise<geometry_msgs::PoseStamped>("actualCartesianPoseEEF", 1000);

  } catch (const YAML::Exception& e) {
    ROS_ERROR_STREAM("Error loading YAML file: " << e.what());
  }

  // Wait for ROS master to be connected
  while (!ros::master::check()) {
    ROS_INFO("Waiting for ROS master to be connected...");
    ros::Duration(1.0).sleep(); // Sleep for 1 second before checking again
  }

  // Wait for the callback to be called at least once
  // while (!initJoint_) {
  //   ROS_INFO("Waiting for the callback to be called...");
  //   ros::Duration(1.0).sleep(); // Sleep for 1 second before checking again
  //   ros::spinOnce();            // Ensure the callback is called
  // }
}

void RosInterfaceNoetic::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (!msg->position.empty()) {
    jointsPosition_ = msg->position; // Update the position vector
    jointsSpeed_ = msg->velocity;    // Update the speed vector
    jointsTorque_ = msg->effort;     // Update the torque vector

    if (robotName_ == "ur5_robot") {
      // swap the position to have each joint in the kinematic order, ONLY FOR UR%
      swap(jointsPosition_[0], jointsPosition_[2]);
      swap(jointsSpeed_[0], jointsSpeed_[2]);
    }
    initJoint_ = true;

  } else {
    ROS_WARN("Received joint positions are empty.");
  }
}

void RosInterfaceNoetic::FTCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  if (msg->wrench.force.x != 0.0 || msg->wrench.force.y != 0.0 || msg->wrench.force.z != 0.0) {
    // Extract force and torque components from the sensor message
    double force_x = msg->wrench.force.x;
    double force_y = msg->wrench.force.y;
    double force_z = msg->wrench.force.z;

    double torque_x = msg->wrench.torque.x;
    double torque_y = msg->wrench.torque.y;
    double torque_z = msg->wrench.torque.z;

    // Combine force and torque components into a single vector
    wrenchSensor_ = {force_x, force_y, force_z, torque_x, torque_y, torque_z};
    initFTsensor_ = true;

  } else {
    ROS_WARN("Received ftsensor data is empty.");
    wrenchSensor_ = {0, 0, 0, 0, 0, 0};
  }
}

tuple<vector<double>, vector<double>, vector<double>> RosInterfaceNoetic::receiveState() {
  ros::spinOnce();

  // Create a tuple and fill it with existing vectors
  tuple<vector<double>, vector<double>, vector<double>> stateJoints =
      make_tuple(jointsPosition_, jointsSpeed_, jointsTorque_);

  return stateJoints;
}

vector<double> RosInterfaceNoetic::receiveWrench() {
  ros::spinOnce();
  return wrenchSensor_;
}

void RosInterfaceNoetic::sendState(vector<double>& data) {

  std_msgs::Float64MultiArray nextJointMsg;
  nextJointMsg.data = data;
  pubState_.publish(nextJointMsg);
}

// These functions aff for purpose to plot the speed easily

void RosInterfaceNoetic::setDesiredDsTwist(vector<double>& data) {
  std_msgs::Float64MultiArray nextTwistMsg;
  nextTwistMsg.data = data;
  pubStateDS_.publish(nextTwistMsg);
}

void RosInterfaceNoetic::setCartesianTwist(vector<double>& data) {
  std_msgs::Float64MultiArray actualTwistMsg;
  actualTwistMsg.data = data;
  pubStateCartesianTwistEEF_.publish(actualTwistMsg);
}

void RosInterfaceNoetic::setCartesianPose(pair<Eigen::Quaterniond, Eigen::Vector3d> pairActualQuatPos) {
  geometry_msgs::PoseStamped actualPoseMsg;
  // Populate the quaternion part
  actualPoseMsg.pose.orientation.x = pairActualQuatPos.first.x();
  actualPoseMsg.pose.orientation.y = pairActualQuatPos.first.y();
  actualPoseMsg.pose.orientation.z = pairActualQuatPos.first.z();
  actualPoseMsg.pose.orientation.w = pairActualQuatPos.first.w();
  
  // Populate the position part
  actualPoseMsg.pose.position.x = pairActualQuatPos.second.x();
  actualPoseMsg.pose.position.y = pairActualQuatPos.second.y();
  actualPoseMsg.pose.position.z = pairActualQuatPos.second.z();

  // Set the header information (optional)
  actualPoseMsg.header.stamp = ros::Time::now(); // Set the current time as the timestamp
  actualPoseMsg.header.frame_id = "base_link"; // Set the frame ID as needed

  // Publish the message
  pubStateCartesianPoseEEF_.publish(actualPoseMsg);
}



