/**
 * @file IRoboticArmBase.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-07
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#include "IRoboticArmBase.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

using namespace std;

IRoboticArmBase::IRoboticArmBase(string robotName) : robotName_(robotName) {
  string yamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/../../config/arm_robot_config.yaml";

  ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    cout << "Using general YAML file: " << yamlPath << endl;
  } else {
    yamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/config/arm_robot_config.yaml";
    cout << "Using local YAML file: " << yamlPath << endl;
  }

  // Load parameters from YAML file
  YAML::Node robotConfig = YAML::LoadFile(yamlPath)[robotName_];

  pathUrdf_ = robotConfig["path_urdf"].as<string>();
  jointNames_ = robotConfig["joint_names"].as<vector<string>>();
  originalHomeJoint_ = robotConfig["original_home_joint"].as<vector<double>>();
  referenceFrame_ = robotConfig["reference_frame"].as<string>();
  tipLink_ = robotConfig["tip_link"].as<string>();

  nJoint_ = jointNames_.size();
}

void IRoboticArmBase::printInfo() {
  cout << "Robot name: " << robotName_ << endl;
  cout << "URDF Path: " << pathUrdf_ << endl;
  cout << "Number of joints: " << nJoint_ << endl;

  cout << "Joint names: ";
  for (const string& jointName : jointNames_) {
    cout << jointName << " ";
  }
  cout << endl;

  cout << "Tip link: " << tipLink_ << endl;
  cout << "Reference frame: " << referenceFrame_ << endl;

  cout << "Original home joint: ";
  for (double joint : originalHomeJoint_) {
    cout << joint << " ";
  }
  cout << endl;
}

// function for the inverse kinematic  ------------------------------------------------------------------------

// pair<int, vector<double>> IRoboticArmBase::getIK(vector<double> actualJoint, vector<double> vectorQuatPos ) {

//   //Inverse kinematics trac-IK
//   KDL::JntArray NextJointTask;
//   KDL::JntArray actualJointTask;

//   VectorXd pos_joint_actual_eigen(nJoint_);
//   for(int i = 0 ;i<nJoint;++i){
//       pos_joint_actual_eigen(i) =actualJoint[i];
//   }
//   actualJointTask.data = pos_joint_actual_eigen;

//   KDL::Vector Vec(vectorQuatPos[4],vectorQuatPos[5],vectorQuatPos[6]);

//   Quaterniond q(vectorQuatPos[3],vectorQuatPos[0],vectorQuatPos[1],vectorQuatPos[2]);
//   q.normalize();
//   KDL::Rotation Rot = KDL::Rotation::Quaternion(q.x(),q.y(),q.z(),q.w());
//   KDL::Frame NextJointCartesian(Rot,Vec);
//   rc = ikSolver->CartToJnt(actualJointTask, NextJointCartesian, NextJointTask);
//   if (rc< 0){
//       cout<<"no inverse kinematic found"<<endl;
//   }

//   VectorXd posJointNextEigen = NextJointTask.data;
//   vector<double> posJointNext ;
//   for(int i = 0 ;i<nJoint;++i){
//       posJointNext[i] =posJointNextEigen(i);
//       }
//   //msgP.data = posJointNext;

//   //actualJointTask.data.clear();
//   pair<int, vector<double>> myPair = make_pair(rc, posJointNext);

//   return myPair;
// }

// void IRoboticArmBase::updateIK(double err ,double timeoutInSecs, string solveTypeStr ){
//   // Convert the solve type string to the corresponding enum value
//   TRAC_IK::SolveType solveType;
//   if (solveTypeStr == "Distance") {
//       solveType = TRAC_IK::Distance;
//   } else if (solveTypeStr == "Speed") {
//       solveType = TRAC_IK::Speed;
//   } else {
//       cout<< "Handle unrecognized solve types: set Distance as default value" << endl;
//       solveType = TRAC_IK::Distance;
//   }
//   ikSolver= new TRAC_IK::TRAC_IK(baseLink_, tipLink_,paramURDF_, timeoutInSecs, err, solveType);
// }

// void IRoboticArmBase::initIK(){
//   YAML::Node config = YAML::LoadFile("/../config/config.yaml");
//   // Get the solve type from the YAML file
//   string solveTypeStr = config["IK/solve_type"].as<string>();

//   // Convert the solve type string to the corresponding enum value
//   TRAC_IK::SolveType solveType;
//   if (solveTypeStr == "Distance") {
//       solveType = TRAC_IK::Distance;
//   } else if (solveTypeStr == "Speed") {
//       solveType = TRAC_IK::Speed;
//   } else {
//       cout<< "Handle unrecognized solve types: set Distance as default value" << endl;
//       solveType = TRAC_IK::Distance;
//   }
//   error = config["IK/error"].as<double>();
//   timeoutInSecs = config["IK/timeoutInSecs"].as<double>();

//   ikSolver= new TRAC_IK::TRAC_IK(baseLink_, tipLink_,paramURDF_, timeoutInSecs, error, type);

//   valid = ikSolver->getKDLChain(chain);
//   if (!valid) {
//       cout << "There was no valid KDL chain found"<< endl;
//   }
// }
