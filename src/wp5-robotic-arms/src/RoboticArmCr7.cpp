/**
 * @file RoboticArmCr7.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "RoboticArmCr7.h"

#include <iostream>

using namespace std;

RoboticArmCr7::RoboticArmCr7() : IRoboticArmBase(string("xMateCR7")) {}

pair<Eigen::Quaterniond, Eigen::Vector3d> RoboticArmCr7::getFK(IkSolver ikSolver, vector<double> jointPositions) {
  //TODO(lmunier): Implement the forward kinematics of the xMateCR7 robotic arm
  cout << "Forward kinematics of the UR5 robotic arm" << endl;
}

variant<vector<double>, vector<vector<double>>> RoboticArmCr7::getIK(IkSolver ikSolver,
                                                                     Eigen::Quaterniond quaternion,
                                                                     Eigen::Vector3d position) {
  //TODO(lmunier): Implement the inverse kinematics of the xMateCR7 robotic arm
  cout << "Inverse kinematics of the UR5 robotic arm" << endl;
}
