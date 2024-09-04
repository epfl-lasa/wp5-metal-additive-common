/**
 * @file RoboticArmUr5.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "RoboticArmUr5.h"

#include <ros/duration.h>

using namespace std;

RoboticArmUr5::RoboticArmUr5() : IRoboticArmBase(string("ur5_robot")) {
  trackIkSolver_ = make_unique<TRAC_IK::TRAC_IK>("base_link_inertia", "virual_target", "Distance", ros::Duration(0.01));
  ikGeoSolver_ = make_unique<ik_geo::Robot>(ik_geo::Robot::ur5());
}

vector<double> RoboticArmUr5::getFK(vector<double> jointPositions) {
  //TODO(lmunier): Implement the forward kinematics of the UR5 robotic arm
  cout << "Forward kinematics of the UR5 robotic arm" << endl;
}

vector<double> RoboticArmUr5::getIK(IkType ikType) {
  //TODO(lmunier): Implement the inverse kinematics of the UR5 robotic arm
  cout << "Inverse kinematics of the UR5 robotic arm" << endl;
}