/**
 * @file RoboticArmUr5.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <random>
#include <variant>

#include "RoboticArmUr5.h"

using namespace std;

class RoboticArmUr5Test : public ::testing::Test {
protected:
  const double TOLERANCE = 1e-4;
  static const int NB_TESTS = 5;

  static RoboticArmUr5* roboticArm;
  static mt19937 gen;
  static uniform_real_distribution<> dis;
  static vector<pair<Eigen::Quaterniond, Eigen::Vector3d>> waypoints;

  RoboticArmUr5Test() {}

  static void SetUpTestSuite() {
    ros::NodeHandle nh;
    roboticArm = new RoboticArmUr5(ROSVersion::ROS1_NOETIC);
    generateWaypoints();
  }

  static void TearDownTestSuite() { delete roboticArm; }

  // Function to generate a random quaternion
  static Eigen::Quaterniond generateRandomQuaternion() {
    Eigen::Quaterniond q(dis(gen), dis(gen), dis(gen), dis(gen));
    q.normalize();
    return q;
  }

  // Function to generate a random position vector
  static Eigen::Vector3d generateRandomPosition() { return Eigen::Vector3d(dis(gen), dis(gen), dis(gen)); }

  // Function to generate a random vector of size n
  static vector<double> generateRandomVector(int n) {
    vector<double> vec(n);
    for (int i = 0; i < n; ++i) {
      vec[i] = dis(gen);
    }
    return vec;
  }

  // Function to generate a reachable random waypoint
  static pair<Eigen::Quaterniond, Eigen::Vector3d> generateReachableWaypoint() {
    while (true) {
      Eigen::Quaterniond quaternion = generateRandomQuaternion();
      Eigen::Vector3d position = generateRandomPosition();
      vector<double> jointPos{};

      bool isValid = roboticArm->getIK(quaternion, position, jointPos);

      // Check if the IK solver found a valid solution
      if (isValid) {
        return make_pair(quaternion, position);
      }
    }
  }

  // Function to generate waypoints
  static void generateWaypoints() {
    for (int i = 0; i < NB_TESTS; ++i) {
      waypoints.push_back(generateReachableWaypoint());
    }
  }

  // Function to check if two quaternions represent the same orientation
  static void areQuaternionsEquivalent(const Eigen::Quaterniond& q1,
                                       const Eigen::Quaterniond& q2,
                                       double tolerance = 1e-5) {
    Eigen::Matrix3d rot1 = q1.toRotationMatrix();
    Eigen::Matrix3d rot2 = q2.toRotationMatrix();

    EXPECT_LT((rot1 - rot2).norm(), tolerance);
  }

  // Function to check if two positions are equivalent
  static void arePositionsEquivalent(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double tolerance = 1e-5) {
    EXPECT_LT((p1 - p2).norm(), tolerance);
  }
};

// Static member initialization
RoboticArmUr5* RoboticArmUr5Test::roboticArm = nullptr;
mt19937 RoboticArmUr5Test::gen(random_device{}());
uniform_real_distribution<> RoboticArmUr5Test::dis(-0.5, 0.5);
vector<pair<Eigen::Quaterniond, Eigen::Vector3d>> RoboticArmUr5Test::waypoints;

// Create a test to check the forward kinematics of the UR5 robotic arm
TEST_F(RoboticArmUr5Test, TestTracIkSolver) {
  for (auto& [quaternion, position] : waypoints) {
    vector<double> jointPos{};
    roboticArm->getIK(quaternion, position, jointPos);

    // Compute forward kinematics
    pair<Eigen::Quaterniond, Eigen::Vector3d> fkResult = roboticArm->getFK(jointPos);

    areQuaternionsEquivalent(fkResult.first, quaternion, TOLERANCE);
    arePositionsEquivalent(fkResult.second, position, TOLERANCE);
  }
}

TEST_F(RoboticArmUr5Test, TestIkGeoSolver) {
  int iter = 0;
  for (auto& [quaternion, position] : waypoints) {
    iter++;
    vector<vector<double>> ikSolutions;
    roboticArm->getIKGeo(quaternion, position, ikSolutions);

    // Compute forward kinematics
    for (const auto& sol : ikSolutions) {
      pair<Eigen::Quaterniond, Eigen::Vector3d> fkResult = roboticArm->getFKGeo(sol);

      areQuaternionsEquivalent(fkResult.first, quaternion, TOLERANCE);
      arePositionsEquivalent(fkResult.second, position, TOLERANCE);
    }
  }
}

TEST_F(RoboticArmUr5Test, TestSwapJoints) {
  // Generate fake input data to check swapJoints_ function
  int nbJoints = roboticArm->getNbJoints();

  vector<double> jointPosIn = generateRandomVector(nbJoints);
  vector<double> jointVelIn = generateRandomVector(nbJoints);
  vector<double> jointTorqueIn = generateRandomVector(nbJoints);
  tuple<vector<double>, vector<double>, vector<double>> state{jointPosIn, jointVelIn, jointTorqueIn};

  // Generate fake output data to check swapJoints_ function
  vector<double> jointPosOut = jointPosIn;
  vector<double> jointVelOut = jointVelIn;
  vector<double> jointTorqueOut = jointTorqueIn;

  // Swap the data 0 to 2 for both fake input and output data using different methods
  swap(jointPosOut[0], jointPosOut[2]);
  swap(jointVelOut[0], jointVelOut[2]);
  swap(jointTorqueOut[0], jointTorqueOut[2]);

  roboticArm->swapJoints_(state);

  // Check if the joint positions are valid
  EXPECT_EQ(jointPosOut, get<0>(state));
  EXPECT_EQ(jointVelOut, get<1>(state));
  EXPECT_EQ(jointTorqueOut, get<2>(state));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_robotic_arm_ur5");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}