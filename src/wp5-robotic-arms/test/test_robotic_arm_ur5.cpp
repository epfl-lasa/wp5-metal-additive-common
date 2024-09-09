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

#include <variant>

#include "RoboticArmUr5.h"

using namespace std;

class RoboticArmUr5Test : public ::testing::Test {
protected:
  const double TOLERANCE = 1e-5;

  RoboticArmUr5* roboticArm;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); // w, x, y, z
  Eigen::Vector3d position = Eigen::Vector3d(0.3, 0.209, 0.6);

  void SetUp() override {
    ros::NodeHandle nh;
    roboticArm = new RoboticArmUr5();
  }

  void TearDown() override { delete roboticArm; }
};

// Create a test to check the forward kinematics of the UR5 robotic arm
TEST_F(RoboticArmUr5Test, TestTracIkSolver) {
  auto ikResult = roboticArm->getIK(IkSolver::TRAC_IK_SOLVER, quaternion, position);

  // Use get_if to safely extract the type
  auto ikSolutions = get_if<vector<double>>(&ikResult);
  ASSERT_NE(ikSolutions, nullptr) << "Failed to extract vector<double> from variant";

  // Compute forward kinematics
  pair<Eigen::Quaterniond, Eigen::Vector3d> fkResult = roboticArm->getFK(IkSolver::TRAC_IK_SOLVER, *ikSolutions);

  // Check the position
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(fkResult.second[i], position[i], TOLERANCE);
  }

  // Check the orientation
  Eigen::Vector4d quaternionCoeffs = fkResult.first.coeffs();
  for (int i = 0; i < 4; i++) {
    EXPECT_NEAR(quaternionCoeffs[i], quaternion.coeffs()[i], TOLERANCE);
  }
}

TEST_F(RoboticArmUr5Test, TestIkGeoSolver) {
  auto ikResult = roboticArm->getIK(IkSolver::IK_GEO_SOLVER, quaternion, position);

  // Use get_if to safely extract the type
  auto ikSolutions = get_if<vector<vector<double>>>(&ikResult);
  ASSERT_NE(ikSolutions, nullptr) << "Failed to extract vector<vector<double>> from variant";

  // Compute forward kinematics
  for (const auto& sol : *ikSolutions) {
    pair<Eigen::Quaterniond, Eigen::Vector3d> fkResult = roboticArm->getFK(IkSolver::IK_GEO_SOLVER, sol);

    // Check the position
    for (int i = 0; i < 3; i++) {
      EXPECT_NEAR(fkResult.second[i], position[i], TOLERANCE);
    }

    // Check the orientation
    Eigen::Vector4d quaternionCoeffs = fkResult.first.coeffs();
    for (int i = 0; i < 4; i++) {
      EXPECT_NEAR(quaternionCoeffs[i], quaternion.coeffs()[i], TOLERANCE);
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_robotic_arm_ur5");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}