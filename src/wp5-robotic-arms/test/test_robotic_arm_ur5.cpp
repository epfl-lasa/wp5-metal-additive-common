/**
 * @file RoboticArmUr5.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-09-05
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
  const double TOLERANCE = 1e-6;

  RoboticArmUr5* robotic_arm;
  vector<double> seed = {};
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); // w, x, y, z
  Eigen::Vector3d position = Eigen::Vector3d(0.3, 0.209, 0.6);

  void SetUp() override {
    ros::NodeHandle nh;
    robotic_arm = new RoboticArmUr5();

    for (int i = 0; i < robotic_arm->getNJoint(); i++) {
      seed.push_back(0.0);
    }
  }

  void TearDown() override { delete robotic_arm; }
};

// Create a test to check the forward kinematics of the UR5 robotic arm
TEST_F(RoboticArmUr5Test, TestTracIkSolver) {
  auto ik_result = robotic_arm->getIK(IkSolver::TRAC_IK_SOLVER, quaternion, position);

  // Use get_if to safely extract the type
  auto ik_solutions = get_if<vector<double>>(&ik_result);
  ASSERT_NE(ik_solutions, nullptr) << "Failed to extract vector<double> from variant";

  // Compute forward kinematics
  vector<double> fk_result = robotic_arm->getFK(*ik_solutions);

  // Check the position
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(fk_result[i], position[i], TOLERANCE);
  }

  // Check the orientation
  for (int i = 3; i < 7; i++) {
    EXPECT_NEAR(fk_result[i], quaternion.coeffs()[i - 3], TOLERANCE);
  }
}

TEST_F(RoboticArmUr5Test, TestIkGeoSolver) {
  //   cout << "Number of solutions: " << solutions.size() << endl;

  //   for (auto& solution : solutions) {
  //     cout << "Solution: ";
  //     for (size_t i = 0; i < 6; i++) {
  //       cout << solution.q[i] << " ";
  //     }

  //     array<double, 9> rotation_matrix;
  //     array<double, 3> position_vector;
  //     ikGeoSolver_->fk(solution.q, rotation_matrix, position_vector);

  //     cout << "Is LS: " << (solution.is_ls ? "True" : "False") << endl;
  //     cout << "Rotation Matrix: " << endl;
  //     for (size_t i = 0; i < 3; i++) {
  //       for (size_t j = 0; j < 3; j++) cout << rotation_matrix[i * 3 + j] << " ";
  //       cout << endl;
  //     }
  //     cout << "Position Vector: " << endl;
  //     for (size_t i = 0; i < 3; i++) {
  //       cout << position_vector[i] << " ";
  //     }
  //     cout << endl;
  //   }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_robotic_arm_ur5");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}