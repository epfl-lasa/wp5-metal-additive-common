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

#include "RoboticArmUr5.h"

const double TOLERANCE = 1e-6;

class RoboticArmUr5Test : public ::testing::Test {
protected:
  RoboticArmUr5* robotic_arm;
  std::vector<double> seed = {};
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d position = Eigen::Vector3d(0.0, 0.0, 0.0);

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

  // If you know the type is std::vector<std::vector<double>>
  auto solutions = std::get<std::vector<double>>(ik_result);

  // Check the position
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(solutions[i], position[i], TOLERANCE);
  }

  // Check the orientation
  for (int i = 3; i < 7; i++) {
    EXPECT_NEAR(solutions[i], quaternion.coeffs()[i - 3], TOLERANCE);
  }
}

TEST_F(RoboticArmUr5Test, TestIkGeoSolver) {
  //   std::cout << "Number of solutions: " << solutions.size() << std::endl;

  //   for (auto& solution : solutions) {
  //     std::cout << "Solution: ";
  //     for (size_t i = 0; i < 6; i++) {
  //       std::cout << solution.q[i] << " ";
  //     }

  //     array<double, 9> rotation_matrix;
  //     array<double, 3> position_vector;
  //     ikGeoSolver_->fk(solution.q, rotation_matrix, position_vector);

  //     std::cout << "Is LS: " << (solution.is_ls ? "True" : "False") << std::endl;
  //     std::cout << "Rotation Matrix: " << std::endl;
  //     for (size_t i = 0; i < 3; i++) {
  //       for (size_t j = 0; j < 3; j++) std::cout << rotation_matrix[i * 3 + j] << " ";
  //       std::cout << std::endl;
  //     }
  //     std::cout << "Position Vector: " << std::endl;
  //     for (size_t i = 0; i < 3; i++) {
  //       std::cout << position_vector[i] << " ";
  //     }
  //     std::cout << std::endl;
  //   }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_robotic_arm_ur5");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}