/**
 * @file test_subtask.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-09
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "Subtask.h"

using namespace std;

static Subtask* subtask = nullptr;

class SubtaskTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    ros::NodeHandle nh;
    subtask = new Subtask(nh);
  }

  static void TearDownTestSuite() { delete subtask; }
};

// Create a function to test the splitString_ function of the Subtask class
// Test the exact splitting string but also test the case where the string is not correctly formatted
TEST_F(SubtaskTest, TestSplitString) {
  string str = "waypoint1,1,2,3,4,5,6";
  string id;
  vector<double> pos;
  subtask->splitString_(str, ',', id, pos);

  EXPECT_EQ(id, "waypoint1");
  EXPECT_EQ(pos.size(), 6);
  EXPECT_EQ(pos[0], 1);
  EXPECT_EQ(pos[1], 2);
  EXPECT_EQ(pos[2], 3);
  EXPECT_EQ(pos[3], 4);
  EXPECT_EQ(pos[4], 5);
  EXPECT_EQ(pos[5], 6);

  // TODO(lmunier): Fix the test
  //   str = "waypoint2,1,2,,4,5";
  //   subtask->splitString_(str, ',', id, pos);

  //   EXPECT_EQ(id, "waypoint2");
  //   EXPECT_EQ(pos.size(), 4);
  //   EXPECT_EQ(pos[0], 1);
  //   EXPECT_EQ(pos[1], 2);
  //   EXPECT_EQ(pos[2], 4);
  //   EXPECT_EQ(pos[3], 5);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_subtask");
  ::testing::InitGoogleTest(&argc, argv);

  // Set the filter to run only the specific test
  // ::testing::GTEST_FLAG(filter) = "RoboticArmUr5Test.TestReferenceConfiguration";

  return RUN_ALL_TESTS();
}
