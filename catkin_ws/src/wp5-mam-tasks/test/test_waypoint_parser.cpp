/**
 * @file test_waypoint_parser.cpp
 * @brief Test the WaypointParser class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.1
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024 - EPFL - LASA. All rights reserved.
 *
 */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <string>
#include <vector>

#include "WaypointParser.h"

using namespace std;

class WaypointParserTest : public ::testing::Test {
protected:
  static WaypointParser* waypointParser_;

  static void SetUpTestSuite() { waypointParser_ = new WaypointParser(); }

  static void TearDownTestSuite() { delete waypointParser_; }
};

WaypointParser* WaypointParserTest::waypointParser_ = nullptr;

TEST_F(WaypointParserTest, TestWellPackedWaypoint) {
  string id = "";
  vector<double> pos;

  // Test unpack a well formatted waypoint
  string str = "waypoint1,1,2,3,4,5,6";
  vector<double> waypointUnpacked = {1, 2, 3, 4, 5, 6};
  waypointParser_->unpackWaypoint(str, ',', id, pos);

  EXPECT_EQ(id, "waypoint1");
  EXPECT_EQ(pos.size(), waypointUnpacked.size());
  EXPECT_EQ(pos, waypointUnpacked);
}

TEST_F(WaypointParserTest, TestMissingValueWaypoint) {
  string id = "";
  vector<double> pos;

  // Test unpack a waypoint with a missing value
  string str = "waypoint2,1,2,,4,5";
  vector<double> waypointUnpacked = {1, 2, 4, 5};
  waypointParser_->unpackWaypoint(str, ',', id, pos);

  EXPECT_EQ(id, "waypoint2");
  EXPECT_EQ(pos.size(), waypointUnpacked.size());
  EXPECT_EQ(pos, waypointUnpacked);
}

TEST_F(WaypointParserTest, TestMissingIDWaypoint) {
  string id = "";
  vector<double> pos;

  // Test unpack a waypoint with a missing id
  string str = "2,1,2,,4,5";
  vector<double> waypointUnpacked = {};
  waypointParser_->unpackWaypoint(str, ',', id, pos);

  EXPECT_EQ(id, "");
  EXPECT_EQ(pos.size(), waypointUnpacked.size());
  EXPECT_EQ(pos, waypointUnpacked);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_subtask");
  ::testing::InitGoogleTest(&argc, argv);

  // Set the filter to run only the specific test
  // ::testing::GTEST_FLAG(filter) = "RoboticArmUrTest.TestReferenceConfiguration";

  return RUN_ALL_TESTS();
}
