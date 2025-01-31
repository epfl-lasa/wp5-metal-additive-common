/**
 * @file test_waypoint_parser.cpp
 * @brief Test the WaypointParser class
 *
 * @author [Louis Munier] - lmunier@protonmail.com
 * @version 0.1
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2025 - EPFL - LASA. All rights reserved.
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

  void testUnpackWaypoint(const string& str,
                          const string& expectedId,
                          const string& expectedRefFrame,
                          const vector<double>& expectedPos,
                          const vector<double>& expectedNormal) {
    string id = "";
    string refFrame = "";
    vector<double> pos;
    vector<double> normal;

    bool result = waypointParser_->unpackWaypoint(str, id, refFrame, pos, normal);

    EXPECT_EQ(result, !expectedId.empty() && !expectedRefFrame.empty() && !expectedPos.empty());
    EXPECT_EQ(id, expectedId);
    EXPECT_EQ(refFrame, expectedRefFrame);
    EXPECT_EQ(pos.size(), expectedPos.size());
    EXPECT_EQ(pos, expectedPos);
    EXPECT_EQ(normal.size(), expectedNormal.size());
    EXPECT_EQ(normal, expectedNormal);
  }
};

WaypointParser* WaypointParserTest::waypointParser_ = nullptr;

// Test to unpack a well packed damage string
TEST_F(WaypointParserTest, TestWellPackedWaypoint) {
  string str = "waypoint1,base_link,1,2,3,4,5,6,0.1,0.2,0.3";
  vector<double> waypointUnpacked = {1, 2, 3, 4, 5, 6};
  vector<double> normalUnpacked = {0.1, 0.2, 0.3};
  testUnpackWaypoint(str, "waypoint1", "base_link", waypointUnpacked, normalUnpacked);
}

// Test to unpack a damage string with a string at a waypoint position
TEST_F(WaypointParserTest, TestNotWellPackedWaypoint) {
  string str = "waypoint1,base_link,1,2,3,4,5,f,0.1,0.2,0.3";
  vector<double> waypointUnpacked = {};
  vector<double> normalUnpacked = {};
  testUnpackWaypoint(str, "", "", waypointUnpacked, normalUnpacked);
}

// Test to unpack a damage string missing its ID
TEST_F(WaypointParserTest, TestMissingIDWaypoint) {
  string str = "base_link,2,1,2,,4,5,0.1,0.2,0.3";
  vector<double> waypointUnpacked = {};
  vector<double> normalUnpacked = {};
  testUnpackWaypoint(str, "", "", waypointUnpacked, normalUnpacked);
}

// Test to unpack a damage string with a wrond ID
TEST_F(WaypointParserTest, TestNumberOnlyIDWaypoint) {
  string str = "1,base_link,2,1,2,,4,5,0.1,0.2,0.3";
  vector<double> waypointUnpacked = {};
  vector<double> normalUnpacked = {};
  testUnpackWaypoint(str, "", "", waypointUnpacked, normalUnpacked);
}

// Test to unpack a damage string missing its reference frame
TEST_F(WaypointParserTest, TestMissingRefFrameWaypoint) {
  string str = "waypoint2,1,2,3,4,5,6,0.1,0.2,0.3";
  vector<double> waypointUnpacked = {};
  vector<double> normalUnpacked = {};
  testUnpackWaypoint(str, "", "", waypointUnpacked, normalUnpacked);
}

// Test to unpack a damage string missing one position value
TEST_F(WaypointParserTest, TestMissingValueWaypoint) {
  string str = "waypoint2,base_link,1,2,,4,5,0.1,0.2,0.3";
  vector<double> waypointUnpacked = {};
  vector<double> normalUnpacked = {};
  testUnpackWaypoint(str, "", "", waypointUnpacked, normalUnpacked);
}

// Test to unpack a damage string having extra values
TEST_F(WaypointParserTest, TestExtraValueWaypoint) {
  string str = "waypoint2,base_link,1,2,3,4,5,6,7,0.1,0.2,0.3";
  vector<double> waypointUnpacked = {};
  vector<double> normalUnpacked = {};
  testUnpackWaypoint(str, "", "", waypointUnpacked, normalUnpacked);
}

// Test to unpack a damage string having an extra comma
TEST_F(WaypointParserTest, TestExtraCommaWaypoint) {
  string str = "waypoint2,base_link,1,2,3,4,5,6,0.1,0.2,0.3,";
  vector<double> waypointUnpacked = {1, 2, 3, 4, 5, 6};
  vector<double> normalUnpacked = {0.1, 0.2, 0.3};
  testUnpackWaypoint(str, "waypoint2", "base_link", waypointUnpacked, normalUnpacked);
}

// Test to unpack a damage string having multiple extra commas
TEST_F(WaypointParserTest, TestExtraCommasWaypoint) {
  string str = "waypoint2,base_link,1,2,3,4,5,6,0.1,0.2,0.3,,,";
  vector<double> waypointUnpacked = {};
  vector<double> normalUnpacked = {};
  testUnpackWaypoint(str, "", "", waypointUnpacked, normalUnpacked);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_subtask");
  ::testing::InitGoogleTest(&argc, argv);

  // Set the filter to run only the specific test
  // ::testing::GTEST_FLAG(filter) = "RoboticArmUrTest.TestReferenceConfiguration";

  return RUN_ALL_TESTS();
}
