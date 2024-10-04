/**
 * @file MAMPlanner.cpp
 * @brief Declaration of the MAMPlanner class
 * @author [Louis Munier]
 * @date 2024-09-12
 */
#include "MAMPlanner.h"

#include "RoboticArmFactory.h"
#include "yaml_tools.h"

using namespace std;

MAMPlanner::MAMPlanner(ROSVersion rosVersion, ros::NodeHandle& nh) :
    spinner_(1), nh_(nh), tfListener_(tfBuffer_), subTask_(make_unique<Subtask>(nh_)) {
  RoboticArmFactory armFactory = RoboticArmFactory();
  robot_ = armFactory.createRoboticArm("ur5_robot", rosVersion);
  robot_->printInfo();

  initMoveit_();

  pubWeldingState_ = nh_.advertise<std_msgs::Bool>("welding_state", 1);
  pubDisplayTrajectory_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 20);
  pubWaypoint_ = nh_.advertise<geometry_msgs::PoseStamped>("debug_waypoint", 10);

  // Add obstacles

  // addStaticObstacles_();

  // getWaypoints_(); // Elise
}

bool MAMPlanner::planTrajectory() {
  cout << "Planning trajectory" << endl;

  // TODO(lmunier): Avoid using dynamic_cast
  RoboticArmUr5* robotUr5 = dynamic_cast<RoboticArmUr5*>(robot_.get());

  currentWPointID_ = 0;
  bestPlan_.clear();
  geometry_msgs::Pose currentPose = moveGroup_->getCurrentPose().pose;

  bool pathFound = false;

  while (subTask_->isSubtaskEmpty()) {
    float TIME_WAIT = 0.2;
    ROS_WARN("Waiting for Waypoints");
    ros::Duration(TIME_WAIT).sleep();
  }

  while (!subTask_->isSubtaskEmpty()) {
    std::vector<std::vector<double>> waypoint = subTask_->getROI<double>();
    geometry_msgs::Pose startTaskPose = generatePose_(waypoint[0]);
    geometry_msgs::Pose endTaskPose = generatePose_(waypoint[1]);
    publishWaypoint_(currentPose, "base_link_inertia");
    publishWaypoint_(startTaskPose, "base_link_inertia");

    // Robot goes to start pose
    pathFound = computeTrajectory_(currentPose, startTaskPose, false, robotUr5);
    if (!pathFound) {
      return pathFound;
    }

    // Robot welding task
    pathFound = computeTrajectory_(startTaskPose, endTaskPose, true, robotUr5);
    if (!pathFound) {
      return pathFound;
    }
  }

  return true;
}

bool MAMPlanner::computeTrajectory_(const geometry_msgs::Pose currentPose,
                                    const geometry_msgs::Pose nextPose,
                                    const bool welding,
                                    RoboticArmUr5* robotUr5) {
  vector<vector<double>> ikSolutions{};
  bool isPathFound = false;

  robotUr5->getIKGeo(geometryToEigen_(currentPose.orientation), geometryToEigen_(currentPose.position), ikSolutions);
  for (const auto& ikSol : ikSolutions) {
    vector<double> startConfig = ikSol;
    isPathFound += computePath_(startConfig, currentPose, nextPose, welding);
  }
  if (!isPathFound) {
    ROS_ERROR("No path found to go to start waypoint");
    return false;
  }
  return true;
}

void MAMPlanner::executeTrajectory() {
  bool success = false;
  vector<double> firstJointConfig{};
  cout << "Executing trajectory" << endl;

  if (bestPlan_.empty()) {
    ROS_ERROR("No trajectory to execute");
    return;
  }

  for (const auto& trajectory : bestPlan_) {
    firstJointConfig = trajectory.joint_trajectory.points[0].positions;

    if (!robot_->isAtJointPosition(firstJointConfig)) {
      moveGroup_->setJointValueTarget(firstJointConfig);
      success = (moveGroup_->move() == moveit::core::MoveItErrorCode::SUCCESS);

      if (!success) {
        ROS_ERROR("Failed to move to the starting joint configuration.");
        return;
      }
    }

    moveGroup_->stop();
    moveGroup_->clearPoseTargets();
    success = moveGroup_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;

    if (success) {
      ROS_INFO("Trajectory executed successfully");
    } else {
      ROS_ERROR("Failed to execute trajectory");
    }

    moveGroup_->stop();
    moveGroup_->clearPoseTargets();
  }
}

void MAMPlanner::initMoveit_() {
  const string robotGroup = "manipulator";
  ros::Duration timeout(2.0);

  try {
    moveGroup_ = make_unique<moveit::planning_interface::MoveGroupInterface>(
        robotGroup, make_shared<tf2_ros::Buffer>(), timeout);
  } catch (const runtime_error& e) {
    ROS_ERROR("Failed to initialize MoveGroupInterface: %s", e.what());
    return;
  }

  planningScene_ = make_unique<moveit::planning_interface::PlanningSceneInterface>();
  setupMovegroup_();

  spinner_.start();

  // Wait for the scene to get ready
  ros::Duration(1.0).sleep();
}

void MAMPlanner::setupMovegroup_() {
  moveGroup_->setPoseReferenceFrame(robot_->getReferenceFrame());
  moveGroup_->setPlannerId("RRTConnect");
  moveGroup_->setPlanningTime(2.0);
  moveGroup_->setNumPlanningAttempts(10);
  moveGroup_->setGoalPositionTolerance(0.005);
  moveGroup_->setGoalOrientationTolerance(0.01);
}

bool MAMPlanner::computePath_(const vector<double>& startConfig,
                              const geometry_msgs::Pose& currentPose,
                              const geometry_msgs::Pose& targetPose,
                              const bool isWeldging) {
  bool success = false;
  const string robotGroup = "manipulator";
  moveit_msgs::RobotTrajectory planCartesianTrajectory{};
  moveGroup_->clearPoseTargets();

  // Create a RobotState object and set it to the desired starting joint configuration
  moveit::core::RobotState startState(*moveGroup_->getCurrentState());
  startState.setJointGroupPositions(robotGroup, startConfig);

  // Set the starting state in the planning scene
  moveGroup_->setStartState(startState);

  if (isWeldging) {
    // Compute Cartesian path
    vector<geometry_msgs::Pose> waypoints{};
    waypoints.push_back(currentPose);
    waypoints.push_back(targetPose);

    double fraction = moveGroup_->computeCartesianPath(waypoints, 0.01, 0.0, planCartesianTrajectory, true);
    success = fraction == 1.0; // Path fully computed
    cout << "Fraction: " << fraction << endl;
  } else {
    // Set the target pose
    moveGroup_->setPoseTarget(targetPose);

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = moveGroup_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    planCartesianTrajectory = plan.trajectory_;
  }

  if (success) {
    if (currentWPointID_ >= bestPlan_.size()) {
      bestPlan_.push_back(planCartesianTrajectory);
    } else {
      double currentPlanSize = planCartesianTrajectory.joint_trajectory.points.size();
      double bestPlanSize = bestPlan_[currentWPointID_].joint_trajectory.points.size();

      if (currentPlanSize < bestPlanSize) {
        bestPlan_[currentWPointID_] = planCartesianTrajectory;
      }
    }
  }

  return success;
}

geometry_msgs::Pose MAMPlanner::generatePose_(const vector<double>& pose) {
  if (pose.size() != 6 && pose.size() != 7) {
    ROS_ERROR("Invalid pose size it should be 6 for Euler use or 7 for Quaternions.");
    return geometry_msgs::Pose();
  }

  geometry_msgs::Pose newPose;
  newPose.position.x = pose[0];
  newPose.position.y = pose[1];
  newPose.position.z = pose[2];

  if (pose.size() == 6) {
    Eigen::Quaterniond q = eulerToQuaternion_<double>({pose[3], pose[4], pose[5]});

    newPose.orientation.x = q.x();
    newPose.orientation.y = q.y();
    newPose.orientation.z = q.z();
    newPose.orientation.w = q.w();
  } else if (pose.size() == 7) {
    newPose.orientation.x = pose[3];
    newPose.orientation.y = pose[4];
    newPose.orientation.z = pose[5];
    newPose.orientation.w = pose[6];
  }

  return newPose;
}

geometry_msgs::Pose MAMPlanner::projectPose_(const geometry_msgs::Pose& pose,
                                             const string& fromFrame,
                                             const string& toFrame) {
  geometry_msgs::PoseStamped inputPose;
  inputPose.pose = pose;
  inputPose.header.frame_id = fromFrame;
  inputPose.header.stamp = ros::Time::now();

  try {
    geometry_msgs::TransformStamped transformStamped =
        tfBuffer_.lookupTransform(toFrame, fromFrame, ros::Time(0), ros::Duration(1.0));
    geometry_msgs::PoseStamped outputPose;

    tf2::doTransform(inputPose, outputPose, transformStamped);
    return outputPose.pose;
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Could NOT transform: %s", ex.what());
    return pose;
  }
}

void MAMPlanner::createNewFrame_(const string& parentFrame,
                                 const string& newFrame,
                                 const geometry_msgs::Transform& transform) {
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parentFrame;
  transformStamped.child_frame_id = newFrame;
  transformStamped.transform = transform;

  br_.sendTransform(transformStamped);
}

// void MAMPlanner::getWaypoints_() {
//   string yamlPath = YamlTools::getYamlPath("waypoints.yaml", string(WP5_MAM_PLANNER_DIR));
//   YAML::Node config = YAML::LoadFile(yamlPath);

//   Waypoint newWaypoint{};
//   vector<double> pose{};
//   vector<double> tmpOrientation{};
//   geometry_msgs::Pose newPose{};

//   for (const auto& waypoint : config) {
//     newWaypoint.clear();
//     newWaypoint.frame = waypoint["frame"].as<string>();

//     // Get the pose and orientation, convert it and project it to the robot frame
//     pose = waypoint["pos"].as<vector<double>>();
//     tmpOrientation = waypoint["angle"].as<vector<double>>();
//     pose.insert(pose.end(), tmpOrientation.begin(), tmpOrientation.end());

//     newPose = generatePose_(pose);
//     // TODO(lmunier): Fix the frame projection, use it when planning only to take into account the robot position
//     //newPose = projectPose_(newPose, newWaypoint.frame, robot_->getReferenceFrame());

//     // Store the waypoint
//     newWaypoint.pos = geometryToEigen_(newPose.position);
//     newWaypoint.quat = geometryToEigen_(newPose.orientation);
//     newWaypoint.speed = waypoint["speed"].as<double>();
//     newWaypoint.welding = waypoint["welding"].as<bool>();

//     waypoints_.push_back(newWaypoint);
//   }
// }

void MAMPlanner::publishWaypoint_(const geometry_msgs::Pose& pose, const std::string& frameId) {
  float TIME_WAIT = 0.2;
  size_t NB_PUBLISH = 3;

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = frameId;
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.pose = pose;

  for (size_t i = 0; i < NB_PUBLISH; ++i) {
    pubWaypoint_.publish(poseStamped);
    ros::Duration(TIME_WAIT).sleep();
  }
}

Eigen::Vector3d MAMPlanner::geometryToEigen_(const geometry_msgs::Point& point) {
  return Eigen::Vector3d{point.x, point.y, point.z};
}

Eigen::Quaterniond MAMPlanner::geometryToEigen_(const geometry_msgs::Quaternion& orientation) {
  return Eigen::Quaterniond{orientation.w, orientation.x, orientation.y, orientation.z};
}

geometry_msgs::Point MAMPlanner::eigenToGeometry_(const Eigen::Vector3d& position) {
  geometry_msgs::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();

  return move(point);
}

geometry_msgs::Quaternion MAMPlanner::eigenToGeometry_(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion quat;
  quat.x = orientation.x();
  quat.y = orientation.y();
  quat.z = orientation.z();
  quat.w = orientation.w();

  return move(quat);
}

void MAMPlanner::addStaticObstacles_() {
  string name{}, type{};

  moveit_msgs::CollisionObject collisionObject;
  vector<moveit_msgs::CollisionObject> collisionObjects;
  collisionObject.header.frame_id = moveGroup_->getPlanningFrame();

  // Get obstacles from the config file
  string yamlPath = YamlTools::getYamlPath("obstacles.yaml", string(WP5_MAM_PLANNER_DIR));
  YAML::Node config = YAML::LoadFile(yamlPath);
  vector<YAML::Node> obstacles = config["obstacles"].as<vector<YAML::Node>>();

  for (const auto& obstacle : obstacles) {
    name = obstacle["name"].as<string>();
    type = obstacle["type"].as<string>();
    geometry_msgs::PoseStamped newObstacle;

    newObstacle.header.frame_id = collisionObject.header.frame_id;
    newObstacle.pose = generatePose_(obstacle["pose"].as<vector<double>>());

    // Create the collision object
    if (type == "box") {
      vector<double> size = obstacle["size"].as<vector<double>>();

      collisionObject.primitives.push_back(createBox_(name, size));
    } else if (type == "cylinder") {
      double height = obstacle["height"].as<double>();
      double radius = obstacle["radius"].as<double>();

      collisionObject.primitives.push_back(createCylinder_(name, height, radius));
    } else if (type == "sphere") {
      double radius = obstacle["radius"].as<double>();

      collisionObject.primitives.push_back(createSphere_(name, radius));
    } else if (type == "mesh") {
      string meshPath = obstacle["mesh_path"].as<string>();

      collisionObject.meshes.push_back(createMesh_(name, meshPath));
    } else {
      ROS_ERROR("No such obstacle type.");
      continue;
    }

    // Add the obstacle to the scene
    if (type == "mesh") {
      collisionObject.mesh_poses.push_back(newObstacle.pose);
    } else {
      collisionObject.primitive_poses.push_back(newObstacle.pose);
    }

    collisionObject.id = name;
    collisionObjects.push_back(collisionObject);
  }

  planningScene_->applyCollisionObjects(collisionObjects);
}

shape_msgs::SolidPrimitive MAMPlanner::MAMPlanner::createBox_(const string name, const vector<double>& size) const {
  ROS_INFO("Adding box %s...", name.c_str());
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;

  primitive.dimensions.resize(3);
  primitive.dimensions = size;

  return move(primitive);
}

shape_msgs::SolidPrimitive MAMPlanner::createCylinder_(const string name,
                                                       const double height,
                                                       const double radius) const {
  ROS_INFO("Adding cylinder %s...", name.c_str());
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;

  primitive.dimensions.resize(2);
  primitive.dimensions = {height, radius};

  return move(primitive);
}

shape_msgs::SolidPrimitive MAMPlanner::createSphere_(const string name, const double radius) const {
  ROS_INFO("Adding sphere %s...", name.c_str());
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;

  primitive.dimensions.resize(1);
  primitive.dimensions = {radius};

  return move(primitive);
}

shape_msgs::Mesh MAMPlanner::createMesh_(const string name, const string meshPath) const {
  ROS_INFO("Adding mesh %s...", name.c_str());
  shapes::Mesh* m = shapes::createMeshFromResource(meshPath);

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg meshMsg;
  shapes::constructMsgFromShape(m, meshMsg);
  mesh = boost::get<shape_msgs::Mesh>(meshMsg);

  return move(mesh);
}