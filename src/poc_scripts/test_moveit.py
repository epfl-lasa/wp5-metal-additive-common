#!/usr/bin/env python
import sys
import rospy
import copy
import tf
from moveit_commander import (
    MoveGroupCommander,
    PlanningSceneInterface,
    roscpp_initialize,
    RobotCommander,
)
from moveit_msgs.msg import RobotTrajectory
from math import pi
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PoseStamped


MOVE_GROUPE_NAME = "manipulator"


def main():
    scene, robot, move_group, pub_welding_state = init_moveit()
    get_information(robot, move_group)

    scene.add_box(
        "robot_platform",
        generate_obstacle([-0.3, 0.0, -0.25, 0.0, 0.0, 0.0, 1.0]),
        size=(1.3, 0.77, 0.54),
    )

    # scene.add_box(
    #     "welding_boxe",
    #     generate_obstacle(
    #         [-0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
    #     ),
    #     size=(0.2, 0.77, 0.6)
    # )

    init_pose = generate_init_pose()
    move_group.go(init_pose, wait=True)
    move_group.stop()

    # Trajectory sequence
    # Multiples lines side by side
    # lst_target_offset = [
    #     {'y': -0.05, 'speed': 0.01, 'weld': False},
    #     {'x': -0.1, 'speed': 0.001, 'weld': True},
    #     {'y': 0.05, 'speed': 0.05, 'weld': False},
    #     {'x': 0.1, 'z': -0.05, 'speed': 0.1, 'weld': False}
    # ]

    # Multiples lines on top of each other
    lst_target_offset = [
        {"y": -0.05, "speed": 0.01, "weld": False},
        {"x": -0.1, "speed": 0.001, "weld": True},
        {"y": 0.052, "speed": 0.05, "weld": False},
        {"x": 0.1, "speed": 0.1, "weld": False},
    ]

    waypoints = []
    waypoints.append(move_group.get_current_pose().pose)

    nb_iterations = 3
    for i in range(nb_iterations):
        for to in lst_target_offset:
            waypoints = generate_waypoints(waypoints[-1], [to])

            if i == nb_iterations - 1 and to == lst_target_offset[-1]:
                break

            # Compute the Cartesian path
            (plan_trajectory, fraction) = move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0,  # jump_threshold
                True,
            )  # avoid_collisions

            if fraction < 1.0:
                print(
                    f"Path planning was not successful, fraction: {fraction}, iteration {i}."
                )
                return

            # Scale the velocity of the trajectory
            plan_trajectory = move_group.retime_trajectory(
                robot.get_current_state(), plan_trajectory, to["speed"]
            )

            # Publish the welding state
            pub_welding_state.publish(to["weld"])

            # Execute the plan
            move_group.execute(plan_trajectory, wait=True)


def init_moveit() -> tuple:
    roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface", anonymous=True)

    robot = RobotCommander()
    scene = PlanningSceneInterface()

    group = MoveGroupCommander(MOVE_GROUPE_NAME)
    group.set_pose_reference_frame("world")

    pub_welding_state = rospy.Publisher("welding_state", Bool, queue_size=1)

    # Wait for the scene to get ready
    rospy.sleep(1)

    return scene, robot, group, pub_welding_state


def get_information(robot, move_group):
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print(f"============ Reference frame: {planning_frame}")

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print(f"============ End effector: {eef_link}")

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print(f"============ Robot Groups: {group_names}")

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state:")
    print(robot.get_current_state())
    print("")


def is_valid_quaternion(q):
    return abs(q.x**2 + q.y**2 + q.z**2 + q.w**2 - 1.0) < 1e-6


def generate_init_pose() -> Pose:
    # Define the initial pose of the robot
    init_pose = Pose()
    init_pose.position.x = 0.082
    init_pose.position.y = -0.45
    init_pose.position.z = 0.65

    # Orientation
    roll = pi / 2
    pitch = 0
    yaw = 0
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    init_pose.orientation.x = q[0]
    init_pose.orientation.y = q[1]
    init_pose.orientation.z = q[2]
    init_pose.orientation.w = q[3]

    print(f"Quaternion validity : {is_valid_quaternion(init_pose.orientation)}")

    return init_pose


def generate_obstacle(pose: list) -> PoseStamped:
    # Define the pose of a box (in the same frame as the robot)
    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = pose[0]
    box_pose.pose.position.y = pose[1]
    box_pose.pose.position.z = pose[2]
    box_pose.pose.orientation.x = pose[3]
    box_pose.pose.orientation.y = pose[4]
    box_pose.pose.orientation.z = pose[5]
    box_pose.pose.orientation.w = pose[6]

    return box_pose


def generate_waypoints(init_pose: Pose, target_offset: list) -> list:
    waypoints = []
    new_pose = copy.deepcopy(init_pose)

    # Create the waypoints by incrementing x, z, and y positions respectively
    for target in target_offset:
        for k, v in target.items():
            new_pose.position.x = (
                new_pose.position.x + v if k == "x" else new_pose.position.x
            )
            new_pose.position.y = (
                new_pose.position.y + v if k == "y" else new_pose.position.y
            )
            new_pose.position.z = (
                new_pose.position.z + v if k == "z" else new_pose.position.z
            )

        waypoints.append(copy.deepcopy(new_pose))

    return waypoints


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
