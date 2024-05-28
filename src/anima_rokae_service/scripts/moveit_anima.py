#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This script is designed to be used with Moveit and share to ANIMA to
# test simple trajectories on their Rokae CR7 robot.

# Author: Louis Munier
# Last update: 2024-05-13

import copy
import rospy
import signal
import sys
import tf
import threading
import time
import tf.transformations

from geometry_msgs.msg import Pose, PoseStamped
from math import pi, radians
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, RobotCommander
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Bool


class AnimaMoveit:
    def __init__(self, safe_wait=3):
        """
        Main function that initializes the ROS node, MoveIt, and executes the trajectory.
        """
        self.FLAG_STOP = False
        self.FLAG_THREAD = False
        self.LISTENER = None

        self.LST_TARGET_OFFSETt = None
        self.EE_OFFSET = None
        self.ID_SCENARIO = None

        signal.signal(signal.SIGINT, self.signal_handler)

        try:
            self.init_moveit(safe_wait)
            self.get_information()

            # Add obstacles
            self.add_obstacle(
                [-0.3, 0.0, -0.25, 0.0, 0.0, 0.0, 1.0],
                (1.3, 0.77, 0.54),
                "robot_platform"
            )
            # self.add_obstacle(
            #    [-0.2, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0],
            #    (0.2, 0.77, 0.6),
            #    "welding_boxe"
            # )

            # Setup move group options
            self.move_group.allow_replanning(True)
            self.move_group.set_goal_position_tolerance(0.005)
            self.move_group.set_goal_orientation_tolerance(0.01)
        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return

    def signal_handler(self, sig, frame):
        """
        Signal handler function that handles the Ctrl+C signal.
        """
        if self.FLAG_THREAD:
            self.FLAG_STOP = True
        else:
            sys.exit(0)

    def plan_display_move(self, time_wait=2):
        plan_init_pose = self.move_group.plan()

        self.display_trajectory(plan_init_pose[1])

        if rospy.get_param('anima_step_by_step'):
            input(
                "============ Press `Enter` to execute the trajectory ... ============"
            )

        self.move_group.execute(plan_init_pose[1], wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def set_scenario(self, id_scenario):
        """
        Depending on the scenario we define the target offsets for the waypoints
        The target offsets are defined as dictionaries with the keys :
        - 'pos' [x, y, z]
        - 'angle' [roll, pitch, yaw]
        - 'speed'
        - 'weld'
        WARNING : Giving a rotation will not output in a straight line

        Args:
            - id_scenario: the identification number for the scenario
        """
        self.ID_SCENARIO = id_scenario

        if id_scenario == 1:
            # Multiples lines side by side with an offset
            self.LST_TARGET_OFFSET = [
                {'pos': [0, -0.05, 0], 'speed': 0.1, 'weld': False},
                {'pos': [-0.1, 0, 0], 'speed': 0.01, 'weld': True},
                {'pos': [0, 0.05, 0], 'speed': 0.5, 'weld': False},
                {'pos': [0.1, 0, -0.05], 'speed': 0.1, 'weld': False}
            ]

            # End effector offset
            self.EE_OFFSET = {
                'pos': [0, 0, 0],
                'angle': [0, 0, 0]
            }
        elif id_scenario == 2:
            # Multiples lines on top of each other
            self.LST_TARGET_OFFSET = [
                {'pos': [0, -0.05, 0], 'speed': 0.01, 'weld': False},
                {'pos': [-0.1, 0, 0], 'speed': 0.001, 'weld': True},
                {'pos': [0, 0.052, 0], 'speed': 0.05, 'weld': False},
                {'pos': [0.1, 0, 0], 'speed': 0.1, 'weld': False}
            ]

            # End effector offset
            self.EE_OFFSET = {
                'pos': [0, 0, 0],
                'angle': [0, 0, 0]
            }
        else:
            rospy.logerr("Invalid scenario ID.")
            sys.exit(0)

    def add_obstacle(self, pose: list, size: tuple, name: str):
        """
        Function to add obstacles to the planning scene.

        Args:
            pose: Position and Orientation of the obstacle
            size: Size of the obstacle
            name: Name of the obstacle
        """
        frame_id = self.robot_base

        if len(pose) != 7:
            rospy.logerr(
                "Pose should be size 7 : pose - xyz, orientation - xyzw.")

        if len(size) != 3:
            rospy.logerr("Size should be size 3 - xyz.")

        new_obstacle = PoseStamped()
        new_obstacle.header.frame_id = frame_id
        new_obstacle.pose = self.generate_pose(pose)
        self.scene.add_box(name, new_obstacle, size=size)

    def plan_execute_trajectory(self):
        """
        Function to plan and execute the trajectory.
        """
        waypoints = []
        waypoints.append(self.move_group.get_current_pose().pose)

        for i in range(rospy.get_param('~nb_iterations')):
            for to in self.LST_TARGET_OFFSET:
                waypoints = self.generate_waypoints(
                    waypoints[-1], [to], self.EE_OFFSET
                )

                if i == rospy.get_param('~nb_iterations') - 1 and to == self.LST_TARGET_OFFSET[-1]:
                    break

                # Compute the Cartesian path
                iter = 0
                fraction = 0.0
                while fraction < 1.0:
                    iter += 1

                    if iter > rospy.get_param('~max_path_trial'):
                        print(
                            f"Path planning was not successful, fraction: {fraction}, iteration {i}."
                        )
                        return

                    (plan_trajectory, fraction) = self.move_group.compute_cartesian_path(
                        waypoints,   # waypoints to follow
                        0.01,        # eef_step
                        0.0,         # jump_threshold
                        True)        # avoid_collisions

                    print("Fraction of cartesian path well computed: ", fraction)

                    # If path not possible, reconfigure robot to a new joint position
                    if fraction < 1.0:
                        print("Retrying path planning...")
                        self.move_group.set_pose_target(waypoints[-1])
                        new_plan = self.move_group.plan()

                        self.display_trajectory(new_plan[1])
                        input(
                            "============ Press `Enter` to execute the trajectory ... ============"
                        )

                        self.move_group.execute(new_plan[1], wait=True)

                # Scale the velocity of the trajectory
                plan_trajectory = self.move_group.retime_trajectory(
                    self.robot.get_current_state(),
                    plan_trajectory,
                    to['speed']
                )

                # Display the trajectory
                self.display_trajectory(plan_trajectory)

                if rospy.get_param('anima_step_by_step'):
                    input(
                        "============ Press `Enter` to execute the trajectory ... ============")

                # Publish the welding state
                self.pub_welding_state.publish(to['weld'])

                # Execute the plan in a separate thread
                execute_thread = threading.Thread(
                    target=self.move_group.execute, args=(
                        plan_trajectory, True)
                )
                execute_thread.start()

                # Wait for the execution to finish or for the stop flag to be set
                while execute_thread.is_alive():
                    self.FLAG_THREAD = True

                    if self.FLAG_STOP:
                        print('Stopping...')
                        self.move_group.stop()
                        self.move_group.clear_pose_targets()
                        execute_thread.join()
                        return

                    time.sleep(0.1)

                self.FLAG_THREAD = False

        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def init_moveit(self, safe_wait):
        """
        Function to initialize MoveIt and the ROS node.

        Args:
            - safe_wait: Parameter to wait before each movement, can be adapted
                         using rosparam functions
        """
        roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)

        self.robot_type = rospy.get_param('/robot_type')
        self.robot_group = rospy.get_param(
            '~' + self.robot_type + '/group'
        )
        self.robot_base = rospy.get_param(
            '~' + self.robot_type + '/base'
        )
        self.robot_end_effector = rospy.get_param(
            '~' + self.robot_type + '/end_effector'
        )

        self.robot = RobotCommander(
            '/' + self.robot_type + "/robot_description"
        )
        self.scene = PlanningSceneInterface()

        self.move_group = MoveGroupCommander(self.robot_group)
        self.move_group.set_pose_reference_frame(self.robot_base)

        self.pub_welding_state = rospy.Publisher(
            "welding_state", Bool, queue_size=1
        )
        self.pub_display_trajectory = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=20,
        )

        self.LISTENER = tf.TransformListener()
        self.LISTENER.waitForTransform(
            self.robot_base, self.robot_end_effector, rospy.Time(), rospy.Duration(4.0)
        )

        # Set ros params
        rospy.set_param('anima_safe_wait', safe_wait)
        rospy.set_param('anima_step_by_step', rospy.get_param('~step_by_step'))

        # Wait for the scene to get ready
        rospy.sleep(1)

    def get_information(self):
        """
        Function to get information about the robot and the MoveGroup.
        """
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print(f"============ Reference frame: {planning_frame}")

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print(f"============ End effector: {eef_link}")

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print(f"============ Robot Groups: {group_names}")

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state:")
        print(self.robot.get_current_state())
        print("")

    def is_valid_quaternion(self, q):
        """
        Function to check if a quaternion is valid.

        Args:
            q: The quaternion to check.

        Returns:
            True if the quaternion is valid, False otherwise.
        """
        return abs(q.x**2 + q.y**2 + q.z**2 + q.w**2 - 1.0) < 1e-6

    def generate_init_pose(self) -> Pose:
        """
        Function to generate the initial pose of the robot.

        Returns:
            The initial pose as a Pose object.
        """
        init_pose = rospy.get_param(
            '~' + self.robot_type + '/init_pose'
        )

        # Define the initial pose of the robot
        self.init_pose = Pose()
        self.init_pose.position.x = init_pose[0]
        self.init_pose.position.y = init_pose[1]
        self.init_pose.position.z = init_pose[2]

        # Orientation
        roll = init_pose[3]
        pitch = init_pose[4]
        yaw = init_pose[5]
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        self.init_pose.orientation.x = q[0]
        self.init_pose.orientation.y = q[1]
        self.init_pose.orientation.z = q[2]
        self.init_pose.orientation.w = q[3]

        if not self.is_valid_quaternion(self.init_pose.orientation):
            print("Invalid quaternion.")
            exit(0)

    def get_pose_from_ref(self, pose, base_ref, end_ref, angle_rad=True):
        position_pose = pose['pos']

        # Convert Euler angles to quaternion
        orientation_pose = pose['angle']
        if not angle_rad:
            orientation_pose = [radians(x) for x in orientation_pose]

        orientation_pose_quat = tf.transformations.quaternion_from_euler(
            orientation_pose[0], orientation_pose[1], orientation_pose[2]
        )

        # Convert point and orientation to homogeneous coordinates
        pose_base = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(position_pose),
            tf.transformations.quaternion_matrix(orientation_pose_quat)
        )

        # Get the transform from the base frame to the end effector frame
        (trans, rot) = self.LISTENER.lookupTransform(
            base_ref, end_ref, rospy.Time(0)
        )

        # Convert the transform to a 4x4 matrix
        transform = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(trans),
            tf.transformations.quaternion_matrix(rot)
        )

        # Apply the transform to the point and orientation
        pose_end = tf.transformations.concatenate_matrices(
            transform, pose_base
        )

        # Extract the transformed point and orientation
        point_end = tf.transformations.translation_from_matrix(pose_end)
        orientation_end_quat = tf.transformations.quaternion_from_matrix(
            pose_end
        )

        return self.generate_pose(point_end.tolist() + orientation_end_quat.tolist())

    def generate_pose(self, pose: list) -> Pose:
        """
        Function to generate an obstacle pose.

        Args:
            pose: The pose of the obstacle as a list.

        Returns:
            The obstacle pose as a PoseStamped object.
        """
        new_pose = Pose()

        new_pose.position.x = pose[0]
        new_pose.position.y = pose[1]
        new_pose.position.z = pose[2]

        new_pose.orientation.x = pose[3]
        new_pose.orientation.y = pose[4]
        new_pose.orientation.z = pose[5]
        new_pose.orientation.w = pose[6]

        return new_pose

    def generate_waypoints(self, orig_pose: Pose, target_offset: list, end_effector_offset: dict) -> list:
        """
        Function to generate the waypoints for the trajectory.

        Args:
            orig_pose: The original pose of the robot.
            target_offset: The target offset for the waypoints.
            end_effector_offset: The offset of the end effector.

        Returns:
            The waypoints as a list of Pose objects.
        """
        waypoints = []
        new_pose = copy.deepcopy(orig_pose)

        # Create the waypoints by incrementing x, z, and y positions or roll, pitch,
        # yaw angle respectively
        for target in target_offset:
            pos_offset = [0, 0, 0]
            rot_offset = [0, 0, 0]

            for k, v in target.items():
                if k == 'pos':
                    pos_offset = v

                if k == 'angle':
                    rot_offset = v

        waypoints.append(self.apply_offset_to_pose(new_pose, {
            'pos': pos_offset,
            'angle': rot_offset
        }))

        return waypoints

    def apply_offset_to_pose(self, orig_pose: Pose, offset: dict, angle_rad=True) -> Pose:
        """
        Function to apply an offset to a pose.

        Args:
            orig_pose: The original pose.
            offset: The offset to be applied.

        Returns:
            The new pose with the offset applied.
        """
        new_pose = copy.deepcopy(orig_pose)

        # Apply position offset
        new_pose.position.x += offset['pos'][0]
        new_pose.position.y += offset['pos'][1]
        new_pose.position.z += offset['pos'][2]

        # Apply orientation offset
        orientation_pose = offset['angle']
        if not angle_rad:
            orientation_pose = [radians(x) for x in orientation_pose]

        q_trans = tf.transformations.quaternion_from_euler(
            orientation_pose[0], orientation_pose[1], orientation_pose[2]
        )

        q_orig = [
            new_pose.orientation.x, new_pose.orientation.y,
            new_pose.orientation.z, new_pose.orientation.w
        ]

        quat_new = tf.transformations.quaternion_multiply(
            q_orig, q_trans
        )

        new_pose.orientation.x = quat_new[0]
        new_pose.orientation.y = quat_new[1]
        new_pose.orientation.z = quat_new[2]
        new_pose.orientation.w = quat_new[3]

        return new_pose

    def go_home(self):
        """
        Function to move the robot to the home position.
        """
        homing_pose = rospy.get_param('~' + self.robot_type + '/home_pose')
        self.move_group.set_joint_value_target(homing_pose)
        self.plan_display_move()

    def display_trajectory(self, plan):
        """
        Function to display the planned trajectory.

        Args:
            plan: The planned trajectory.
        """
        display = DisplayTrajectory()
        display.trajectory_start = self.robot.get_current_state()
        display.trajectory.append(plan)

        self.pub_display_trajectory.publish(display)

        if not rospy.get_param('anima_step_by_step'):
            rospy.sleep(rospy.get_param('anima_safe_wait'))
