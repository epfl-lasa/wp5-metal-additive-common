from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import matplotlib.pyplot as plt

from scipy.interpolate import CubicSpline
from math import tau, dist, fabs, cos
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Constraints, OrientationConstraint
from mpl_toolkits.mplot3d import Axes3D

MOVE_GROUPE_NAME = "manipulator"


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)

        d = dist((x1, y1, z1), (x0, y0, z0))

        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("test_moveit", anonymous=True)
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        move_group = moveit_commander.MoveGroupCommander(MOVE_GROUPE_NAME)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def get_pose(self, pose):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.x = pose[3]
        pose_goal.orientation.y = pose[4]
        pose_goal.orientation.z = pose[5]
        pose_goal.orientation.w = pose[6]

        return pose_goal

    def go_to_pose_goal(self, pose):
        pose_goal = self.get_pose(pose)
        return self.go_to_pose_goal(pose_goal)

    def go_to_pose_goal(self, pose: geometry_msgs.msg.Pose):
        move_group = self.move_group
        move_group.set_pose_target(pose)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1
        wpose.position.y += scale * 0.2
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def display_trajectory(self, plan):

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):

        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def create_orientation_constraints(self, orientation_tolerance):
        constraints = Constraints()
        ocm = OrientationConstraint()

        ocm.orientation.w = 1
        ocm.absolute_x_axis_tolerance = orientation_tolerance
        ocm.absolute_y_axis_tolerance = orientation_tolerance
        ocm.absolute_z_axis_tolerance = orientation_tolerance
        ocm.weight = 1.0

        constraints.orientation_constraints.append(ocm)
        return constraints


def main():
    try:
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()

        # Define the waypoints
        waypoints = np.array(
            [
                [0.3, 0.4, 0.4],
                [0.5, 0.2, 0.3],
                [0.4, 0.3, 0.4],
                [0.1, 0.2, 0.3],
                [0.4, 0.4, 0.4],
                [-0.2, 0.2, -0.3],
            ]
        )

        # Fit a cubic spline to the waypoints for each dimension
        spline_x = CubicSpline(np.arange(len(waypoints)), waypoints[:, 0])
        spline_y = CubicSpline(np.arange(len(waypoints)), waypoints[:, 1])
        spline_z = CubicSpline(np.arange(len(waypoints)), waypoints[:, 2])

        # Discretize the spline
        t_values = np.linspace(0, len(waypoints) - 1, num=100)
        x_values = spline_x(t_values)
        y_values = spline_y(t_values)
        z_values = spline_z(t_values)

        # Create orientation constraints with a tolerance of 3.14 radians
        # constraints = tutorial.create_orientation_constraints(3.14)
        # tutorial.move_group.set_path_constraints(constraints)

        # Create a new figure
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # Plot the waypoints
        ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], color="r")
        ax.plot(x_values, y_values, z_values)

        # Set the labels
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        plt.show()

        # Convert the discretized spline to a list of poses
        poses = []
        for x, y, z in zip(x_values, y_values, z_values):
            pose = geometry_msgs.msg.Pose()

            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0

            poses.append(pose)

        tutorial.go_to_pose_goal(poses[0])
        print("Initial pose reached")

        iter = 0
        while len(poses) > 0:
            iter += 1

            if iter > 10:
                return

            (plan_trajectory, fraction) = tutorial.move_group.compute_cartesian_path(
                poses,  # waypoints to follow
                0.01,  # eef_step
                0.0,  # jump_threshold
                True,
            )  # avoid_collisions

            tutorial.display_trajectory(plan_trajectory)
            tutorial.move_group.execute(plan_trajectory, wait=True)

            del poses[: int(fraction * len(poses))]

            if fraction < 1.0:
                print("Retrying path planning...")
                tutorial.move_group.set_pose_target(poses[0])
                new_plan = tutorial.move_group.plan()

                tutorial.move_group.execute(new_plan[1], wait=True)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
