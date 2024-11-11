#!/usr/bin/env python
import sys
import copy
import tf
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, dist, fabs, cos

from moveit_commander.conversions import pose_to_list


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

        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))

        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)

        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        move_group = moveit_commander.MoveGroupCommander(MOVE_GROUPE_NAME)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def is_valid_quaternion(self, q):
        return abs(q.x**2 + q.y**2 + q.z**2 + q.w**2 - 1.0) < 1e-6

    def generate_init_pose(self) -> geometry_msgs.msg.Pose:
        # Define the initial pose of the robot
        init_pose = geometry_msgs.msg.Pose()
        init_pose.position.x = 0.5
        init_pose.position.y = -0.47
        init_pose.position.z = 0.8

        # Orientation
        roll = pi / 2
        pitch = 0
        yaw = 0
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        init_pose.orientation.x = q[0]
        init_pose.orientation.y = q[1]
        init_pose.orientation.z = q[2]
        init_pose.orientation.w = q[3]

        print(
            f"Quaternion validity : {self.is_valid_quaternion(init_pose.orientation)}"
        )

        return init_pose

    def go_to_pose_goal(self):
        move_group = self.move_group

        init_pose = self.generate_init_pose()
        move_group.set_pose_target(init_pose)

        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(init_pose, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

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

        ##

        move_group.execute(plan, wait=True)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


def main():
    try:
        move_group = MoveGroupPythonInterface()

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        move_group.go_to_pose_goal()

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = move_group.plan_cartesian_path()

        input(
            "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        )
        move_group.display_trajectory(cartesian_plan)

        input("============ Press `Enter` to execute a saved path ...")
        move_group.execute_plan(cartesian_plan)

        input(
            "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        )
        cartesian_plan, fraction = move_group.plan_cartesian_path(scale=-1)
        move_group.execute_plan(cartesian_plan)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
