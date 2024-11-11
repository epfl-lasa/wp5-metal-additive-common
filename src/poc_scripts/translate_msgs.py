import rospy
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Float64MultiArray

def callback(data):
    # The DisplayTrajectory message might contain multiple trajectories
    # We're interested in the last one
    trajectory = data.trajectory[-1]

    previous_time = None
    time_difference = None

    for point in trajectory.joint_trajectory.points:
        positions = Float64MultiArray()

        positions.data = list(point.positions)

        # Publish the joint positions, velocities, and efforts on new topics
        pub_positions.publish(positions)

        # Calculate the time difference between this point and the previous one
        current_time = point.time_from_start.to_sec()
        if previous_time is not None:
            time_difference = current_time - previous_time
        else:
            time_difference = current_time

        # Update the previous time to the current time
        previous_time = current_time

        # Sleep for the duration of this trajectory point
        rospy.sleep(rospy.Duration.from_sec(time_difference))

rospy.init_node('trajectory_to_states')

# Create a subscriber
sub = rospy.Subscriber('/move_group/monitored_planning_scene', DisplayTrajectory, callback)

# Create publishers
pub_positions = rospy.Publisher('/joint_trajectory/joint_positions', Float64MultiArray, queue_size=10)

# Keep the program running until it's stopped
rospy.spin()