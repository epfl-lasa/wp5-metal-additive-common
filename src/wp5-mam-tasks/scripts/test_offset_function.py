import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

def plot_waypoints(steps):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    colors = ['blue', 'green', 'red', 'purple', 'orange']
    markers = ['o', '^', 's', 'D', 'x']

    for i, step in enumerate(steps):
        waypoints = step['waypoints']
        x = [wp.position.x for wp in waypoints]
        y = [wp.position.y for wp in waypoints]
        z = [wp.position.z for wp in waypoints]
        ax.plot(x, y, z, marker=markers[i % len(markers)], color=colors[i % len(colors)], label=f'Step {i+1}')

        for wp in waypoints:
            # Extract orientation as a rotation matrix
            quat = [wp.orientation.x, wp.orientation.y, wp.orientation.z, wp.orientation.w]
            rot = R.from_quat(quat)
            direction = rot.apply([1, 0, 0])  # Assuming the orientation vector is along the x-axis

            # Plot the orientation vector
            ax.quiver(wp.position.x, wp.position.y, wp.position.z,
                      direction[0], direction[1], direction[2],
                      length=0.1, normalize=True, color=colors[i % len(colors)])

        # Plot vectors and quaternions
        for vec in step['vectors']:
            ax.quiver(vec['start'][0], vec['start'][1], vec['start'][2],
                      vec['direction'][0], vec['direction'][1], vec['direction'][2],
                      length=0.1, normalize=True, color=vec['color'], label=vec['label'])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Waypoints Trajectory')
    plt.legend()

    # Set limits to include the zero pose
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 2])

    plt.show()

def add_offset(pose, offset):
    new_pose = Pose()
    new_pose.position.x = pose.position.x + offset.position.x
    new_pose.position.y = pose.position.y + offset.position.y
    new_pose.position.z = pose.position.z + offset.position.z
    new_pose.orientation = offset.orientation
    return new_pose

def compute_trajectory(waypoints, ee_pose_work_offset, ee_pose_offset):
    steps = []

    # Apply the offset to the waypoints
    points_array = [None] * 2

    points_array[0] = np.array(
        [waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z]
    )
    points_array[1] = np.array(
        [waypoints[1].position.x, waypoints[1].position.y, waypoints[1].position.z]
    )

    wp_vector = points_array[1] - points_array[0]
    points_array[0] /= np.linalg.norm(points_array[0])
    points_array[1] /= -np.linalg.norm(points_array[1])

    normal_vector = np.cross(
        wp_vector, points_array[1]
    )
    normal_vector /= np.linalg.norm(normal_vector)

    print(np.dot(wp_vector, normal_vector))
    print(np.dot(points_array[1], normal_vector))

    # TODO: Change angle to params

    offset_vec_normalized = np.cross(wp_vector, normal_vector)
    offset_vec_normalized /= np.linalg.norm(offset_vec_normalized)

    offset_vec_work = offset_vec_normalized * ee_pose_work_offset[-1]
    offset_vec = offset_vec_normalized * ee_pose_offset[-1]

    # Rotate offset_vec_normalized by 20 degrees around -normal_vector
    rotation_angle = np.deg2rad(20)
    rotation_axis = -normal_vector
    rotation = R.from_rotvec(rotation_angle * rotation_axis)
    vectorA = rotation.apply(offset_vec_normalized)


    # Assuming vectorA is already computed and normalized
    vectorA = vectorA / np.linalg.norm(vectorA)

    # Define the x-axis
    x_axis = np.array([1, 0, 0])

    # Compute the axis of rotation (cross product)
    rotation_axis = np.cross(x_axis, vectorA)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

    # Compute the angle of rotation (dot product)
    rotation_angle = np.arccos(np.dot(x_axis, vectorA))

    # Create the quaternion from the axis and angle
    rotation = R.from_rotvec(rotation_angle * rotation_axis)
    quaternion = rotation.as_quat()

    offset_pose_work = eigen_to_geometry(quaternion, offset_vec_work)
    offset_pose = eigen_to_geometry(quaternion, offset_vec)

    # Add moving to welding pose
    step = {
        'waypoints': [add_offset(waypoints[0], offset_pose_work)],
        'vectors': [
            {'start': points_array[0], 'direction': points_array[1], 'color': 'black', 'label': 'vector1'},
            {'start': points_array[0], 'direction': wp_vector, 'color': 'black', 'label': 'vector2'},
            {'start': points_array[0], 'direction': normal_vector, 'color': 'cyan', 'label': 'Normal Vector'},
            {'start': points_array[0], 'direction': wp_vector, 'color': 'magenta', 'label': 'WP Vector'},
            {'start': points_array[0], 'direction': offset_vec_normalized, 'color': 'yellow', 'label': 'Offset Vector Normalized'}
        ]
    }
    steps.append(step)

    # Add welding waypoints
    step = {
        'waypoints': [],
        'vectors': []
    }
    for waypoint in waypoints:
        step['waypoints'].append(add_offset(waypoint, offset_pose))
    steps.append(step)

    # Add moving away from welding pose
    step = {
        'waypoints': [add_offset(waypoints[-1], offset_pose_work)],
        'vectors': []
    }
    steps.append(step)

    return steps

def eigen_to_geometry(quat, vec):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = vec
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
    return pose

def main():
    # Toy data
    waypoints = [Pose(), Pose()]
    waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z = -0.2, 0.5, 1.33
    (
        waypoints[0].orientation.x,
        waypoints[0].orientation.y,
        waypoints[0].orientation.z,
        waypoints[0].orientation.w,
    ) = (0, 0, 0, 1)

    waypoints[1].position.x, waypoints[1].position.y, waypoints[1].position.z = 0.1, 0.5, 1.33
    (
        waypoints[1].orientation.x,
        waypoints[1].orientation.y,
        waypoints[1].orientation.z,
        waypoints[1].orientation.w,
    ) = (0, 0, 0, 1)

    ee_pose_work_offset = [0, 0, -0.3]
    ee_pose_offset = [0, 0, -0.02]

    # Test the function
    steps = compute_trajectory(waypoints, ee_pose_work_offset, ee_pose_offset)

    # Plot the waypoints
    plot_waypoints(steps)

if __name__ == "__main__":
    main()