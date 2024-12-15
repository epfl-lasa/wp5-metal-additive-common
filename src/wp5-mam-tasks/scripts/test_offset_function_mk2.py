import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R


def plot_waypoints(steps):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    colors = ["blue", "green", "red", "purple", "orange"]
    markers = ["o", "^", "s", "D", "x"]

    for i, step in enumerate(steps):
        waypoints = step["waypoints"]
        x = [wp.position.x for wp in waypoints]
        y = [wp.position.y for wp in waypoints]
        z = [wp.position.z for wp in waypoints]
        ax.plot(
            x,
            y,
            z,
            marker=markers[i % len(markers)],
            color=colors[i % len(colors)],
            label=f"Step {i+1}",
        )

        for wp in waypoints:
            quat = [
                wp.orientation.x,
                wp.orientation.y,
                wp.orientation.z,
                wp.orientation.w,
            ]
            rot = R.from_quat(quat)
            direction = rot.apply([1, 0, 0])
            ax.quiver(
                wp.position.x,
                wp.position.y,
                wp.position.z,
                direction[0],
                direction[1],
                direction[2],
                length=0.1,
                normalize=True,
                color=colors[i % len(colors)],
            )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Waypoints Trajectory")
    plt.legend()
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


def get_normal_vector(wp_vector, waypoint):
    temp_vector = [-waypoint.position.x, -waypoint.position.y, -waypoint.position.z]
    temp_vector /= np.linalg.norm(temp_vector)

    normal_vector = np.cross(wp_vector, temp_vector)
    normal_vector /= np.linalg.norm(normal_vector)

    return normal_vector


def get_offset_vector(wp_vector, normal_vector):
    offset_vec_normalized = np.cross(wp_vector, normal_vector)
    offset_vec_normalized /= np.linalg.norm(offset_vec_normalized)

    return offset_vec_normalized


def compute_trajectory(waypoints, ee_pose_work_offset, ee_pose_offset):
    steps = []
    wp_vector = np.array(
        [
            waypoints[1].position.x - waypoints[0].position.x,
            waypoints[1].position.y - waypoints[0].position.y,
            waypoints[1].position.z - waypoints[0].position.z,
        ]
    )
    wp_vector /= np.linalg.norm(wp_vector)

    normal_vector = get_normal_vector(wp_vector, waypoints[1])
    offset_vec_normalized = get_offset_vector(wp_vector, normal_vector)

    offset_vec_work = offset_vec_normalized * ee_pose_work_offset[-1]
    offset_vec = offset_vec_normalized * ee_pose_offset[-1]

    rotation = R.from_rotvec(np.deg2rad(20) * -normal_vector)
    x_vector = np.array([1, 0, 0])
    rotation_quaternion = R.from_rotvec(np.arccos(np.dot(x_vector, offset_vec_normalized)) * np.cross(x_vector, offset_vec_normalized) / np.linalg.norm(np.cross(x_vector, offset_vec_normalized)))

    combined_quaternion = (rotation * rotation_quaternion).as_quat()

    offset_pose_work = eigen_to_geometry(combined_quaternion, offset_vec_work)
    offset_pose = eigen_to_geometry(combined_quaternion, offset_vec)

    steps.append(
        {"waypoints": [add_offset(waypoints[0], offset_pose_work)], "vectors": []}
    )
    steps.append(
        {"waypoints": [add_offset(wp, offset_pose) for wp in waypoints], "vectors": []}
    )
    steps.append(
        {"waypoints": [add_offset(waypoints[-1], offset_pose_work)], "vectors": []}
    )

    return steps


def eigen_to_geometry(quat, vec):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = vec
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        quat
    )
    return pose


def main():
    waypoints = [Pose(), Pose()]
    waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z = (
        -0.2,
        0.5,
        1.33,
    )
    (
        waypoints[0].orientation.x,
        waypoints[0].orientation.y,
        waypoints[0].orientation.z,
        waypoints[0].orientation.w,
    ) = (0, 0, 0, 1)
    waypoints[1].position.x, waypoints[1].position.y, waypoints[1].position.z = (
        0.1,
        0.5,
        1.33,
    )
    (
        waypoints[1].orientation.x,
        waypoints[1].orientation.y,
        waypoints[1].orientation.z,
        waypoints[1].orientation.w,
    ) = (0, 0, 0, 1)

    ee_pose_work_offset = [0, 0, -0.3]
    ee_pose_offset = [0, 0, -0.02]

    steps = compute_trajectory(waypoints, ee_pose_work_offset, ee_pose_offset)
    plot_waypoints(steps)


if __name__ == "__main__":
    main()
