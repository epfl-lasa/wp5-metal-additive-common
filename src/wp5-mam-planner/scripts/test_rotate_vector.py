#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# This script is used to quickly iterate and test the rotate_vector_in_plan function.

# Author: Louis Munier
# Last update: 2024-11-05

import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rotate_vector_in_plan(points_array, theta, debug = False):
    # Compute vector normal to plan defined by the 3 points
    plan_vector1 = points_array[0] - points_array[1]
    plan_vector2 = points_array[0] - points_array[2]
    normal_vector = np.cross(plan_vector1, plan_vector2)
    normal_vector /= np.linalg.norm(normal_vector)

    # Rotate vector defined by plan, forming by points : points_array, by theta
    quat_rotation = R.from_quat([
        normal_vector[0] * np.sin(theta / 2),
        normal_vector[1] * np.sin(theta / 2),
        normal_vector[2] * np.sin(theta / 2),
        np.cos(theta / 2)
    ])
    rotated_vect_plan = quat_rotation.apply(plan_vector2)

    if debug:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.quiver(0, 0, 0, points_array[0][0], points_array[0][1], points_array[0][2], color='black')
        ax.quiver(0, 0, 0, points_array[1][0], points_array[1][1], points_array[1][2], color='black')
        ax.quiver(0, 0, 0, points_array[2][0], points_array[2][1], points_array[2][2], color='black')

        # Plot original vectors
        ax.quiver(0, 0, 0, plan_vector1[0], plan_vector1[1], plan_vector1[2], color='r')
        ax.text(plan_vector1[0], plan_vector1[1], plan_vector1[2], 'Vec 1', color='r')

        ax.quiver(0, 0, 0, plan_vector2[0], plan_vector2[1], plan_vector2[2], color='g')
        ax.text(plan_vector2[0], plan_vector2[1], plan_vector2[2], 'Vec 2', color='g')

        ax.quiver(0, 0, 0, normal_vector[0], normal_vector[1], normal_vector[2], color='b')
        ax.text(normal_vector[0], normal_vector[1], normal_vector[2], 'normal_vector', color='b')

        # Take the inverse of the normal vector ToDo in C++: then convert into robot frame
        ax.quiver(0, 0, 0, -rotated_vect_plan[0], -rotated_vect_plan[1], -rotated_vect_plan[2], color='c')
        ax.text(-rotated_vect_plan[0], -rotated_vect_plan[1], -rotated_vect_plan[2], 'rotated_vect_plan', color='c')

        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        plt.show()

    return rotated_vect_plan

if __name__ == "__main__":
    # Define test data
    points_array = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ])
    theta = np.pi / 4 # 45 degrees

    # Call the function
    rotated_vect_plan = rotate_vector_in_plan(points_array, theta, debug=True)
    print(f"Rotated vector in plan: {rotated_vect_plan} (theta = {theta})")