import numpy as np
import rospy

from ur_ikfast import ur_kinematics
from trac_ik_python.trac_ik import IK

# Compute IK using IKFast
ur5_arm = ur_kinematics.URKinematics('ur5')

inv_pose = ur5_arm.inverse(
    [0.3, 0.209, 0.6, -0.579, 0.579, -0.406, 0.406],
    True
)
inv_rounded = [np.round(num, 3) for num in inv_pose]
for inv in inv_rounded:
    print("inverse() all from quat", inv)

# Compute IK using TRAC-IK
ik_solver = IK(
    "base_link",
    "tool0",
    solve_type="Distance",
    urdf_string=rospy.get_param('/ur5/robot_description'),
    timeout=1
)

nb_trials = 100
ik_solutions = []
for _ in range(nb_trials):
    joint_pos = np.random.uniform(-np.pi, np.pi, 6).tolist()
    ik_solution = ik_solver.get_ik(
        joint_pos,
        0.3, 0.209, 0.6, -0.579, 0.579, -0.406, 0.406,
        bx=1e-6, by=1e-6, bz=1e-6,
        brx=1e-6, bry=1e-6, brz=1e-6
    )  # X, Y, Z, QX, QY, QZ, QW

    if ik_solution is not None:
        ik_sol_rounded = [round(elem, 1) for elem in ik_solution]

        if ik_sol_rounded not in ik_solutions:
            ik_solutions.append(ik_sol_rounded)

print(f'Found {len(ik_solutions)} solutions:')
for i, sol in enumerate(ik_solutions):
    print(f'Solution {i}: {sol} type {type(sol)}')
