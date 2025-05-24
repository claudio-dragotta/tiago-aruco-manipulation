import numpy as np
from scipy.spatial.transform import Rotation as R

def dh_transformation_matrix(d, theta, a, alpha):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    return np.array([
        [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
        [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(dh_params, joint_angles):
    T_current = np.eye(4)
    for i, (d, _, a, alpha) in enumerate(dh_params):
        T_i = dh_transformation_matrix(d, joint_angles[i], a, alpha)
        T_current = np.dot(T_current, T_i)
    return T_current

def calculate_geometric_jacobian(dh_params, joint_angles):
    n = len(joint_angles)
    T = np.eye(4)
    T_matrices = []

    for i in range(n):
        d, _, a, alpha = dh_params[i]
        theta = joint_angles[i]
        Ti = dh_transformation_matrix(d, theta, a, alpha)
        T = np.dot(T, Ti)
        T_matrices.append(T)

    z_axes = [np.array([0, 0, 1])]
    positions = [np.array([0, 0, 0])]

    for T in T_matrices:
        z_axes.append(T[:3, 2])
        positions.append(T[:3, 3])

    p_e = positions[-1]
    J = np.zeros((6, n))
    for i in range(n):
        J[:3, i] = np.cross(z_axes[i], p_e - positions[i])  # linear
        J[3:, i] = z_axes[i]                                # angular

    return J

def inverse_kinematics_with_orientation(dh_params, initial_q, target_position, target_orientation_quat, joint_limits,
                                        Kp=1.0, Ko=1.0, max_iterations=1000, tol=1e-2, dt=0.5):
    q = np.array(initial_q)
    R_target = R.from_quat(target_orientation_quat).as_matrix()

    for _ in range(max_iterations):
        T_ee = forward_kinematics(dh_params, q)
        pos_ee = T_ee[:3, 3]
        R_ee = T_ee[:3, :3]

        # position error
        e_pos = target_position - pos_ee

        # orientation error (rotation vector)
        R_err = R_target @ R_ee.T
        rotvec = R.from_matrix(R_err).as_rotvec()
        e_ori = rotvec

        # combined error
        error = np.hstack((Kp * e_pos, Ko * e_ori))

        if np.linalg.norm(error) < tol:
            return q.tolist(), True

        J = calculate_geometric_jacobian(dh_params, q)
        J_full = J  # already 6xN

        dq = np.linalg.pinv(J_full) @ error
        q += dq * dt

        for i in range(len(q)):
            q[i] = np.clip(q[i], joint_limits[i, 0], joint_limits[i, 1])

    return q.tolist(), False

# DH params e limiti per TIAGo
dh_params_tiago = [
    (0.08, 0, 0, 90 * np.pi / 180),
    (0.0, 0, 0, -90 * np.pi / 180),
    (0.17, 0, 0, -90 * np.pi / 180),
    (0.0, 0, 0, 90 * np.pi / 180),
    (0.25, 0, 0, 90 * np.pi / 180),
    (0.0, 0, 0, -90 * np.pi / 180),
    (0.04, 0, 0, 0),
]

joint_limits_tiago = np.array([
    [-150 * np.pi / 180, 114 * np.pi / 180],
    [-67 * np.pi / 180, 109 * np.pi / 180],
    [-150 * np.pi / 180, 41 * np.pi / 180],
    [-92 * np.pi / 180, 110 * np.pi / 180],
    [-150 * np.pi / 180, 150 * np.pi / 180],
    [92 * np.pi / 180, 113 * np.pi / 180],
    [-150 * np.pi / 180, 150 * np.pi / 180]
])
