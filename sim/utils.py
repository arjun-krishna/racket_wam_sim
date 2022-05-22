import numpy as np


def rotation_matrix_from_quat(q):
    s = 1 / np.linalg.norm(q)
    i, j, k, r = q
    return np.array([
        [1 - 2*s*(j**2 + k**2), 2*s*(i*j - k*r), 2*s*(i*k + j*r)],
        [2*s*(i*j + k*r), 1 - 2*s*(i**2 + k**2), 2*s*(i*k - i*r)],
        [2*s*(i*k - j*r), 2*s*(j*k + i*r), 1 - 2*s*(i**2 + j**2)],
    ])


def quat_mult(q1: np.array, q2: np.array):
    v1, w1 = q1[:3], q1[3]
    v2, w2 = q2[:3], q2[3]
    q = np.zeros(4)
    q[:3] = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    q[3] = w1 * w2 - v1 @ v2
    return q
