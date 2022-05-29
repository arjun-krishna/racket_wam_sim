import numpy as np


class WAMConfig:
    end_effector_index = 8

    # See: ping_pong.py on how these are obtained from URDF
    j_idx = [1, 2, 3, 4, 5, 6, 7]  # joint_id
    jn = [  # name
        "wam/base_yaw_joint",
        "wam/shoulder_pitch_joint",
        "wam/shoulder_yaw_joint",
        "wam/elbow_pitch_joint",
        "wam/wrist_yaw_joint",
        "wam/wrist_pitch_joint",
        "wam/palm_yaw_joint",
    ]
    ll = [-2.6, -1.985, -2.8, -0.9, -4.55, -1.5707, -3.0]  # lower limits (null space)
    ul = [2.6, 1.985, 2.8, 3.14159, 1.25, 1.5707, 3.0]  # upper limits (null space)
    jr = [5.2, 3.97, 5.6, 4.04159, 5.8, 3.1414, 6.0]  # joint ranges = ul - ll?
    rp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # rest poses
    jd = [1.98, 0.55, 1.65, 0.88, 0.55, 0.11, 0.11]  # joint damping coefficients

    # racket_center - end_effector
    # target_pos - (offset) = end_effector_pos
    racket_center_offset = np.array([])  # racket offset
    racket_rp_orn = np.array([])  # rest-pose orientation
