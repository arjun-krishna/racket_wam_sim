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
    ll = [-2.6, -1.98, -2.8, -0.9, -4.55, -1.57, -2.95]  # lower limits (null space)
    ul = [2.6, 1.98, 2.8, 3.1, 1.25, 1.57, 2.95]  # upper limits (null space)
    jr = [
        5.2,
        3.96,
        5.6,
        4.0,
        5.8,
        3.14,
        5.9,
    ]  # joint ranges = ul - ll? (TODO check IK)
    rp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # rest poses
    jd = [1.98, 0.55, 1.65, 0.88, 0.55, 0.11, 0.11]  # joint damping coefficients

    # joint vel/acc/torq limits (emperically determined)
    v_lim = [3.23, 6.3, 10.0, 10.0, 24.0, 19.0, 27.0]
    a_lim = [6.82, 11.5, 17.0, 21.5, 84.0, 110.0, 100.0]

    # obtained from manual, choose slightly higher than stall rating and well below rated cable limit
    # https://web.barrett.com/support/WAM_Documentation/WAM_UserManual_AH-00.pdf
    torq_lim = [1.55, 1.55, 1.55, 1.49, 0.4, 0.4, 0.095]

    # racket_center - end_effector
    # target_pos - (offset) = end_effector_pos
    racket_center_offset = np.array([])  # racket offset
    racket_rp_orn = np.array([])  # rest-pose orientation
