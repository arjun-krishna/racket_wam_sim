"""
Visual Debug of WAM urdf to configure rest pose + limits
"""
import pybullet as p
import pybullet_data as pd
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.3])


robot_uid = p.loadURDF(
    "../assets/sport/table_tennis/wam7.urdf", [0, 0, 3], [1, 0, 0, 0], useFixedBase=1,
)

end_effector_index = 8

j_idx = []  # joint_id
jn = []  # name
ll = []  # lower limits (null space)
ul = []  # upper limits (null space)
jr = []  # joint ranges = ul - ll?
rp = []  # rest poses
jd = []  # joint damping coefficients

num_joints = p.getNumJoints(robot_uid)
for joint_idx in range(num_joints):
    joint_info = p.getJointInfo(robot_uid, joint_idx)

    name, type, damping, lower_limit, upper_limit = [
        joint_info[i] for i in [1, 2, 6, 8, 9]
    ]
    print(joint_idx, name, type, damping, lower_limit, upper_limit)

    if type == p.JOINT_FIXED:
        continue

    j_idx.append(joint_idx)
    jn.append(name)
    ll.append(lower_limit)
    ul.append(upper_limit)
    jr.append(upper_limit - lower_limit)
    rp.append(0.0)
    jd.append(damping)

p.setGravity(0, 0, 0)
t = 0

prev_pose = [0, 0, 0]
has_prev_pose = 0
use_null_space = 1
use_orientation = 1
ik_solver = 0

# debug params
pos_ids = [
    p.addUserDebugParameter("pos_x", -1, 1, 0),
    p.addUserDebugParameter("pos_y", -1, 1, 0),
    p.addUserDebugParameter("pos_z", 0.5, 1.25, 0.5),
]

orn_id = p.addUserDebugParameter("roll", -np.pi, np.pi, 0)

while True:
    p.stepSimulation()

    pos = [p.readUserDebugParameter(id) for id in pos_ids]
    roll = p.readUserDebugParameter(orn_id)
    orn = p.getQuaternionFromEuler([roll, -np.pi, 0])  # face downwards

    joint_poses = p.calculateInverseKinematics(
        robot_uid, end_effector_index, pos, orn, ll, ul, jr, rp
    )

    for i, j_id in enumerate(j_idx):
        p.resetJointState(robot_uid, j_id, joint_poses[i])
