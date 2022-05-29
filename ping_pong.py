import pybullet as p
import pybullet_data as pd
import numpy as np
import time
from sim.arm.config import WAMConfig

# debug
from sim.arm.joint_profile import JointProfiler

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

p.setGravity(0, 0, 0)
p.loadURDF("plane.urdf")
p.loadURDF("assets/sport/table_tennis/table.urdf", useFixedBase=1)
# p.loadURDF("assets/sport/table_tennis/racket.urdf", [0, -1.8, 1.1], useFixedBase=1)

ball_id = p.loadURDF("assets/sport/table_tennis/ball.urdf", [0, -1, 2.1])
robot_id = p.loadURDF(
    "assets/sport/table_tennis/wam7.urdf", [0, -1.95, 3], [1, 0, 0, 0], useFixedBase=1
)
jp = JointProfiler(
    p, robot_id, WAMConfig()
) # separate tracking thread


def reset():
    p.resetBasePositionAndOrientation(ball_id, [0, 1, 1.3], [0, 0, 0, 1])
    p.resetBaseVelocity(ball_id, [0, -7.0, 0], [0.0, 0.0, 0.0])


def apply_ball_forces():
    # coefficients obtained from: https://arxiv.org/pdf/2109.03100.pdf
    drag_coef = 0.4
    density_air = 1.29  # kg/m^3
    lift_coef = 0.6
    r1 = 0.02  # radius
    mass = 0.0027 # kg
    g = 10 # m/s^2
    A = np.pi * r1 * r1

    pos, _ = p.getBasePositionAndOrientation(ball_id)
    v, w = p.getBaseVelocity(ball_id)
    v, w = np.array(v), np.array(w)

    F_d = -0.5 * drag_coef * density_air * A * np.linalg.norm(v) * v  # drag
    F_m = 0.5 * lift_coef * density_air * A * r1 * np.cross(w, v)  # magnus force
    F_g = mass * np.array([0, 0, -g])

    p.applyExternalForce(ball_id, -1, F_d + F_m + F_g, pos, p.WORLD_FRAME)


def get_ball_trajectory(pos, quat, vel, ang_vel, t=2, dt=1 / 240):
    # obtain the ball trajectory from euler integration of the dynamics
    # consider only semi-elastic collision with the board and in-elastic ground
    #
    # EOM:
    #         X'' = -g + F_d / m + F_m / m
    #   (board collision plane): z = 0.74 x = [-1.370,1.370] y = [-0.762, 0.762]
    #         v_z = -0.95 * v_z                     // (table COR = 0.95)
    #   (ground): z = 0
    #         v_z = 0
    state = (pos, quat, vel, ang_vel)
    traj = [state]

    def f(state):
        return state[1]

    pass


### ARM config ========================================================
end_effector_index = 8

j_idx = []  # joint_id
jn = []  # name
ll = []  # lower limits (null space)
ul = []  # upper limits (null space)
jr = []  # joint ranges = ul - ll?
rp = []  # rest poses
jd = []  # joint damping coefficients

num_joints = p.getNumJoints(robot_id)
for joint_idx in range(num_joints):
    joint_info = p.getJointInfo(robot_id, joint_idx)

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

pos_ids = [
    p.addUserDebugParameter("pos_x", -1, 1, 0),
    p.addUserDebugParameter("pos_y", -2.95, -0.95, -1.62),
    p.addUserDebugParameter("pos_z", 0.5, 1.4, 1.286),
]

orn_id = p.addUserDebugParameter("roll", -np.pi, np.pi, 0)

ARM_IN_SWING = False
t = 0
T = int(0.3 * 240)  # 0.3 second (swing end)


def enable_swing():
    global ARM_IN_SWING, t
    if ARM_IN_SWING:
        return
    # if ball is in hitting distance swing is enabled
    ball_pos, _ = p.getBasePositionAndOrientation(ball_id)
    ball_pos = np.array(ball_pos)

    centroid = np.array([0, -1.62, 1.286])
    dh_side = 0.2 * np.sqrt(3)  # \frac{\sqrt{3}}{2} a

    if (ball_pos >= (centroid - dh_side)).all() and (
        ball_pos <= (centroid + dh_side)
    ).all():
        ARM_IN_SWING = True
        t = 0


# =================================================================

# ik reset plane
def arm_controller():
    global ARM_IN_SWING, t
    # hitting plane definition
    pos = [p.readUserDebugParameter(id) for id in pos_ids]
    roll = p.readUserDebugParameter(orn_id)
    orn = p.getQuaternionFromEuler([roll, -np.pi, 0])  # face downwards

    if ARM_IN_SWING:  # assume hitting plan(e has been reached
        pos[1] -= 0.2 * np.sin(np.pi * (t / T))
        joint_poses = p.calculateInverseKinematics(robot_id, end_effector_index, pos,)
        target_vel = 2.5 * np.cos(np.pi * (t / T))
        t += 1
        if t > T:
            ARM_IN_SWING = False
    else:
        # target pose for reaching hitting plane
        joint_poses = p.calculateInverseKinematics(
            robot_id, end_effector_index, pos, orn, ll, ul, jr, rp
        )
        target_vel = 0

    for i, j_id in enumerate(j_idx):
        # p.resetJointState(robot_id, j_id, joint_poses[i])
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=j_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_poses[i],
            targetVelocity=target_vel,
            force=500,
            positionGain=0.03,
            velocityGain=1,
        )


def process_key_events():
    key_events = p.getKeyboardEvents()
    if 114 in key_events:  # r
        reset()
    if 101 in key_events:  # e
        pass


reset()
jp.start()
while True:
    enable_swing()
    apply_ball_forces()
    arm_controller()
    p.stepSimulation()
    time.sleep(1 / 240)
    process_key_events()
