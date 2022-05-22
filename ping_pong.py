import pybullet as p
import pybullet_data as pd
import numpy as np
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")
p.loadURDF("assets/sport/table_tennis/table.urdf", useFixedBase=1)

# p.loadURDF('assets/sport/table_tennis/racket.urdf', [0, -1, 3], useFixedBase=1)
ball_id = p.loadURDF("assets/sport/table_tennis/ball.urdf", [0, -1, 2.1])
robot_id = p.loadURDF(
    "assets/sport/table_tennis/wam7.urdf", [0, -1.95, 3], [1, 0, 0, 0], useFixedBase=1
)


def reset():
    p.resetBasePositionAndOrientation(ball_id, [0, 1, 1.3], [0, 0, 0, 1])
    p.resetBaseVelocity(ball_id, [0, -7.0, 0], [0.0, 0.0, 0.0])


def apply_ball_forces():
    # coefficients obtained from: https://arxiv.org/pdf/2109.03100.pdf
    drag_coef = 0.4
    density_air = 1.29  # kg/m^3
    lift_coef = 0.6
    r1 = 0.02  # radius
    A = np.pi * r1 * r1

    v, w = p.getBaseVelocity(ball_id)
    v, w = np.array(v), np.array(w)

    F_d = -0.5 * drag_coef * density_air * A * np.linalg.norm(v) * v  # drag
    F_m = 0.5 * lift_coef * density_air * A * r1 * np.cross(w, v)  # magnus force

    p.applyExternalForce(ball_id, -1, F_d + F_m, [0, 0, 0], p.LINK_FRAME)


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


ARM_ENABLED = False

# hitting plane
def arm_controller():
    pass


def process_key_events():
    key_events = p.getKeyboardEvents()
    if 114 in key_events:  # r
        reset()
    if 101 in key_events:  # e
        ARM_ENABLED = not ARM_ENABLED


while True:
    apply_ball_forces()
    arm_controller()
    p.stepSimulation()
    time.sleep(1 / 240)
    process_key_events()
