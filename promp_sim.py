from cmath import e
import pybullet as p
import pybullet_data as pd
import pybullet_utils.bullet_client as bc
import numpy as np
import time

#==============================================================================================================
# Ball Physics
#==============================================================================================================
def apply_ball_forces(client: bc.BulletClient, ball_id: int):
    # coefficients obtained from: https://arxiv.org/pdf/2109.03100.pdf
    drag_coef = 0.4
    density_air = 1.29  # kg/m^3
    lift_coef = 0.6
    r1 = 0.02  # radius
    mass = 0.0027 # kg
    g = 10 # m/s^2
    A = np.pi * r1 * r1

    pos, _ = client.getBasePositionAndOrientation(ball_id)
    v, w = client.getBaseVelocity(ball_id)
    v, w = np.array(v), np.array(w)

    F_d = -0.5 * drag_coef * density_air * A * np.linalg.norm(v) * v  # drag
    F_m = 0.5 * lift_coef * density_air * A * r1 * np.cross(w, v)  # magnus force
    F_g = mass * np.array([0, 0, -g])

    client.applyExternalForce(ball_id, -1, F_d + F_m + F_g, pos, p.WORLD_FRAME)

#==============================================================================================================
# Ball Physics Simulator
#==============================================================================================================
ball_sim = bc.BulletClient(p.DIRECT)
traj_sim_dt = 1 / 240
ball_sim.setTimeStep(traj_sim_dt)
ball_sim.setAdditionalSearchPath(pd.getDataPath())
ball_sim.setGravity(0, 0, 0)
ball_sim_assets = {}

ball_sim_assets['plane'] = ball_sim.loadURDF("plane.urdf")
ball_sim_assets['table'] = ball_sim.loadURDF("assets/sport/table_tennis/table.urdf", useFixedBase=1)
ball_sim_assets['ball'] = ball_sim.loadURDF("assets/sport/table_tennis/ball.urdf", [0, -1, 2.1])

def get_trajectory(pos: np.ndarray, lin_vel: np.ndarray, ang_vel: np.ndarray, T: float = 2.0):
    global ball_sim, ball_sim_assets, traj_sim_dt
    ball_id = ball_sim_assets['ball']
    ball_sim.resetBasePositionAndOrientation(ball_id, pos, [0, 0, 0, 1])
    ball_sim.resetBaseVelocity(ball_id, lin_vel, ang_vel)

    t = 0
    time_, traj = [], []
    while t < T:
        apply_ball_forces(ball_sim, ball_id)
        pos, _ = ball_sim.getBasePositionAndOrientation(ball_id)
        time_.append(t)
        traj.append(pos)
        t += traj_sim_dt
        ball_sim.stepSimulation()

    return time_, traj

#==============================================================================================================
# World Simulator
#==============================================================================================================

sim = bc.BulletClient(p.GUI)
sim_dt = 1 / 240
sim.setTimeStep(sim_dt)
sim.setAdditionalSearchPath(pd.getDataPath())
sim.setGravity(0, 0, 0)
sim_assets = {}

sim_assets['plane'] = sim.loadURDF("plane.urdf")
sim_assets['table'] = sim.loadURDF("assets/sport/table_tennis/table.urdf", useFixedBase=1)
sim_assets['ball'] = sim.loadURDF("assets/sport/table_tennis/ball.urdf", [-1, -0.5, 1.5])
sim_assets['arm'] = sim.loadURDF("assets/sport/table_tennis/wam7.urdf", [1.95, 0, 3], [1, 0, 0, 0], useFixedBase=1)

# 
j_names = [
    'wam/base_yaw_joint',
    'wam/shoulder_pitch_joint',
    'wam/shoulder_yaw_joint',
    'wam/elbow_pitch_joint',
    'wam/wrist_yaw_joint',
    'wam/wrist_pitch_joint',
    'wam/palm_yaw_joint'
]
j_idx = {}
for i in range(sim.getNumJoints(sim_assets['arm'])):
    info = sim.getJointInfo(sim_assets['arm'], i)
    if info[2] == 4:
        continue
    else:
        j_idx[info[1].decode('UTF-8')] = info[0]
print(j_idx)
j_inds = [j_idx[name] for name in j_names]
ball_start_pos = [-1.5, 0.0, 1.1]
ball_start_lin_vel = [10.0, -1.0, 0]
ball_start_ang_vel = [0, 0, 0]

#==============================================================================================================
# ProMP controller / reset based joint view
#==============================================================================================================
controller_enabled = True
if controller_enabled:
    import robpy.full_promp as promp
    mp = promp.FullProMP.load('promp.json')

    sample_t = np.linspace(0, 1, 241)
    y = mp.sample([sample_t])[0]
    phase = 'reset'
    t, T = 0, 240
    
    def pid_control(sim: bc.BulletClient, sim_assets, target_jpos: np.ndarray):
        global j_inds
        target_jpos[-1] = 0.0 # do not control palm
        sim.setJointMotorControlArray(
            sim_assets['arm'],
            j_inds,
            p.POSITION_CONTROL,
            target_jpos,
            forces=[60, 60, 45, 30, 10, 10, 10],
            positionGains=[50, 50, 30, 20, 15, 15, 15]
        )

    def read_curr_jpos(sim: bc.BulletClient, sim_assets):
        global j_inds
        return [js[0] for js in sim.getJointStates(sim_assets['arm'], j_inds)]

    def swing(sim: bc.BulletClient, sim_assets):
        global phase, y, t, T
        curr_jpos = read_curr_jpos(sim, sim_assets)
        if phase == 'reset':
            err = np.clip(y[0] - curr_jpos, a_min=-0.05, a_max=0.05)
            pid_control(sim, sim_assets, curr_jpos + err)
            t = (t + 1) % T
            if t == 0:
                phase = 'swing'
                t = 1
        if phase == 'swing':
            pid_control(sim, sim_assets, y[t])
            t = (t + 1) % T
            if t == 0:
                phase = 'reset'

else:
    def create_arm_joints_debug_parameters(sim: bc.BulletClient):
        jparams = {}
        jparams['wam/base_yaw_joint'] = sim.addUserDebugParameter('wam/base_yaw_joint', -2.6, 2.6, 0.0)
        jparams['wam/shoulder_pitch_joint'] = sim.addUserDebugParameter('wam/shoulder_pitch_joint', -1.98, 1.98, 0.0)
        jparams['wam/shoulder_yaw_joint'] = sim.addUserDebugParameter('wam/shoulder_yaw_joint', -2.8, 2.8, 0.0)
        jparams['wam/elbow_pitch_joint'] = sim.addUserDebugParameter('wam/elbow_pitch_joint', -0.9, 3.1, 0.0)
        jparams['wam/wrist_yaw_joint'] = sim.addUserDebugParameter('wam/wrist_yaw_joint', -4.55, 1.25, 0.0)
        jparams['wam/wrist_pitch_joint'] = sim.addUserDebugParameter('wam/wrist_pitch_joint', -1.57, 1.57, 0.0)
        jparams['wam/palm_yaw_joint'] = sim.addUserDebugParameter('wam/palm_yaw_joint', -2.95, 2.95, 0.0)
        return jparams

    def read_arm_joints_debug_parameters(sim: bc.BulletClient, jparams):
        return np.array([
            sim.readUserDebugParameter(jparams['wam/base_yaw_joint']),
            sim.readUserDebugParameter(jparams['wam/shoulder_pitch_joint']),
            sim.readUserDebugParameter(jparams['wam/shoulder_yaw_joint']),
            sim.readUserDebugParameter(jparams['wam/elbow_pitch_joint']),
            sim.readUserDebugParameter(jparams['wam/wrist_yaw_joint']),
            sim.readUserDebugParameter(jparams['wam/wrist_pitch_joint']),
            sim.readUserDebugParameter(jparams['wam/palm_yaw_joint']),
        ])

    jparams = create_arm_joints_debug_parameters(sim)


#==============================================================================================================
# Main Simulation Loop
#==============================================================================================================
while True:
    sim.removeAllUserDebugItems()
    ball_id = sim_assets['ball']
    sim.resetBasePositionAndOrientation(ball_id, ball_start_pos, [0, 0, 0, 1])
    sim.resetBaseVelocity(ball_id, ball_start_lin_vel, ball_start_ang_vel)

    time_, traj = get_trajectory(ball_start_pos, ball_start_lin_vel, ball_start_ang_vel, T=0.7)
    
    # plot trajectory
    # for a, b in zip(traj[:-1], traj[1:]):
    #     sim.addUserDebugLine(a, b, [1, 0, 0], 0.1)

    for i in range(2 * 240):
        apply_ball_forces(sim, ball_id)
        if not controller_enabled:
            jpos = read_arm_joints_debug_parameters(sim, jparams)
            for i, n in enumerate(j_names):
                sim.resetJointState(
                    sim_assets['arm'],
                    j_idx[n],
                    jpos[i],
                    0
                )
        else:
            swing(sim, sim_assets)
        sim.stepSimulation()
        time.sleep(sim_dt)
    time.sleep(1)

