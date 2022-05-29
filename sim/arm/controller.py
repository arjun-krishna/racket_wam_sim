import pybullet_utils.bullet_client as bc
from sim.arm.config import WAMConfig


class WAMController:
    def __init__(self, client: bc.BulletClient, robot_uid: int, cfg: WAMConfig):
        self.client = client
        self.robot_uid = robot_uid
        self.cfg = cfg

    def step(self, obs):
        """
        obs: (FrameOfReference: wam/fixed_base_joint)
            - ball[x,y,z]
            - joint pos[j1...j7]
            - joint vel[j1...j7]
            - racket_pos[x,y,z] (TODO need vel?)
            - racket_orn[x,y,z]
            - target_hit_pos[x,y,z]
            - target_ball_speed[v]
            - target_y_spin [top/back spin]
            - target_x_spin [side spin]
        """
        pass

    def ik_pose(self, ef_pos, ef_orn):
        # get IK pose for specified end effector pose and orientation
        return self.client.calculateInverseKinematics(
            self.robot_uid,
            self.cfg.end_effector_index,
            ef_pos,
            ef_orn,
            self.cfg.ll,
            self.cfg.ul,
            self.cfg.jr,
            self.cfg.rp,
        )
