import pybullet as p
from sim.arm.config import WAMConfig


class WAMIK:
    def __init__(self, robot_uid: int, cfg: WAMConfig):
        self.robot_uid = robot_uid
        self.cfg = cfg

    def get_pose(self, ef_pos, ef_orn):
        return p.calculateInverseKinematics(
            self.robot_uid,
            self.cfg.end_effector_index,
            ef_pos,
            ef_orn,
            self.cfg.ll,
            self.cfg.ul,
            self.cfg.jr,
            self.cfg.rp,
        )
