import pybullet_utils.bullet_client as bc
import numpy as np
from sim.arm.joint_profile import create_jp_plot, update_jp_plot
from sim.arm.config import WAMConfig


class WAMController:
    def __init__(
        self,
        client: bc.BulletClient,
        robot_uid: int,
        cfg: WAMConfig,
        profile_joints = True,
    ):
        self.client = client
        self.robot_uid = robot_uid
        self.cfg = cfg

        self.profile_joints = profile_joints
        if profile_joints:
            ll = np.array(
                [cfg.ll, [-1 * v for v in cfg.v_lim], [-1 * v for v in cfg.torq_lim],]
            ).T
            ul = np.array([cfg.ul, cfg.v_lim, cfg.torq_lim]).T
            self.joint_profile_plt_obj = create_jp_plot(
                7, joint_names=cfg.jn, ll=ll, ul=ul
            )

    def step(self, obs):
        """
        obs: (FrameOfReference: wam/fixed_base_joint)
            - ball[x,y,z]
            - joint pos[j1...j7]
            - joint vel[j1...j7]
            - racket_pos[x,y,z] (TODO need vel?)
            - racket_orn[x,y,z]
        """
        js = self.client.getJointStates(self.robot_uid, self.cfg.j_idx)
        jp = [s[0] for s in js]
        jv = [s[1] for s in js]
        jt = [s[3] for s in js]
        if self.profile_joints:
            update_jp_plot(self.joint_profile_plt_obj, jp, jv, jt)


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
