from typing import List
from threading import Thread
import time
import pybullet_utils.bullet_client as bc
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from sim.arm.config import WAMConfig

class JointProfiler(Thread):
    def __init__(
        self,
        client: bc.BulletClient,
        robot_uid: int,
        cfg: WAMConfig,
        dt: float = 1 / 240.0,
    ) -> None:
        super().__init__()
        self.client = client
        self.robot_uid = robot_uid
        self.cfg = cfg
        self.dt = dt

        self.ll = np.array(
            [cfg.ll, [-1 * v for v in cfg.v_lim], [-1 * v for v in cfg.torq_lim]]
        ).T
        self.ul = np.array([cfg.ul, cfg.v_lim, cfg.torq_lim]).T

    def run(self):
        self.create_jp_plot(len(self.cfg.jn), self.cfg.jn, ll=self.ll, ul=self.ul)
        while True:
            js = self.client.getJointStates(self.robot_uid, self.cfg.j_idx)
            jp = [s[0] for s in js]
            jv = [s[1] for s in js]
            jt = [s[3] for s in js]
            self.update_jp_plot(jp, jv, jt)
            time.sleep(self.dt)

    def create_jp_plot(
        self,
        num_joints: int,
        joint_names: List[str] = None,
        T: int = 240,  # 1 second
        ll: np.array = None,  # lower limits
        ul: np.array = None,  # upper limits
    ):
        matplotlib.use("TKAgg")
        x = np.arange(T)
        y = np.zeros(T)
        data = {}
        fig, axes = plt.subplots(num_joints, 3)
        for i in range(num_joints):
            for j, n in enumerate(["pos", "vel", "torq"]):
                if joint_names is not None:
                    axes[i, j].set_title(f"{joint_names[i]}/{n}")
                else:
                    axes[i, j].set_title(f"joint{i}_{n}")
                (line,) = axes[i, j].plot(x, y)
                axes[i, j].set(xlabel="t")
                # axes[i, j].label_outer()
                data[(i, j)] = line
                if ll is not None and ul is not None:
                    axes[i, j].set_ylim(ll[i, j], ul[i, j])
                axes[i, j].set_xticks([])

        # fig.tight_layout()
        plt.show(block=False)
        self.fig = fig
        self.data = data

    def update_jp_plot(
        self, joint_pos: np.array, joint_vel: np.array, joint_torq: np.array,
    ):
        fig, data = self.fig, self.data
        for i in range(len(joint_pos)):
            for j, v in enumerate([joint_pos[i], joint_vel[i], joint_torq[i]]):
                line = data[(i, j)]
                y = line.get_ydata().copy()
                y[:-1] = y[1:]
                y[-1] = v
                line.set_ydata(y)
        fig.canvas.draw()
        fig.canvas.flush_events()
