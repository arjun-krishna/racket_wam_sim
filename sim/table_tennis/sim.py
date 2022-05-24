import os
import pybullet as p
import pybullet_data as pd
import pybullet_utils.bullet_client as bc
import numpy as np

from sim.table_tennis.config import SimConfig


class TTSim:
    def __init__(self, client: bc.BulletClient, config: SimConfig) -> None:
        self.client = client
        self.config = config

        client.setAdditionalSearchPath(pd.getDataPath())

        client.loadURDF(self.get_assets_path("sport/table_tennis/table.urdf"))
        self.ball_uid = client.loadURDF(
            self.get_assets_path("sport/table_tennis/ball.urdf")
        )
        self.robot_uid = client.loadURDF(
            self.get_assets_path("sport/table_tennis/wam7.urdf"),
            config.arm.pos,
            config.arm.orientation,
            useFixedBase=1,
        )

        self.run_sim()

    def get_assets_path(self, path: str) -> str:
        return os.path.join(os.path.dirname(__file__), "../../assets/", path)

    def run_sim(self):
        while True:
            pass
