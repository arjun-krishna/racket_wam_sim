import pybullet_utils.bullet_client as bc
import pybullet as p
import numpy as np
import hydra

from sim.table_tennis.config import SimConfig
from sim.table_tennis.sim import TTSim

# NOTE: cfg is a DictConfig instance (SimConfig just duck types the variables)
@hydra.main(config_path="conf", config_name="table_tennis")
def main(cfg: SimConfig):
    client = bc.BulletClient(connection_mode=p.GUI)
    sim = TTSim(client, cfg)


if __name__ == "__main__":
    main()
