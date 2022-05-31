import hydra
from hydra.core.config_store import ConfigStore
from hydra.utils import to_absolute_path
from omegaconf import DictConfig, OmegaConf

import numpy as np
import isaacgym

def train(env_cfg, train_cfg, env_class, task_name, args):
    pass

OmegaConf.register_new_resolver("eq", lambda x, y: x.lower()==y.lower())
OmegaConf.register_new_resolver("contains", lambda x, y: x.lower() in y.lower())
OmegaConf.register_new_resolver("if", lambda pred, a, b: a if pred else b)
OmegaConf.register_new_resolver("arange", lambda start,end,delta: list(np.arange(start, end + 1e-3, delta)))
OmegaConf.register_new_resolver("resolve_default", lambda default, arg: default if arg is None else arg)
@hydra.main(config_name="config", config_path="conf/")
def run(cfg: DictConfig):
    pass

if __name__ == "__main__":
    run()