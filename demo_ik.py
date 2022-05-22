"""
Simulated Demonstrations with the help of Inverse Kinematics
"""
import hydra
from omegaconf import DictConfig


@hydra.main(config_path="conf", config_name="demo_ik")
def main(cfg: DictConfig):
    pass


if __name__ == "__main__":
    main()
