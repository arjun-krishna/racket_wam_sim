# racket_wam_sim
Tennis/Table-Tennis simulation of Barrett WAM Arm in PyBullet and IsaacSim for LfD and RL research


# Table Tennis

## Setup

1. Install Omniverse and IsaacSim: [guide](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html)
2. Setup isaac-sim conda environment: [guide](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_python.html)
> **note:** locate the setup environment.yaml etc., in ~/.local/share/ov/pkg/isaac_sim-*/

> **note:** At time of setup we used isaac_sim-2022.1.0
3. run `pip install -r requirements.txt`

## Limitations
- PyBullet (sim-inaccuracies):
  - Doesn't consider racket multi-surface properties (spin vs no-spin surface)