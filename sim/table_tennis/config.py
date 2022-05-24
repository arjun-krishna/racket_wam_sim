from dataclasses import MISSING, dataclass

import pybullet as p
import pybullet_utils.bullet_client as bc
import numpy as np


@dataclass
class BallPhysics:
    mass: float = 0.0027

    # coefficients obtained from: https://arxiv.org/pdf/2109.03100.pdf
    drag_coef: float = 0.4
    air_density: float = 1.29
    lift_coef: float = 0.6

    # sphere shell properties
    radius_1: float = 0.0196
    radius_2: float = 0.02


@dataclass
class TablePhysics:
    # fixed class -- symmetric in XY plane
    # note URDF mesh defines these properties at object scale 1
    extent_x: float = 1.370
    extent_y: float = 0.762
    z: float = 0.74

    # needs to be altered in URDF if changed (TODO make it programmatic)
    restitution: float = 0.9

    collision_tol: float = 1e-3


@dataclass
class BallLauncher:
    pos: np.array
    vel: np.array
    ang_vel: np.array

    def __init__(self, client: bc.BulletClient):
        is_connected, connection_method = client.getConnectionInfo()
        if is_connected and (connection_method is p.GUI):
            pass

    def update_launcher(self, pos: np.array, vel: np.array, ang_vel: np.array):
        self.pos = pos
        self.vel = vel
        self.ang_vel = ang_vel


@dataclass
class WAMConfig:
    pos: np.array = np.array([0, -1.95, 3])
    orientation: np.array = np.array([1, 0, 0, 0])


@dataclass
class SimConfig:
    arm: WAMConfig = MISSING
    ball: BallPhysics = MISSING
    table: TablePhysics = MISSING
    launcher: BallLauncher = MISSING

    gravity: float = 9.8
