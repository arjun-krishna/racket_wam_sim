from dataclasses import dataclass

import pybullet as p
import pybullet_utils.bullet_client as bc
import numpy as np


@dataclass
class BallPhysics:
    mass: float = 1

    # coefficients obtained from: https://arxiv.org/pdf/2109.03100.pdf
    drag_coef: float = 0.4
    air_density: float = 1.29
    lift_coef: float = 0.6

    # sphere shell properties
    radius_1: float = 0.0196
    radius_2: float = 0.02

    def drag_force(self, v) -> float:
        A = np.pi * self.radius_2 * self.radius_2
        return -0.5 * self.drag_coef * self.air_density * A * np.linalg.norm(v) * v

    def magnus_force(self, v, w) -> float:
        A = np.pi * self.radius_2 * self.radius_2
        return (
            0.5 * self.lift_coef * self.air_density * A * self.radius_2 * np.cross(w, v)
        )


@dataclass
class TablePhysics:
    # fixed class -- symmetric in XY plane
    # note URDF mesh defines these properties at object scale 1
    extent_x: float = 1.370
    extent_y: float = 0.762
    z: float = 0.74

    # needs to be altered in URDF if changed
    restitution: float = 0.93

    collision_tol: float = 1e-3

    def check_collision(self, pos: np.array, radius: float) -> bool:
        return (
            (-self.extent_x <= pos[0] <= self.extent_x)
            and (-self.extent_y <= pos[1] <= self.extent_y)
            and (pos[2] < self.z[2] + radius + self.collision_tol)
        )


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
    pos: np.array


@dataclass
class SimConfig:
    arm: WAMConfig
    ball: BallPhysics
    launcher: BallLauncher
