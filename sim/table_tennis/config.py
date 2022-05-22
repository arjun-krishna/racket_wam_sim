from dataclasses import dataclass

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

@dataclass
class Sim:
    arm_pos = 0
