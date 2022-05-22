from re import L
import numpy as np


class RigidBody:
    def __init__(
        self,
        position=np.array([0, 0, 0]),
        orientation=np.array([0, 0, 0, 1]),  # [x,y,z,w]
        linear_momentum=np.array([0, 0, 0]),
        angular_momentum=np.array([0, 0, 0]),
        mass=1,
        fixed=False,
    ):
        self.position = np.array(position, dtype=np.float64)
        self.orientation = np.array(orientation, dtype=np.float64)
        self.linear_momentum = np.array(linear_momentum, dtype=np.float64)
        self.angular_momentum = np.array(angular_momentum, dtype=np.float64)
        self.mass = mass
        self.fixed = fixed
        self.inertia = None
        self.inverse_inertia = None


class Sphere(RigidBody):
    def __init__(
        self,
        position=np.array([0, 0, 0]),
        orientation=np.array([0, 0, 0, 1]),
        linear_momentum=np.array([0, 0, 0]),
        angular_momentum=np.array([0, 0, 0]),
        mass=1,
        fixed=False,
        radius=1,
    ):
        super().__init__(
            position, orientation, linear_momentum, angular_momentum, mass, fixed
        )
        self.inertia = 0.4 * mass * radius * radius * np.eye(3)
        self.inverse_inertia = np.eye(3) * 2.5 / mass / radius / radius
        self.radius = radius


class HollowSphere(RigidBody):
    def __init__(
        self,
        position=np.array([0, 0, 0]),
        orientation=np.array([0, 0, 0, 1]),
        linear_momentum=np.array([0, 0, 0]),
        angular_momentum=np.array([0, 0, 0]),
        mass=1,
        fixed=False,
        radius_1=0.95,
        radius_2=1.0,
    ):
        super().__init__(
            position, orientation, linear_momentum, angular_momentum, mass, fixed
        )
        assert radius_2 > radius_1, "radius_2 should be greater than radius_1"
        self.inertia = (
            0.4
            * mass
            * ((radius_2 ** 5 - radius_1 ** 5) / (radius_2 ** 3 - radius_1 ** 3))
            * np.eye(3)
        )
        self.inverse_inertia = (
            np.eye(3)
            * (2.5 / mass)
            * ((radius_2 ** 3 - radius_1 ** 3) / (radius_2 ** 5 - radius_1 ** 5))
        )
        self.radius_1 = radius_1
        self.radius_2 = radius_2
