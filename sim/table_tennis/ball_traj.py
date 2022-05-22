from typing import Tuple
import numpy as np
from sim.rigid_body import HollowSphere, RigidBody
from sim.utils import rotation_matrix_from_quat, quat_mult

class PingPongTraj:

    def __init__(self, plane: Tuple[float, float, float], restitution: float, gravity: float = 9.8) -> None:
        self.ball: RigidBody = HollowSphere()
        self.plane = plane
        self.restitution = restitution
        self.gravity = gravity

    def check_collision(self):
        return (-self.plane[0] <= self.ball.position[0] <= self.plane[0]) \
            and (-self.plane[1] <= self.ball.position[1] <= self.plane[1]) \
                and (self.ball.position[2] < self.plane[2] + 1e-6)

    def handle_collision(self):
        if self.check_collision():
            # handle penetrating / seperating collision contact
            point_contact = self.ball.position[2].copy(); point_contact[2] = self.plane[2]
            
            r = point_contact - self.ball.position
            rot_m = rotation_matrix_from_quat(self.ball.orientation)
            inv_inertia = rot_m @ self.ball.inverse_inertia @ rot_m.T
            w = inv_inertia @ self.ball.angular_momentum
            v = np.cross(w, r) + self.ball.linear_momentum / self.ball.mass
            inv_m = 1 / self.ball.mass

            v_pre = self.ball.velocity[2]
            if v_pre > 0:   # seperating contact
                return
            
            # impulse
            j = (-(1 + self.restitution)*v_pre) / (inv_m)
            J = j * np.array([0, 0, 1])

            self.ball.linear_momentum += J
            # angular momentum doesn't change (consider friction?)


    def get_traj(self, position, orientation, linear_velocity, angular_velocity):
        pass

    def integrate_eom(self, dt: float):
        self.ball.position += dt * (self.ball.linear_momentum / self.ball.mass)

        rot_m = rotation_matrix_from_quat(self.ball.orientation)
        inertia_inverse = rot_m @ self.ball.inverse_inertia @ rot_m.T
        w = inertia_inverse @ self.ball.angular_momentum
        dq = 0.5 * quat_mult(np.hstack([w, 0]), self.ball.orientation)
        self.ball.orientation += dt * dq
        self.ball.orientation /= np.linalg.norm(self.ball.orientation)

        self.ball.linear_momentum += dt * (
            self.ball.mass * np.array([0, 0, -self.gravity])
            + 1
        )

        self.ball.angular_momentum += 0