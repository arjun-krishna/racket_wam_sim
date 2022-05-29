from typing import Tuple
import numpy as np
from sim.utils.rigid_body import HollowSphere
from sim.table_tennis.config import BallPhysics, TablePhysics
from sim.table_tennis.physx import check_collision, drag_force, magnus_force
from sim.utils.quat import rotation_matrix_from_quat, quat_mult


class BallTraj:
    def __init__(
        self, table: TablePhysics, ball: BallPhysics, gravity: float = 9.8
    ) -> None:
        self.ball = ball
        self.table = table
        self.gravity = gravity

        # holds ball state for physics simulation of the trajectory
        self.rb: HollowSphere = HollowSphere(
            mass=ball.mass, radius_1=ball.radius_1, radius_2=ball.radius_2
        )

    def handle_collision(self):
        # TODO: handle ground plane collision
        if check_collision(self.table, self.ball, self.rb.position):
            # handle penetrating / seperating collision contact
            r = np.zeros(3)
            r[2] = self.table.z - self.rb.position[2]

            rot_m = rotation_matrix_from_quat(self.rb.orientation)
            inv_inertia = rot_m @ self.rb.inverse_inertia @ rot_m.T
            w = inv_inertia @ self.rb.angular_momentum
            v = np.cross(w, r) + self.rb.linear_momentum / self.rb.mass
            inv_m = 1 / self.rb.mass

            v_pre = v[2]
            if v_pre > 0:  # seperating contact
                return

            # impulse
            j = (-(1 + self.restitution) * v_pre) / (inv_m)
            J = j * np.array([0, 0, 1])

            self.rb.linear_momentum += J
            # angular momentum doesn't change (consider friction?)

    def get_traj(
        self,
        position: np.array,
        orientation: np.array,
        linear_velocity: np.array,
        angular_velocity: np.array,
        t: float = 2,
        sim_dt: float = 1 / 240,
    ):
        traj = []
        steps = int(t / sim_dt)

        self.rb.position = position
        self.rb.orientation = orientation
        self.rb.linear_momentum = self.rb.mass * linear_velocity
        self.rb.angular_momentum = self.rb.inertia * angular_velocity

        for _ in range(steps):
            traj.append(self.rb.position)
            self.integrate_eom(sim_dt)
        return traj

    def integrate_eom(self, dt: float):
        # explicit euler integration of Eqns of Motion
        # TODO consider midpoint method of integration
        v = self.rb.linear_momentum / self.rb.mass
        self.rb.position += dt * v

        rot_m = rotation_matrix_from_quat(self.rb.orientation)
        inertia_inverse = rot_m @ self.rb.inverse_inertia @ rot_m.T
        w = inertia_inverse @ self.rb.angular_momentum
        dq = 0.5 * quat_mult(np.hstack([w, 0]), self.rb.orientation)
        self.rb.orientation += dt * dq
        self.rb.orientation /= np.linalg.norm(self.rb.orientation)

        self.rb.linear_momentum += dt * (
            self.rb.mass * np.array([0, 0, -self.gravity])
            + drag_force(self.ball, v)
            + +magnus_force(self.ball, v, w)
        )

        self.rb.angular_momentum += 0  # forces act on COM
