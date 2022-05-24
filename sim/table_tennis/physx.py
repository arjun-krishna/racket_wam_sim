import numpy as np

from sim.table_tennis.config import BallPhysics, TablePhysics


def drag_force(ball: BallPhysics, v) -> float:
    A = np.pi * ball.radius_2 * ball.radius_2
    return -0.5 * ball.drag_coef * ball.air_density * A * np.linalg.norm(v) * v


def magnus_force(ball: BallPhysics, v, w) -> float:
    A = np.pi * ball.radius_2 * ball.radius_2
    return 0.5 * ball.lift_coef * ball.air_density * A * ball.radius_2 * np.cross(w, v)


def check_collision(table: TablePhysics, ball: BallPhysics, ball_pos: np.array) -> bool:
    return (
        (-table.extent_x <= ball_pos[0] <= table.extent_x)
        and (-table.extent_y <= ball_pos[1] <= table.extent_y)
        and (ball_pos[2] < table.z[2] + ball.radius_2 + table.collision_tol)
    )
