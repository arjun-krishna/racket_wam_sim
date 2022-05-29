import matplotlib.pyplot as plt
import numpy as np


def create_jp_plot(
    num_joints: int,
    T: int = 3,
    lower_limits: np.array = None,
    upper_limits: np.array = None,
):
    x = np.arange(T)
    y = np.zeros(T)
    data = {}
    fig, axes = plt.subplots(num_joints, 3)
    for i in range(num_joints):
        for j, n in enumerate(["pos", "vel", "acc"]):
            axes[i, j].set_title(f"joint{i}_{n}")
            (line,) = axes[i, j].plot(x, y)
            axes[i, j].set(xlabel="t")
            axes[i, j].label_outer()
            data[(i, j)] = line
            axes[i, j].set_ylim(lower_limits[i, j], upper_limits[i, j])

    fig.tight_layout()
    plt.show(block=False)
    plt_obj = {"fig": fig, "data": data}
    return plt_obj


def update_jp_plot(
    plt_obj, joint_pos: np.array, joint_vel: np.array, joint_acc: np.array
):
    fig, data = plt_obj["fig"], plt_obj["data"]
    for i in range(len(joint_pos)):
        for j, v in enumerate([joint_pos[i], joint_vel[i], joint_acc[i]]):
            line = data[(i, j)]
            y = line.get_ydata().copy()
            y[:-1] = y[1:]
            y[-1] = v
            line.set_ydata(y)
    fig.canvas.draw()
    fig.canvas.flush_events()
