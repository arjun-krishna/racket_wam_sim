from typing import List
import matplotlib.pyplot as plt
import numpy as np


def create_jp_plot(
    num_joints: int,
    joint_names: List[str] = None,
    T: int = 240,  # 1 second
    ll: np.array = None,  # lower limits
    ul: np.array = None,  # upper limits
):
    x = np.arange(T)
    y = np.zeros(T)
    data = {}
    fig, axes = plt.subplots(num_joints, 3)
    for i in range(num_joints):
        for j, n in enumerate(["pos", "vel", "torq"]):
            if joint_names is not None:
                axes[i, j].set_title(f"{joint_names[i]}/{n}")
            else:
                axes[i, j].set_title(f"joint{i}_{n}")
            (line,) = axes[i, j].plot(x, y)
            axes[i, j].set(xlabel="t")
            # axes[i, j].label_outer()
            data[(i, j)] = line
            if ll is not None and ul is not None:
                axes[i, j].set_ylim(ll[i, j], ul[i, j])
            axes[i, j].set_xticks([])

    # fig.tight_layout()
    plt.show(block=False)
    plt_obj = {"fig": fig, "data": data}
    return plt_obj


def update_jp_plot(
    plt_obj, joint_pos: np.array, joint_vel: np.array, joint_torq: np.array,
):
    fig, data = plt_obj["fig"], plt_obj["data"]
    for i in range(len(joint_pos)):
        for j, v in enumerate([joint_pos[i], joint_vel[i], joint_torq[i]]):
            line = data[(i, j)]
            y = line.get_ydata().copy()
            y[:-1] = y[1:]
            y[-1] = v
            line.set_ydata(y)
    fig.canvas.draw()
    fig.canvas.flush_events()
