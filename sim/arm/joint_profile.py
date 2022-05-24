import matplotlib.pyplot as plt
import numpy as np

def create_jp_plot(num_joints: int):
    fig, axes = plt.subplots(num_joints, 3)
    for i in range(num_joints):
        for j, n in enumerate(["pos", "vel", "acc"]):
            axes[i,j].set_title(f"joint{i}_{n}")
    for ax in axes.flat:
        ax.set(xlabel='t')
        ax.label_outer()
    fig.tight_layout()
    plt.show(block=False)
    return fig, axes

def update_jp_plot(fig, axes, joint_pos: np.array, joint_vel: np.array, joint_acc: np.array):
    for i in range(len(joint_pos)):
        for j, v in enumerate([joint_pos[i], joint_vel[i], joint_acc[i]]):
            axes[i,j].set()