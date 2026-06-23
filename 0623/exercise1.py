# 실습 1. 2DOF 해석적 IK
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

L1, L2 = 1.0, 1.0

def fk_2dof(theta1, theta2):
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    return np.array([[0, 0], [x1, y1], [x2, y2]])

def ik_2dof(x, y, elbow="up"):
    r2 = x**2 + y**2
    c2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    c2 = np.clip(c2, -1.0, 1.0)

    if elbow == "up":
        theta2 = np.arccos(c2)
    else:
        theta2 = -np.arccos(c2)

    theta1 = np.arctan2(y, x) - np.arctan2(L2*np.sin(theta2), L1 + L2*np.cos(theta2))
    return theta1, theta2

def plot_robot(points, target, title, filename):
    plt.figure()
    plt.plot(points[:,0], points[:,1], marker="o")
    plt.scatter(target[0], target[1], marker="x")
    plt.xlim(-2.2, 2.2)
    plt.ylim(-2.2, 2.2)
    plt.gca().set_aspect("equal")
    plt.grid(True)
    plt.title(title)
    plt.savefig(f"results/{filename}", dpi=150)
    plt.show()

target = np.array([1.2, 0.8])

for mode in ["up", "down"]:
    t1, t2 = ik_2dof(target[0], target[1], mode)
    points = fk_2dof(t1, t2)
    plot_robot(points, target, f"2DOF IK - Elbow {mode}", f"M7_lab1_solution_{mode}.png")

targets = []
errors = []

for x in np.linspace(0.4, 1.6, 4):
    for y in np.linspace(0.2, 1.4, 4):
        t1, t2 = ik_2dof(x, y, "up")
        end = fk_2dof(t1, t2)[-1]
        err = np.linalg.norm(end - np.array([x, y]))
        targets.append([x, y])
        errors.append(err)

targets = np.array(targets)

plt.figure()
plt.scatter(targets[:,0], targets[:,1], label="Target")
plt.title("2DOF IK Validation")
plt.grid(True)
plt.gca().set_aspect("equal")
plt.savefig("results/M7_lab1_validation.png", dpi=150)
plt.show()

print("평균 오차:", np.mean(errors))
print("최대 오차:", np.max(errors))