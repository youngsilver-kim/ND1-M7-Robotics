# 실습 2. 3DOF 수치 IK + DLS
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

L = np.array([1.0, 0.8, 0.6])

def fk_3dof(theta):
    x, y = 0, 0
    angle = 0
    points = [[0, 0]]

    for i in range(3):
        angle += theta[i]
        x += L[i] * np.cos(angle)
        y += L[i] * np.sin(angle)
        points.append([x, y])

    return np.array(points)

def jacobian_3dof(theta):
    J = np.zeros((2, 3))

    for i in range(3):
        sx, sy = 0, 0
        for j in range(i, 3):
            angle = np.sum(theta[:j+1])
            sx += -L[j] * np.sin(angle)
            sy +=  L[j] * np.cos(angle)
        J[0, i] = sx
        J[1, i] = sy

    return J

def ik_dls(target, theta_init=None, lam=0.05, max_iter=200, tol=1e-6):
    if theta_init is None:
        theta = np.zeros(3)
    else:
        theta = theta_init.astype(float)

    errors = []

    for _ in range(max_iter):
        pos = fk_3dof(theta)[-1]
        e = target - pos
        err = np.linalg.norm(e)
        errors.append(err)

        if err < tol:
            break

        J = jacobian_3dof(theta)
        dtheta = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(2)) @ e
        theta += dtheta

    return theta, errors

targets = [
    np.array([1.6, 0.6]),
    np.array([1.2, 1.2]),
    np.array([0.8, 1.5])
]

for idx, target in enumerate(targets, start=1):
    theta, errors = ik_dls(target)
    points = fk_3dof(theta)

    plt.figure()
    plt.plot(points[:,0], points[:,1], marker="o")
    plt.scatter(target[0], target[1], marker="x")
    plt.xlim(-2.5, 2.5)
    plt.ylim(-2.5, 2.5)
    plt.gca().set_aspect("equal")
    plt.grid(True)
    plt.title(f"3DOF IK Pose {idx}")
    plt.savefig(f"results/M7_lab2_pose{idx}.png", dpi=150)
    plt.show()

    print(f"Pose {idx} 최종 오차:", errors[-1])

plt.figure()
for target in targets:
    _, errors = ik_dls(target)
    plt.plot(errors)

plt.yscale("log")
plt.grid(True)
plt.title("DLS IK Convergence")
plt.xlabel("Iteration")
plt.ylabel("Position Error")
plt.savefig("results/M7_lab2_convergence.png", dpi=150)
plt.show()