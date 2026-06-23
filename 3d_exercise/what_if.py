import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython.display import HTML
import os

os.makedirs("results", exist_ok=True)

L1 = 1.0
L2 = 0.8
L3 = 0.6

def rot_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1]
    ])

def rot_y(theta):
    return np.array([
        [ np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def fk(theta):
    t1, t2, t3 = theta

    p0 = np.array([0, 0, 0])

    R0 = rot_z(t1)
    R1 = R0 @ rot_y(t2)
    R2 = R1 @ rot_y(t3)

    p1 = p0 + np.array([0, 0, L1])
    p2 = p1 + R1 @ np.array([L2, 0, 0])
    p3 = p2 + R2 @ np.array([L3, 0, 0])

    return np.array([p0, p1, p2, p3])

frames = 150
theta_list = []

for t in np.linspace(0, 2*np.pi, frames):
    theta1 = t
    theta2 = 0.7 * np.sin(t)
    theta3 = 0.8 * np.cos(t)
    theta_list.append([theta1, theta2, theta3])

theta_list = np.array(theta_list)

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(0, 2.5)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3DOF Robot Arm 3D Simulation")

line, = ax.plot([], [], [], "o-", linewidth=4)
trace_line, = ax.plot([], [], [], "--")

trace_x = []
trace_y = []
trace_z = []

def update(frame):
    pts = fk(theta_list[frame])

    x = pts[:, 0]
    y = pts[:, 1]
    z = pts[:, 2]

    line.set_data(x, y)
    line.set_3d_properties(z)

    trace_x.append(x[-1])
    trace_y.append(y[-1])
    trace_z.append(z[-1])

    trace_line.set_data(trace_x, trace_y)
    trace_line.set_3d_properties(trace_z)

    return line, trace_line

ani = FuncAnimation(
    fig,
    update,
    frames=frames,
    interval=50,
    blit=False
)

ani.save("results/M7_3d_robot_simulation.gif", writer="pillow", fps=20)

print("저장 완료: results/M7_3d_robot_simulation.gif")

HTML(ani.to_jshtml())