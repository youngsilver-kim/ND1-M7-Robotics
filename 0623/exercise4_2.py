# 실습 4-2. 2D 등속 운동 칼만 필터
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

class KalmanFilter2D:
    def __init__(self, dt=1.0, Q=0.01, R=0.5):
        self.dt = dt
        self.x = np.array([[0], [0], [1], [1]], dtype=float)

        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        self.P = np.eye(4) * 100
        self.Q = np.eye(4) * Q
        self.R = np.eye(2) * R

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        z = z.reshape(2, 1)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return self.x.flatten()

true_positions = []
measurements = []
estimates = []

velocity = np.array([1.0, 0.5])
position = np.array([0.0, 0.0])

kf2 = KalmanFilter2D(Q=0.01, R=0.5)

for _ in range(50):
    position = position + velocity
    z = position + np.random.normal(0, 0.7, 2)

    kf2.predict()
    est = kf2.update(z)

    true_positions.append(position.copy())
    measurements.append(z)
    estimates.append(est[:2])

true_positions = np.array(true_positions)
measurements = np.array(measurements)
estimates = np.array(estimates)

rmse_2d = np.sqrt(np.mean(np.sum((estimates - true_positions) ** 2, axis=1)))

plt.figure()
plt.plot(true_positions[:,0], true_positions[:,1], label="True")
plt.scatter(measurements[:,0], measurements[:,1], s=10, label="Measurement")
plt.plot(estimates[:,0], estimates[:,1], label="Kalman Estimate")
plt.grid(True)
plt.legend()
plt.title(f"2D Kalman Filter / RMSE={rmse_2d:.4f}")
plt.savefig("results/M7_lab4_kalman_2d.png", dpi=150)
plt.show()

print("2D RMSE:", rmse_2d)