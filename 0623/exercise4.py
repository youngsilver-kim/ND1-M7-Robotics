# 실습 4. 센서 + 칼만 필터
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

class KalmanFilter1D:
    def __init__(self, x0=0.0, P0=100.0, Q=0.001, R=1.0):
        self.x = x0
        self.P = P0
        self.Q = Q
        self.R = R

    def predict(self):
        self.P += self.Q

    def update(self, z):
        K = self.P / (self.P + self.R)
        self.x = self.x + K * (z - self.x)
        self.P = (1 - K) * self.P
        return self.x, K

true_pos = 5.0
measurements = true_pos + np.random.normal(0, 1.0, 30)

kf = KalmanFilter1D()
estimates = []
gains = []

for z in measurements:
    kf.predict()
    x, K = kf.update(z)
    estimates.append(x)
    gains.append(K)

rmse_1d = np.sqrt(np.mean((np.array(estimates) - true_pos) ** 2))

plt.figure()
plt.plot(measurements, marker=".", label="Measurement")
plt.plot(estimates, marker="o", label="Kalman Estimate")
plt.axhline(true_pos, linestyle="--", label="True Position")
plt.grid(True)
plt.legend()
plt.title(f"1D Kalman Filter / RMSE={rmse_1d:.4f}")
plt.savefig("results/M7_lab4_kalman_1d.png", dpi=150)
plt.show()

print("1D RMSE:", rmse_1d)