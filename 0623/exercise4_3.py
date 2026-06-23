# 실습 4-3. Q·R 민감도 분석 전체 코드
# KalmanFilter2D 정의 포함

import numpy as np
import pandas as pd
import os

os.makedirs("results", exist_ok=True)


class KalmanFilter2D:
    def __init__(self, dt=1.0, Q=0.01, R=0.5):
        self.dt = dt

        # 상태 벡터: [x, y, vx, vy]
        self.x = np.array([[0], [0], [1], [0.5]], dtype=float)

        # 상태 전이 행렬
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # 측정 행렬: 위치 x, y만 측정
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # 오차 공분산
        self.P = np.eye(4) * 100

        # 프로세스 노이즈
        self.Q = np.eye(4) * Q

        # 측정 노이즈
        self.R = np.eye(2) * R

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        z = np.array(z).reshape(2, 1)

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        return self.x.flatten()


# 재현 가능하도록 시드 고정
np.random.seed(42)

summary = []

Q_values = [0.001, 0.01, 0.1, 1.0, 10.0]
R_values = [0.1, 0.5, 1.0, 2.0, 5.0]

for Q in Q_values:
    for R in R_values:
        kf2 = KalmanFilter2D(Q=Q, R=R)

        true_positions = []
        estimated_positions = []

        position = np.array([0.0, 0.0])
        velocity = np.array([1.0, 0.5])

        for _ in range(50):
            # 실제 등속 운동
            position = position + velocity

            # 센서 노이즈가 포함된 위치 측정값
            measurement = position + np.random.normal(0, 0.7, 2)

            # 칼만 필터 예측 및 보정
            kf2.predict()
            estimate = kf2.update(measurement)

            true_positions.append(position.copy())
            estimated_positions.append(estimate[:2])

        true_positions = np.array(true_positions)
        estimated_positions = np.array(estimated_positions)

        rmse = np.sqrt(
            np.mean(
                np.sum((estimated_positions - true_positions) ** 2, axis=1)
            )
        )

        summary.append([Q, R, rmse])


df = pd.DataFrame(summary, columns=["Q", "R", "RMSE"])
df = df.sort_values(by="RMSE").reset_index(drop=True)

df.to_csv("results/M7_lab4_summary.csv", index=False)

print("실습 4-3 실행 완료")
print("Q·R 민감도 분석 결과 저장 완료")
print("저장 파일: results/M7_lab4_summary.csv")
print()
print(df)