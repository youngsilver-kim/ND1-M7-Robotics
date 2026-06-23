# 실습 3. 경로 계획 + 특이점 회피 전체 코드

import numpy as np
import matplotlib.pyplot as plt
import os

os.makedirs("results", exist_ok=True)

# 3DOF 로봇 링크 길이
L = np.array([1.0, 0.8, 0.6])


# 1. 3DOF 순기구학
def fk_3dof(theta):
    x, y = 0.0, 0.0
    angle = 0.0
    points = [[0.0, 0.0]]

    for i in range(3):
        angle += theta[i]
        x += L[i] * np.cos(angle)
        y += L[i] * np.sin(angle)
        points.append([x, y])

    return np.array(points)


# 2. 3DOF Jacobian 계산
def jacobian_3dof(theta):
    J = np.zeros((2, 3))

    for i in range(3):
        sx, sy = 0.0, 0.0

        for j in range(i, 3):
            angle = np.sum(theta[:j+1])
            sx += -L[j] * np.sin(angle)
            sy +=  L[j] * np.cos(angle)

        J[0, i] = sx
        J[1, i] = sy

    return J


# 3. Damped Least Squares 기반 수치 IK
def ik_dls(target, theta_init=None, lam=0.05, max_iter=300, tol=1e-6):
    if theta_init is None:
        theta = np.zeros(3)
    else:
        theta = np.array(theta_init, dtype=float)

    errors = []

    for _ in range(max_iter):
        points = fk_3dof(theta)
        current_pos = points[-1]

        error_vec = target - current_pos
        error_norm = np.linalg.norm(error_vec)
        errors.append(error_norm)

        if error_norm < tol:
            break

        J = jacobian_3dof(theta)

        # DLS 공식: Δθ = Jᵀ(JJᵀ + λ²I)⁻¹e
        dtheta = J.T @ np.linalg.inv(J @ J.T + (lam ** 2) * np.eye(2)) @ error_vec

        theta += dtheta

    return theta, errors


# 4. Manipulability 계산
def manipulability(theta):
    J = jacobian_3dof(theta)
    value = np.linalg.det(J @ J.T)

    if value < 0:
        value = 0

    return np.sqrt(value)


# 5. Waypoint 정의
waypoints = np.array([
    [1.5, 0.3],
    [1.3, 0.9],
    [0.9, 1.2],
    [1.6, 0.7]
])


# 6. Waypoint 사이 경로 생성 및 IK 수행
path = []
joint_path = []
manipulability_values = []

theta_prev = np.array([0.3, 0.3, -0.2])

for i in range(len(waypoints) - 1):
    start = waypoints[i]
    end = waypoints[i + 1]

    for alpha in np.linspace(0, 1, 40):
        target = (1 - alpha) * start + alpha * end

        theta, errors = ik_dls(target, theta_prev)

        theta_prev = theta

        end_effector = fk_3dof(theta)[-1]

        path.append(end_effector)
        joint_path.append(theta)
        manipulability_values.append(manipulability(theta))


path = np.array(path)
joint_path = np.array(joint_path)
manipulability_values = np.array(manipulability_values)


# 7. 경로 시각화 저장
plt.figure(figsize=(7, 7))
plt.plot(path[:, 0], path[:, 1], marker=".", label="Generated Path")
plt.scatter(waypoints[:, 0], waypoints[:, 1], marker="x", s=100, label="Waypoints")

for i, point in enumerate(waypoints):
    plt.text(point[0] + 0.03, point[1] + 0.03, f"W{i+1}")

plt.xlim(-0.5, 2.5)
plt.ylim(-0.5, 2.5)
plt.gca().set_aspect("equal")
plt.grid(True)
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("M7 Lab3 Path Planning Result")
plt.legend()
plt.savefig("results/M7_lab3_path.png", dpi=150)
plt.show()


# 8. Manipulability 그래프 저장
plt.figure(figsize=(8, 4))
plt.plot(manipulability_values)
plt.grid(True)
plt.xlabel("Path Step")
plt.ylabel("Manipulability")
plt.title("M7 Lab3 Manipulability Along Path")
plt.savefig("results/M7_lab3_manipulability.png", dpi=150)
plt.show()


# 9. 마지막 로봇 자세 시각화 저장
final_theta = joint_path[-1]
final_points = fk_3dof(final_theta)

plt.figure(figsize=(7, 7))
plt.plot(final_points[:, 0], final_points[:, 1], marker="o", linewidth=3, label="Robot Arm")
plt.scatter(waypoints[:, 0], waypoints[:, 1], marker="x", s=100, label="Waypoints")
plt.scatter(path[:, 0], path[:, 1], s=10, label="Path")

plt.xlim(-0.5, 2.5)
plt.ylim(-0.5, 2.5)
plt.gca().set_aspect("equal")
plt.grid(True)
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("M7 Lab3 Final Robot Pose")
plt.legend()
plt.savefig("results/M7_lab3_workspace.png", dpi=150)
plt.show()


# 10. 결과 출력
print("실습 3 실행 완료")
print("총 경로 점 개수:", len(path))
print("최소 Manipulability:", np.min(manipulability_values))
print("최대 Manipulability:", np.max(manipulability_values))
print("평균 Manipulability:", np.mean(manipulability_values))

if np.min(manipulability_values) < 0.05:
    print("특이점 위험: 있음")
else:
    print("특이점 위험: 낮음")

print("저장된 파일:")
print("results/M7_lab3_path.png")
print("results/M7_lab3_manipulability.png")
print("results/M7_lab3_workspace.png")