"""칼만 필터 + 센서 시뮬레이터 단위 테스트.

실행: python -m pytest tests/test_kalman.py -v
"""
import pytest
import sys, os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.kalman_filter import KalmanFilter1D, KalmanFilter2D
from src.sensor_simulator import (pinhole_camera_project, lidar_distance,
                                    imu_accel, simulate_constant_velocity_2d)
import numpy as np


# ─── KalmanFilter1D ─────────────────────────────────────
def test_kf1d_converges_to_truth():
    """1D 칼만 필터가 진짜 값에 수렴."""
    TRUE_POS = 5.0
    rng = np.random.default_rng(42)
    kf = KalmanFilter1D(x0=0.0, P0=100.0, Q=0.001, R=1.0)
    for _ in range(50):
        z = TRUE_POS + rng.normal(0, 1.0)
        kf.predict()
        kf.update(z)
    # 50번 측정 후 진짜 값에 0.5m 이내 수렴
    assert abs(kf.x - TRUE_POS) < 0.5, f'수렴 실패: {kf.x:.3f}m (진짜 5.0)'


def test_kf1d_K_decreases():
    """칼만 이득 K가 감소 (불확실성 감소)."""
    kf = KalmanFilter1D(x0=0.0, P0=100.0, Q=0.001, R=1.0)
    Ks = []
    for _ in range(20):
        kf.predict()
        _, K = kf.update(5.0)
        Ks.append(K)
    # 처음 K가 마지막 K보다 큼 (불확실성 줄어듦)
    assert Ks[0] > Ks[-1], f'K 감소 안 함: K0={Ks[0]:.3f}, K-1={Ks[-1]:.3f}'


# ─── KalmanFilter2D ─────────────────────────────────────
def test_kf2d_state_shape():
    """2D 칼만 필터 상태 4차원."""
    kf = KalmanFilter2D()
    assert kf.x.shape == (4,)
    assert kf.P.shape == (4, 4)


def test_kf2d_position_tracking():
    """2D 등속 운동 추적 RMSE < 0.5m."""
    truth, meas = simulate_constant_velocity_2d(
        v=(1.0, 0.5), n_steps=50, dt=0.1, noise_std=0.5, seed=42
    )
    kf = KalmanFilter2D(dt=0.1, R_meas=0.25)
    estimates = []
    for z in meas:
        kf.predict()
        estimates.append(kf.update(z))
    estimates = np.array(estimates)
    rmse = np.sqrt(np.mean((estimates[:, :2] - truth) ** 2))
    assert rmse < 0.5, f'RMSE 너무 큼: {rmse:.3f}m'


def test_kf2d_velocity_estimation():
    """2D 칼만 필터가 보이지 않는 속도까지 추정."""
    truth, meas = simulate_constant_velocity_2d(
        v=(1.0, 0.5), n_steps=80, dt=0.1, noise_std=0.5, seed=42
    )
    kf = KalmanFilter2D(dt=0.1, R_meas=0.25)
    for z in meas:
        kf.predict()
        kf.update(z)
    # 최종 속도 추정이 진짜 (1.0, 0.5)에 가까움 (오차 0.2 이내)
    assert abs(kf.x[2] - 1.0) < 0.2, f'vx 오차: {kf.x[2]:.3f} (진짜 1.0)'
    assert abs(kf.x[3] - 0.5) < 0.2, f'vy 오차: {kf.x[3]:.3f} (진짜 0.5)'


# ─── 센서 시뮬레이터 ─────────────────────────────────────
def test_pinhole_camera():
    """핀홀 카메라 — Z=1m 점 노이즈 없이 투영."""
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=float)
    # P_world = (0.1, 0.2, 1.0) → u = 500*0.1/1 + 320 = 370
    uv = pinhole_camera_project([0.1, 0.2, 1.0], K, noise_std=0)
    assert abs(uv[0] - 370) < 1e-6
    assert abs(uv[1] - 340) < 1e-6


def test_lidar_distance_no_noise():
    """라이다 — 노이즈 없으면 정확한 거리."""
    r = lidar_distance([3, 4, 0], noise_std=0)
    assert abs(r - 5.0) < 1e-6


def test_imu_accel_shape():
    """IMU 출력 3차원."""
    z = imu_accel([0, 0, 9.81], bias=(0, 0, 0), noise_std=0)
    assert z.shape == (3,)
    assert abs(z[2] - 9.81) < 1e-6


if __name__ == '__main__':
    test_kf1d_converges_to_truth();   print('✓ KF1D 수렴')
    test_kf1d_K_decreases();          print('✓ KF1D K 감소')
    test_kf2d_state_shape();          print('✓ KF2D 상태 4차원')
    test_kf2d_position_tracking();    print('✓ KF2D 2D 추적 RMSE<0.5')
    test_kf2d_velocity_estimation();  print('✓ KF2D 속도 추정')
    test_pinhole_camera();            print('✓ 핀홀 카메라')
    test_lidar_distance_no_noise();   print('✓ 라이다 거리')
    test_imu_accel_shape();           print('✓ IMU 형상')
    print('\n모든 Kalman 테스트 통과')
