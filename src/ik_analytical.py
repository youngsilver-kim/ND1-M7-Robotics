"""해석적 역기구학 — 2DOF 평면 로봇.

코사인 법칙으로 IK를 풀어 두 해 (elbow up/down)를 반환.
"""
import numpy as np


def ik_2dof(x, y, L1=0.3, L2=0.3, tol=1e-9):
    """2DOF 평면 로봇 해석적 IK.

    Parameters
    ----------
    x, y : float [m]
        목표 말단부 위치
    L1, L2 : float [m], default 0.3, 0.3
        링크 길이
    tol : float, default 1e-9
        부동소수점 한계 보정 임계값

    Returns
    -------
    solutions : list of tuple or None
        [(θ1_up, θ2_up), (θ1_down, θ2_down)] 두 해 [rad]
        도달 불가능하면 None

    Examples
    --------
    >>> sols = ik_2dof(0.4, 0.2)
    >>> # sols[0] = elbow up, sols[1] = elbow down

    Notes
    -----
    np.arccos 의 부동소수점 함정 회피를 위해 np.clip 적용 필수.
    """
    L1, L2 = float(L1), float(L2)
    r2 = x * x + y * y
    r = np.sqrt(r2)

    # 도달 가능성 검사
    if r > L1 + L2 + tol or r < abs(L1 - L2) - tol:
        return None

    # 코사인 법칙: cos(θ2) = (r² − L1² − L2²) / (2·L1·L2)
    cos_t2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    # ⚠ 부동소수점 함정 회피
    cos_t2 = np.clip(cos_t2, -1.0, 1.0)

    # 두 해
    t2_up = np.arccos(cos_t2)        # elbow up (θ2 > 0)
    t2_down = -t2_up                  # elbow down (θ2 < 0)

    solutions = []
    for t2 in (t2_up, t2_down):
        # θ1 = atan2(y,x) − atan2(L2·sin(θ2), L1 + L2·cos(θ2))
        t1 = np.arctan2(y, x) - np.arctan2(
            L2 * np.sin(t2),
            L1 + L2 * np.cos(t2)
        )
        solutions.append((float(t1), float(t2)))

    return solutions


if __name__ == '__main__':
    print('=== ik_2dof 단위 테스트 ===')

    # 1. 도달 가능한 점
    sols = ik_2dof(0.4, 0.2)
    assert sols is not None
    assert len(sols) == 2
    print(f'✓ (0.4, 0.2) 두 해:')
    for label, (t1, t2) in zip(['elbow up  ', 'elbow down'], sols):
        print(f'    {label}: θ1={np.degrees(t1):6.2f}°, θ2={np.degrees(t2):6.2f}°')

    # 2. FK로 검증
    from .robot_arm_2dof import RobotArm2DOF
    robot = RobotArm2DOF()
    for t1, t2 in sols:
        pos, _ = robot.fk([t1, t2])
        err = np.linalg.norm(np.array(pos[-1][:2]) - np.array([0.4, 0.2]))
        assert err < 1e-6, f'FK 검증 실패: err={err:.2e}'
    print(f'✓ FK로 검증: 두 해 모두 (0.4, 0.2) 도달 (오차 < 1e-6)')

    # 3. 도달 불가능
    sols = ik_2dof(1.0, 0)   # 0.6m 한계 초과
    assert sols is None
    print('✓ 도달 불가능 점 (1.0, 0): None 반환')

    # 4. 경계 (정확히 r_max)
    sols = ik_2dof(0.6, 0)
    assert sols is not None
    print(f'✓ 경계점 (0.6, 0): θ2 ≈ {np.degrees(sols[0][1]):.2f}° (= 0°)')

    print('\n모든 테스트 통과')
