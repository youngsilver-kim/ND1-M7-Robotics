"""IK (해석 + 수치) 단위 테스트.

실행: python -m pytest tests/test_ik.py -v
"""
import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.robot_arm_2dof import RobotArm2DOF
from src.robot_arm import RobotArm3DOF
from src.ik_analytical import ik_2dof
from src.ik_numerical import ik_dls, ik_dls_multi


def test_ik_2dof_two_solutions():
    """ik_2dof: 도달 가능한 점에서 두 해."""
    sols = ik_2dof(0.4, 0.2)
    assert sols is not None
    assert len(sols) == 2
    # elbow up: θ2 > 0, elbow down: θ2 < 0
    assert sols[0][1] > 0
    assert sols[1][1] < 0


def test_ik_2dof_fk_validation():
    """ik_2dof 결과를 FK로 검증."""
    robot = RobotArm2DOF()
    target = (0.5, 0.3)
    sols = ik_2dof(*target)
    assert sols is not None
    for t1, t2 in sols:
        pos, _ = robot.fk([t1, t2])
        err = np.linalg.norm(np.array(pos[-1][:2]) - np.array(target))
        assert err < 1e-6


def test_ik_2dof_unreachable():
    """도달 불가능한 점은 None."""
    assert ik_2dof(1.0, 0) is None     # r > 0.6 한계 초과
    assert ik_2dof(2.0, 2.0) is None


def test_ik_2dof_clip_safety():
    """np.arccos 부동소수점 함정 회피 (정확히 경계)."""
    sols = ik_2dof(0.6, 0)    # 정확히 r_max
    assert sols is not None
    # θ2 ≈ 0 (펴진 자세)
    assert abs(sols[0][1]) < 1e-6


def test_ik_dls_3_targets():
    """3DOF IK — 3목표 도달 < 1e-6 m."""
    robot = RobotArm3DOF()
    targets = [(0.6, 0.2), (0.4, 0.4), (0.5, 0.3)]
    for target in targets:
        theta, hist = ik_dls(robot, target)
        assert hist[-1] < 1e-6, f'{target}: err = {hist[-1]:.2e}'


def test_ik_dls_workspace_boundary():
    """작업공간 경계 (반복 더 필요하지만 수렴)."""
    robot = RobotArm3DOF()
    theta, hist = ik_dls(robot, [0.0, 0.7], max_iter=2000)
    assert hist[-1] < 1e-3   # 경계는 정확도 완화


def test_ik_dls_multi_init():
    """다중 초기값 IK — 지역 최솟값 회피."""
    robot = RobotArm3DOF()
    theta, hist = ik_dls_multi(robot, [0.3, 0.5])
    assert hist[-1] < 1e-6


if __name__ == '__main__':
    test_ik_2dof_two_solutions(); print('✓ ik_2dof: 두 해')
    test_ik_2dof_fk_validation();  print('✓ ik_2dof: FK 검증')
    test_ik_2dof_unreachable();    print('✓ ik_2dof: 도달 불가')
    test_ik_2dof_clip_safety();    print('✓ ik_2dof: np.clip 안전')
    test_ik_dls_3_targets();       print('✓ ik_dls: 3목표 수렴')
    test_ik_dls_workspace_boundary(); print('✓ ik_dls: 경계점')
    test_ik_dls_multi_init();      print('✓ ik_dls_multi: 다중 초기값')
    print('\n모든 IK 테스트 통과')
