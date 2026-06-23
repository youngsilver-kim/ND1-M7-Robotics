"""야코비안 단위 테스트.

실행: python -m pytest tests/test_jacobian.py -v
"""
import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.robot_arm import RobotArm3DOF
from src.jacobian import (jacobian_numerical, jacobian_analytical_3dof,
                           jacobian_analytical, manipulability)


def test_numerical_vs_analytical():
    """수치 야코비안 vs 해석 야코비안 (12 자세)."""
    robot = RobotArm3DOF()
    rng = np.random.default_rng(42)
    for _ in range(12):
        thetas = rng.uniform(-np.pi/2, np.pi/2, 3)
        J_n = jacobian_numerical(robot, thetas)
        J_a = jacobian_analytical_3dof(thetas)
        diff = np.abs(J_n - J_a).max()
        assert diff < 1e-6, f'불일치: {diff:.2e} at {thetas}'


def test_manipulability_normal():
    """일반 자세 조작성 > 0."""
    J = jacobian_analytical_3dof([np.pi/4, np.pi/3, np.pi/6])
    w = manipulability(J)
    assert w > 0.01


def test_manipulability_singular_extended():
    """완전 펴짐 자세 → 특이점 (w ≈ 0)."""
    J = jacobian_analytical_3dof([0, 0, 0])
    w = manipulability(J)
    assert w < 1e-9


def test_manipulability_singular_folded():
    """완전 접힘 자세 → 특이점."""
    J = jacobian_analytical_3dof([0, np.pi, 0])
    w = manipulability(J)
    assert w < 1e-9


def test_jacobian_analytical_alias():
    """교재 호환 alias 검증 — jacobian_analytical == jacobian_analytical_3dof."""
    thetas = [0.3, 0.5, 0.1]
    J1 = jacobian_analytical(thetas)
    J2 = jacobian_analytical_3dof(thetas)
    assert np.allclose(J1, J2)


if __name__ == '__main__':
    test_numerical_vs_analytical(); print('✓ 수치 vs 해석 (12 자세)')
    test_manipulability_normal(); print('✓ 일반 자세 조작성')
    test_manipulability_singular_extended(); print('✓ 완전 펴짐 특이점')
    test_manipulability_singular_folded(); print('✓ 완전 접힘 특이점')
    print('\n모든 야코비안 테스트 통과')
