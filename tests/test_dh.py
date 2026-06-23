"""DH 변환 행렬 + 회전 행렬 단위 테스트.

실행: python -m pytest tests/test_dh.py -v
"""
import numpy as np
import pytest
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.robot_helpers import Rz, Ry, Rx, dh_matrix, make_T, inv_T


def test_rz_90deg():
    """Rz(90°) 가 x축을 y축으로 회전."""
    R = Rz(np.pi / 2)
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])


def test_rotation_properties():
    """회전 행렬 3가지 성질: det=1, RᵀR=I, R⁻¹=Rᵀ."""
    R = Rz(np.radians(37)) @ Ry(np.radians(52)) @ Rx(np.radians(13))
    assert np.allclose(np.linalg.det(R), 1.0), 'det(R) != 1'
    assert np.allclose(R.T @ R, np.eye(3)), 'R is not orthogonal'
    assert np.allclose(R.T, np.linalg.inv(R)), 'R⁻¹ != Rᵀ'


def test_dh_simple():
    """평면 DH (a=0.3, 나머지 0)."""
    T = dh_matrix(0.3, 0, 0, 0)
    assert np.allclose(T[:3, 3], [0.3, 0, 0])


def test_dh_90deg():
    """평면 DH (a=0.3, θ=90°) → y축 방향."""
    T = dh_matrix(0.3, 0, 0, np.pi / 2)
    assert np.allclose(T[:3, 3], [0, 0.3, 0], atol=1e-10)


def test_inv_T():
    """동차변환행렬 역행렬 정합성."""
    T = make_T(Rz(np.radians(45)), [0.5, 0.3, 0.2])
    Tinv = inv_T(T)
    assert np.allclose(T @ Tinv, np.eye(4), atol=1e-12)
    assert np.allclose(Tinv @ T, np.eye(4), atol=1e-12)


if __name__ == '__main__':
    test_rz_90deg(); print('✓ Rz(90°)')
    test_rotation_properties(); print('✓ 회전 행렬 3성질')
    test_dh_simple(); print('✓ DH 단순 케이스')
    test_dh_90deg(); print('✓ DH 90° 케이스')
    test_inv_T(); print('✓ 역행렬')
    print('\n모든 DH 테스트 통과')
