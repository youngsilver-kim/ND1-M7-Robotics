# conftest.py — pytest 전역 설정
#
# 실행:
#   conda activate nd1_m7  (conda) 또는  source nd1_m7/bin/activate  (venv)
#   pytest tests/ -v
#   → 16 passed (test_dh:5, test_ik:7, test_jacobian:4, test_kalman:8 포함)
#
# slow 마커 테스트만 제외 (칼만 시뮬레이션 등 시간 소요 테스트):
#   pytest tests/ -v --skip-slow

import pytest


def pytest_configure(config):
    """커스텀 마커 등록."""
    config.addinivalue_line(
        "markers", "slow: 실행 시간이 긴 테스트 (칼만 시뮬레이션 등)"
    )


def pytest_addoption(parser):
    parser.addoption(
        "--skip-slow", action="store_true",
        default=False, help="slow 마커 테스트 건너뜀"
    )


def pytest_collection_modifyitems(config, items):
    """--skip-slow 플래그로 slow 마커 테스트 건너뛰기."""
    if not config.getoption("--skip-slow", default=False):
        return
    skip_slow = pytest.mark.skip(reason="--skip-slow 플래그로 건너뜀")
    for item in items:
        if "slow" in item.keywords:
            item.add_marker(skip_slow)
