# RUNBOOK — ND1 M7 표준 실행 절차

> **목적**: 새 학습자·채점자·동료가 본 저장소를 받아 **0에서 100까지 그대로 실행** 가능하도록 표준화.
> 라면 봉지 뒷면의 조리법 같은 것 — 누가 끓이든 동일한 결과.

---

## Phase 1 — 환경 설정 (5분, 한 번만)

아래 A~D 중 하나를 선택하여 환경을 구성합니다.

```bash
# (1) 저장소 클론
git clone https://github.com/USERNAME/nd1-m7-robotics.git
cd nd1-m7-robotics
```

**방법 A — conda (권장 · Anaconda/Miniconda 설치 시)**

```bash
conda create -n nd1_m7 python=3.10 -y
conda activate nd1_m7
pip install -r requirements.txt
```

**방법 B — venv (Python 내장· 추가 설치 불필요)**

```bash
python3 -m venv nd1_m7               # Linux/macOS
python  -m venv nd1_m7               # Windows

source nd1_m7/bin/activate           # Linux/macOS
nd1_m7\Scripts\activate              # Windows (cmd)
. nd1_m7/Scripts/Activate.ps1       # Windows (PowerShell)

pip install -r requirements.txt
```

**방법 C — pyenv (Python 버전 관리가 필요할 때)**

```bash
# pyenv + venv 조합 (Python 3.10 미만인 경우)
pyenv install 3.10.14
pyenv local 3.10.14
python -m venv nd1_m7
source nd1_m7/bin/activate           # Linux/macOS
nd1_m7\Scripts\activate              # Windows
pip install -r requirements.txt
```

**방법 D — uv (최신 고속 패키지 관리자)**

```bash
# uv 설치 (Linux/macOS)
curl -LsSf https://astral.sh/uv/install.sh | sh
# Windows (PowerShell): powershell -c "irm https://astral.sh/uv/install.ps1 | iex"

uv venv nd1_m7 --python 3.10
source nd1_m7/bin/activate           # Linux/macOS
nd1_m7\Scripts\activate              # Windows
uv pip install -r requirements.txt
```

```bash
# (4) 환경 검증 (공통)
python -c "import numpy, scipy, matplotlib; print('✓ OK')"
```

**확인 사항**
- 가상환경 프롬프트가 `(nd1_m7)` 표시인지 확인
- `pip list` 에 numpy 1.26+, scipy 1.11+, matplotlib 3.8+ 존재
- ImportError 발생 시 → `deactivate && 방법별 활성화` 재시도

---

## Phase 2 — 단위 테스트 (1분, 매 코드 수정 후)

```bash
# 전체 테스트
python -m pytest tests/ -v
# → 24 passed in 1.24s 확인

# 특정 파일만
python -m pytest tests/test_ik.py -v

# 실패 시 디버그
python -m pytest tests/ -v --tb=long
```

**합격 기준**: `24 passed, 0 failed`

---

## Phase 3 — PBL 산출물 생성 (10분, 1회 또는 코드 변경 시마다)

```bash
# 실습 1 — 2DOF 해석 IK
python scripts/lab1_step3_two_solutions.py
# → results/M7_lab1_two_solutions.png
python scripts/lab1_step4_validation.py
# → results/M7_lab1_validation.png

# 실습 2 ⭐ PBL 핵심 (가장 중요)
python scripts/lab2_pbl_main.py
# → results/M7_lab2_convergence.png       (수렴 곡선)
# → results/M7_lab2_4poses.png            (4가지 자세)
# → results/M7_lab2_jacobian_check.png    (야코비안 검증)
# → results/M7_lab2_summary.csv           (목표별 결과)

# 실습 3 — 경로 + 조작성 + 특이점 회피
python scripts/lab3_step1_waypoints.py
# → results/M7_lab3_waypoints.png
python scripts/lab3_step2_manipulability.py
# → results/M7_lab3_manipulability.png
python scripts/lab3_step3_singularity_compare.py
# → results/M7_lab3_singularity_compare.png
python scripts/lab3_step4_animation.py   # 선택 (가산점)
# → results/M7_lab3_animation.gif
```

**합격 기준 (PBL 정확도 15점)**
- 4 목표 IK 모두 오차 `< 1e-6 m` ✓
- 야코비안 (해석 vs 수치) max diff `< 1e-6` ✓

---

## Phase 4 — GitHub 제출 (5분)

```bash
# (1) 산출물 확인
ls results/
# → 10개 PNG + 1개 GIF + 1개 CSV (총 12개)

# (2) git commit
git add results/ README.md
git commit -m "PBL final: 3목표 IK 1e-6 m 수렴 + 야코비안 검증"

# (3) push
git push origin main

# (4) 제출 — GitHub URL 강사에게 전달
# 예: https://github.com/USERNAME/nd1-m7-robotics
```

---

## Phase 5 — 확장 과제 ★★★ (선택, 가점 +10)

본 단계는 PBL 50점에 포함되지 않으나, **채용 포트폴리오 차별화**에 결정적.

```bash
# 확장 과제 1 — UR5e 6DOF 확장
python scripts/lab4_extension_6dof.py
# → results/M7_lab4_6dof_workspace.png
# → results/M7_lab4_6dof_summary.csv

# 본 교재 코드가 산업 6DOF 로봇에도 적용됨을 검증
```

---

## Phase 6 — M7 → M8 브릿지 검증 (M8 진입 직전)

```bash
# Dry-run 모드 (ROS2 미설치 환경)
python -m src.m7_to_ros2_bridge
# → JointState 메시지 구조 검증

# ROS2 설치 환경 (M8 학습 시작 후)
source /opt/ros/humble/setup.bash
python -m src.m7_to_ros2_bridge
# → ros2 topic echo /joint_states 로 확인 가능
```

---

## Phase 7 — 트러블슈팅

| 증상 | 진단 명령 | 해결 |
|------|----------|------|
| `ImportError: numpy` | `conda info -e` | 가상환경 활성화 확인 |
| `ModuleNotFoundError: src` | `pwd` | 루트 디렉터리에서 실행 |
| IK 수렴 실패 (`err > 1e-3`) | `np.degrees(theta_init)` 출력 | 초기값 변경 또는 λ↑ |
| matplotlib 한글 `□□□` | `fc-list \| grep -i korean` | `src/font_config` 호출 |
| pytest 실패 (test_jacobian) | `pytest -v --tb=long` | `np.clip` 적용 확인 |
| GitHub push 거부 | `git status` | 먼저 `git pull --rebase` |

---

## 한 줄 요약

| Phase | 시간 | 빈도 | 목적 |
|-------|-----|------|------|
| 1. 환경 설정 | 5분 | 한 번만 | 의존성 설치 |
| 2. 단위 테스트 | 1분 | 매 수정 후 | 회귀 확인 |
| 3. PBL 산출물 생성 | 10분 | 변경 시마다 | 결과물 갱신 |
| 4. GitHub 제출 | 5분 | 1회 (또는 갱신) | 평가 제출 |
| 5. 확장 과제 ★★★ | 30분~ | 선택 | 가점 + 포트폴리오 |
| 6. M7→M8 브릿지 | 5분 | M8 진입 시 | 다음 모듈 준비 |
| 7. 트러블슈팅 | — | 문제 발생 시 | 표 참조 |

→ **이 순서를 그대로 따르면 누구든 동일 결과** = production-ready.
