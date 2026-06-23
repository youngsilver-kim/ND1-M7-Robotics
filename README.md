# ND1 M7 — 로봇공학 기초 (FK · IK · 야코비안)

> 3DOF 평면 로봇 팔의 정기구학·역기구학·조작성 분석 구현 패키지

## 📝 프로젝트 개요

본 저장소는 **ND1 피지컬 AI 전문가 과정 M7 모듈**의 PBL 학습 산출물 템플릿입니다.
DH 파라미터 기반 정기구학(FK)과 감쇠 최소자승법(DLS)을 활용한 수치 역기구학(IK)을
직접 구현하고, 시각적·수치적으로 검증하는 학습 파이프라인을 제공합니다.

### 핵심 결과 (작성 예시)

| 항목 | 결과 |
|------|------|
| 4목표 위치 IK 도달 오차 | **< 1e-6 m** ✓ |
| 평균 반복 횟수 (DLS, λ=1e-4) | 약 12~17회 (실측) |
| 야코비안 검증 (해석 vs 수치) | max diff **< 1e-6** ✓ |
| 단위 테스트 통과 | **24 / 24** ✓ |

## 🛠 환경 설정 및 실행

### 1. 가상환경 + 패키지 설치

아래 세 가지 방법 중 하나를 선택하여 환경을 구성합니다.

#### 방법 A — conda (권장 · Anaconda/Miniconda 설치 시)

```bash
conda create -n nd1_m7 python=3.10 -y
conda activate nd1_m7
pip install -r requirements.txt
```

#### 방법 B — venv (Python 내장, 추가 설치 불필요)

```bash
# 가상환경 생성
python3 -m venv nd1_m7          # Linux / macOS
python  -m venv nd1_m7          # Windows (python 명령)

# 가상환경 활성화
source nd1_m7/bin/activate      # Linux / macOS
nd1_m7\Scripts\activate         # Windows (cmd)
. nd1_m7/Scripts/Activate.ps1  # Windows (PowerShell)

# 패키지 설치
pip install -r requirements.txt

# 비활성화 (작업 종료 후)
deactivate
```

> ⚠️ **Python 버전 확인**: `python3 --version` 으로 3.10 이상인지 먼저 확인하세요.  
> 3.10 미만이면 방법 C(pyenv)를 이용하거나 python.org에서 최신 버전을 설치하세요.

#### 방법 C — pyenv (Python 버전 관리가 필요할 때)

```bash
# 1) pyenv 설치 (Linux/macOS)
curl https://pyenv.run | bash
# ~/.bashrc (또는 ~/.zshrc) 에 아래 내용 추가 후 터미널 재시작
# export PYENV_ROOT="$HOME/.pyenv"
# export PATH="$PYENV_ROOT/bin:$PATH"
# eval "$(pyenv init --path)"

# Windows 는 pyenv-win 사용
# pip install pyenv-win --target $HOME/.pyenv

# 2) Python 3.10 설치 및 설정
pyenv install 3.10.14
pyenv local 3.10.14             # 현재 폴더에만 적용

# 3) 가상환경 생성 + 활성화
python -m venv nd1_m7
source nd1_m7/bin/activate      # Linux/macOS
nd1_m7\Scripts\activate         # Windows

# 4) 패키지 설치
pip install -r requirements.txt
```

#### 방법 D — uv (최신 고속 패키지 관리자)

```bash
# 1) uv 설치 (Linux/macOS)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows (PowerShell)
# powershell -c "irm https://astral.sh/uv/install.ps1 | iex"

# 2) Python 3.10 + 가상환경 생성 + 패키지 설치 (한 번에)
uv venv nd1_m7 --python 3.10
source nd1_m7/bin/activate      # Linux/macOS
nd1_m7\Scripts\activate         # Windows
uv pip install -r requirements.txt
```

> 💡 **uv**는 pip보다 10~100배 빠른 설치 속도를 제공합니다. 최신 프로젝트 환경에서 권장합니다.

### 2. 환경 검증

```bash
python -c "import numpy, scipy, matplotlib; print('✓ OK')"
```

### 3. 단위 테스트 실행

```bash
pytest tests/ -v
# → 16 passed
```

### 4. 실습 스크립트 실행

```bash
python scripts/lab1_step3_two_solutions.py    # 실습 1 Step3 — 두 해 시각화
python scripts/lab1_step4_validation.py       # 실습 1 Step4 — 격자 검증
python scripts/lab2_pbl_main.py               # 실습 2 ⭐ PBL 핵심
python scripts/lab3_step1_waypoints.py        # 실습 3 Step1 — waypoint 경로
python scripts/lab3_step2_manipulability.py   # 실습 3 Step2 — 조작성 지형도
python scripts/lab3_step3_singularity_compare.py  # 실습 3 Step3 — 직진 vs 우회
python scripts/lab3_step4_animation.py        # 실습 3 Step4 — GIF (선택)
python scripts/lab4_kalman_estimation.py      # 실습 4 — 가산점
python scripts/lab4_extension_6dof.py         # 실습 4 확장 — 가산점 ★★★
```

> 📋 **표준 실행 절차**: `RUNBOOK.md` (Phase 1~7 전체 단계)

## 📂 디렉터리 구조

```
nd1_m7_studentname/
├── README.md           ⭐ 채점 시 가장 먼저 읽힘
├── RUNBOOK.md          표준 실행 절차 (Phase 1~7)
├── requirements.txt    Python 의존성 (numpy, scipy, matplotlib, pytest)
├── .gitignore
│
├── src/                핵심 모듈 (수정 금지)
│   ├── __init__.py
│   ├── robot_helpers.py    Rz/Ry/Rx, dh_matrix, make_T, inv_T
│   ├── robot_arm.py        RobotArm3DOF — FK + 시각화
│   ├── robot_arm_2dof.py   RobotArm2DOF — 실습 1용
│   ├── ik_analytical.py    ik_2dof — 해석적 IK (코사인 법칙)
│   ├── ik_numerical.py     ik_dls, ik_dls_multi — DLS 알고리즘
│   ├── jacobian.py         jacobian_numerical/analytical, manipulability
│   ├── kalman_filter.py    KalmanFilter1D/2D — 실습 4용
│   ├── sensor_simulator.py 카메라·라이다·IMU 시뮬레이터
│   ├── font_config.py      matplotlib 한글 폰트 자동 설정
│   └── m7_to_ros2_bridge.py M7→M8 JointState 발행 브릿지
│
├── tests/              단위 테스트 (16개)
│   ├── conftest.py         pytest 전역 설정 (slow 마커)
│   ├── test_dh.py          DH·회전 행렬 (5 testcase)
│   ├── test_ik.py          IK 해석 + 수치 (7 testcase)
│   ├── test_jacobian.py    야코비안 + 조작성 (4 testcase)
│   └── test_kalman.py      칼만 필터 + 센서 (8 testcase)
│
├── scripts/            실습 실행 스크립트
│   ├── lab1_step3_two_solutions.py    실습 1 Step3 — 2DOF 해석 IK 두 해
│   ├── lab1_step4_validation.py       실습 1 Step4 — 16격자 도달가능성 검증
│   ├── lab2_pbl_main.py               실습 2 — 3DOF DLS IK ⭐ PBL 핵심
│   ├── lab3_step1_waypoints.py        실습 3 Step1 — 4 waypoint 경로 IK
│   ├── lab3_step2_manipulability.py   실습 3 Step2 — 조작성 지형도
│   ├── lab3_step3_singularity_compare.py  실습 3 Step3 — 직진 vs 우회 비교
│   ├── lab3_step4_animation.py        실습 3 Step4 — 경로 따라가기 GIF (선택)
│   ├── lab4_kalman_estimation.py      실습 4 — 칼만 필터
│   └── lab4_extension_6dof.py         실습 4 확장 — UR5e 6DOF ★★★
│
├── notebooks/
│   └── 01_quickstart.ipynb  핵심 기능 5분 체험
│
├── results/            산출물 자동 저장 위치
│   ├── M7_lab2_convergence.png     ✅ 필수
│   ├── M7_lab2_4poses.png          ✅ 필수
│   ├── M7_lab2_jacobian_check.png  ✅ 필수
│   ├── M7_lab2_summary.csv         ✅ 필수
│   ├── M7_lab3_singularity_compare.png  ✅ 필수 (Step3)
│   ├── M7_lab3_animation.gif       ✅ 필수 (Step4)
│   └── ... (상세 목록 → results/README_results.md)
│
└── docs/
    ├── README_docs.md       교재·PPT·참고자료 링크
    ├── QUICK_START.md       3단계 빠른 시작
    ├── M7_평가문항_학생용.md  PBL 평가 문제지
    └── M7_버전_명세서.md     환경 버전 요구사항
```

## 🔬 알고리즘 핵심

### 정기구학 (FK)

```
DH 파라미터: a={0.3, 0.3, 0.2}m, d=0, α=0, θ=변수
T_total = T₀₁ · T₁₂ · T₂₃   (행렬 곱 누적)
```

### 역기구학 (IK) — DLS

```
θ_(k+1) = θ_k + α · Jᵀ · (J·Jᵀ + λ²I)⁻¹ · e_k
λ = 1e-4,  α = 0.85,  종료: ‖e‖ < 1e-6 m
```

### 야코비안 + 조작성

```python
J = jacobian_analytical_3dof(thetas)   # 2×3
w = manipulability(J)                  # √det(JJᵀ) → 0이면 특이점
```

## 📊 PBL 평가 기준

| 항목 | 점수 | 평가 기준 |
|------|------|----------|
| 코드 작동 | 15 | 실행 에러 없음 + pytest 16/16 |
| 정확도 | 15 | 3목표 IK 오차 < 1e-6 m + 야코비안 검증 |
| 시각화 | 10 | 7개 PNG 제출 + 라벨/범례/타이틀 |
| 문서화 | 10 | README + summary.csv + 결과 해설 |
| **합계** | **50** | |
| 칼만 필터 (가산점) | +10 | RMSE < 0.3m(1D), 0.5m(2D) |

## 📈 결과 해설 (작성 예시)

| 목표 위치 | 반복 횟수 | 최종 오차 | 조작성 w |
|-----------|----------|----------|---------|
| (0.6, 0.2) | 13 | 3.15e-7 | 0.1244 |
| (0.4, 0.4) | 14 | 7.98e-7 | 0.1200 |
| (0.0, 0.7) | 17 | 2.63e-7 | 0.1061 |
| (-0.3, 0.5) | 12 | 6.11e-7 | 0.1592 |

## 🌉 M7 → M8 브릿지

```bash
# Dry-run 모드 (ROS2 불필요)
python -m src.m7_to_ros2_bridge

# ROS2 Humble 환경 (M8 진입 후)
source /opt/ros/humble/setup.bash
python -m src.m7_to_ros2_bridge
# → /joint_states 토픽 발행 → RViz2 / Gazebo 연동
```

## 📚 참고 자료

- Wampler (1986). DLS IK 원논문
- Yoshikawa (1985). Manipulability 정의
- Lynch & Park, *Modern Robotics* — [무료 PDF](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)

## 📜 라이선스

MIT License

---
**© ND1 피지컬 AI 전문가 과정 / Module 7 — 로봇공학 기초**
