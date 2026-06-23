# M7 Quick Start — 3단계로 시작하기

> nd1_m7_template v1.0 | Ubuntu 22.04 LTS / Python 3.10+ 기준

---

## STEP 1 — 환경 설정 (5분)

아래 A~D 중 하나를 선택합니다.

**방법 A — conda (권장)**
```bash
# 1. 압축 해제
unzip ND1_M7_PBL_Template.zip
cd nd1_m7_template

# 2. 가상환경 생성 및 활성화
conda create -n nd1_m7 python=3.10 -y
conda activate nd1_m7

# 3. 패키지 설치
pip install -r requirements.txt

# 4. 설치 확인
python3 -c "import numpy, scipy, matplotlib; print('✅ 환경 설정 완료')"
```

**방법 B — venv (Python 내장· 추가 설치 불필요)**
```bash
unzip ND1_M7_PBL_Template.zip && cd nd1_m7_template

python3 -m venv nd1_m7                   # Linux/macOS
python  -m venv nd1_m7                   # Windows

source nd1_m7/bin/activate               # Linux/macOS
nd1_m7\Scripts\activate                  # Windows (cmd)

pip install -r requirements.txt
python3 -c "import numpy, scipy, matplotlib; print('✅ 환경 설정 완료')"
```

**방법 C — pyenv (Python 버전 관리 필요 시)**
```bash
pyenv install 3.10.14 && pyenv local 3.10.14
python -m venv nd1_m7 && source nd1_m7/bin/activate
pip install -r requirements.txt
```

**방법 D — uv (고속)**
```bash
uv venv nd1_m7 --python 3.10 && source nd1_m7/bin/activate
uv pip install -r requirements.txt
```

---

## STEP 2 — 환경 검증 (1분)

```bash
# 단위 테스트 전체 실행
python -m pytest tests/ -v
```

**기대 결과:**
```
tests/test_dh.py        5 passed
tests/test_ik.py        7 passed
tests/test_jacobian.py  4 passed
tests/test_kalman.py    ? passed
─────────────────────────────
모든 테스트 PASSED ✅
```

---

## STEP 3 — 실습 실행

### 실습 1 — 2DOF 해석적 IK (30초)
```bash
python scripts/lab1_two_solutions.py
# → results/M7_lab1_two_solutions.png 생성
```

### 실습 2 ⭐ PBL 핵심 — 3DOF DLS IK (1~3분)
```bash
python scripts/lab2_pbl_main.py
# → results/M7_lab2_convergence.png
# → results/M7_lab2_4poses.png
# → results/M7_lab2_summary.csv
```

**PBL 합격 기준:**

| 목표 위치 | 수렴 오차 | 기준 |
|-----------|---------|------|
| T1 (0.6, 0.2) | < 1e-6 m | ✅ |
| T2 (0.4, 0.4) | < 1e-6 m | ✅ |
| T3 (0.0, 0.7) | < 1e-6 m | ✅ |

### 실습 3 — 경로 + 조작성 (1분)
```bash
python scripts/lab3_waypoints_manipulability.py
# → results/M7_lab3_waypoints.png
# → results/M7_lab3_manipulability.png
```

### 실습 4 ★선택 — 칼만 필터 (30초)
```bash
python scripts/lab4_kalman_estimation.py
# → results/M7_lab4_kalman_1d.png
# → results/M7_lab4_kalman_2d.png
```

---

## 주요 파일 구조

```
nd1_m7_template/
├── src/              핵심 모듈 (수정 금지)
├── scripts/          실습 실행 스크립트
├── tests/            단위 테스트
├── results/          실행 결과 자동 저장
├── notebooks/        01_quickstart.ipynb
└── docs/             교재·강의자료·평가문항
```

---

## 자주 발생하는 오류

| 증상 | 원인 | 해결 |
|------|------|------|
| `ModuleNotFoundError: src` | 루트 디렉터리 아닌 곳에서 실행 | `cd nd1_m7_template` 후 재실행 |
| matplotlib 한글 □□□ | 한글 폰트 미설치 | `sudo apt-get install fonts-nanum && fc-cache -fv` |
| `ImportError: numpy` | 가상환경 미활성화 | `conda activate nd1_m7` |
| IK 수렴 실패 | lam 값 문제 | `lam=1e-2` 로 증가 시도 |

---

> 상세 내용 → `docs/M7_textbook.docx` | 단계별 절차 → `RUNBOOK.md`
