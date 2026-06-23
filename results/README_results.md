# results — 산출물 저장 폴더

실습 스크립트 실행 시 자동으로 생성됩니다.
이 폴더의 파일을 GitHub에 함께 커밋하여 제출하세요.

## 파일 목록

| 파일 | 생성 스크립트 | 제출 |
|------|------------|------|
| M7_lab1_two_solutions.png | scripts/lab1_step3_two_solutions.py | ✅ 필수 |
| M7_lab1_validation.png | scripts/lab1_step4_validation.py | ✅ 필수 |
| M7_lab2_convergence.png | scripts/lab2_pbl_main.py | ✅ 필수 |
| M7_lab2_4poses.png | scripts/lab2_pbl_main.py | ✅ 필수 |
| M7_lab2_jacobian_check.png | scripts/lab2_pbl_main.py | ✅ 필수 |
| M7_lab2_summary.csv | scripts/lab2_pbl_main.py | ✅ 필수 |
| M7_lab3_waypoints.png | scripts/lab3_step1_waypoints.py | ✅ 필수 |
| M7_lab3_manipulability.png | scripts/lab3_step2_manipulability.py | ✅ 필수 |
| M7_lab3_singularity_compare.png | scripts/lab3_step3_singularity_compare.py | ✅ 필수 |
| M7_lab3_animation.gif | scripts/lab3_step4_animation.py | 권장 (선택) |
| M7_lab4_kalman_1d.png | scripts/lab4_kalman_estimation.py | 가산점 +3 |
| M7_lab4_kalman_2d.png | scripts/lab4_kalman_estimation.py | 가산점 +3 |
| M7_lab4_summary.csv | scripts/lab4_kalman_estimation.py | 가산점 +3 |
| M7_lab4_6dof_workspace.png | scripts/lab4_extension_6dof.py | 가산점 +4 |
| M7_lab4_6dof_summary.csv | scripts/lab4_extension_6dof.py | 가산점 +4 |

## 한 번에 전체 생성

```bash
cd nd1_m7_template
conda activate nd1_m7

python scripts/lab1_step3_two_solutions.py
python scripts/lab1_step4_validation.py
python scripts/lab2_pbl_main.py
python scripts/lab3_step1_waypoints.py
python scripts/lab3_step2_manipulability.py
python scripts/lab3_step3_singularity_compare.py
python scripts/lab3_step4_animation.py        # 선택 (가산점 아님, PNG/GIF 보강)
python scripts/lab4_kalman_estimation.py      # 가산점
python scripts/lab4_extension_6dof.py         # 가산점

ls results/   # 전체 파일 확인
```

## 제출 체크리스트

- [ ] lab1 PNG 2개 (two_solutions, validation)
- [ ] lab2 PNG 3개 + CSV 1개 (PBL 핵심)
- [ ] lab3 PNG 3개 (waypoints, manipulability, singularity_compare — 필수)
- [ ] lab3 GIF 1개 (animation — 권장)
- [ ] lab4 칼만 PNG 2개 + CSV 1개 (가산점)
- [ ] lab4 6DOF PNG 1개 + CSV 1개 (가산점)
- [ ] README.md 핵심 결과 수치 업데이트
