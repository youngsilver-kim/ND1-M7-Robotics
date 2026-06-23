# docs — 참고 문서 모음

## 교재 (강사로부터 배포)

| 파일 | 내용 |
|------|------|
| `M7_표준안_교재.docx` | ND1 M7 표준안 교재 (강사 배포) |
| `M7_강의용_PPT.pptx` | 강의 슬라이드 (강사 배포) |
| `M7_강사용_지도서.docx` | 강사용 운영 가이드 |

> ⚠️ 위 파일들은 크기가 크므로 GitHub에 커밋하지 마세요 (.gitignore 권장)

## 이 폴더의 학습 자료

| 파일 | 내용 |
|------|------|
| `QUICK_START.md` | 3단계로 시작하기 (5분 가이드) |
| `M7_평가문항_학생용.md` | PBL 평가 문제지 (문제풀이 50 + 실습 50) |
| `M7_버전_명세서.md` | Python · 패키지 버전 요구사항 |

## 외부 참고 자료

- **NumPy**: https://numpy.org/doc/stable/
- **Matplotlib**: https://matplotlib.org/stable/
- **SciPy Rotation**: https://docs.scipy.org/doc/scipy/reference/spatial.transform.html
- **DH 파라미터**: https://en.wikipedia.org/wiki/Denavit-Hartenberg_parameters
- **Modern Robotics (무료 PDF)**: http://hades.mech.northwestern.edu/index.php/Modern_Robotics

## 참고 논문

| 논문 | 내용 |
|------|------|
| Wampler (1986) | DLS IK 원논문 |
| Yoshikawa (1985) | 조작성(Manipulability) 정의 |
| Kalman (1960) | 칼만 필터 원논문 |

## M7 → M8 연계

| M7 산출물 | M8 활용 |
|---------|---------|
| `src/m7_to_ros2_bridge.py` | M8 `/joint_states` 토픽 발행 |
| IK 수렴 결과 θ | ROS2 JointTrajectory 웨이포인트 |
| 야코비안 기초 | TF2 프레임 속도 제어 참고 |
| 동차변환행렬 | TF2 map → odom → base_link 변환 |
