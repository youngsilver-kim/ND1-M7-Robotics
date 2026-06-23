# ND1-M7-Robotics (❗과제 0623, 3d_exercise 폴더 참고❗)

A robotics learning project covering Forward Kinematics (FK), Inverse Kinematics (IK), Jacobian analysis, manipulability evaluation, and Kalman filtering for planar robotic manipulators.

---

## Overview

This repository contains implementations and experiments developed during a robotics fundamentals module. The project focuses on core robotics concepts including:

* Forward Kinematics (FK)
* Analytical Inverse Kinematics (IK)
* Numerical Inverse Kinematics using Damped Least Squares (DLS)
* Jacobian Matrix Analysis
* Manipulability Evaluation
* Singularity Analysis
* Kalman Filter-based State Estimation

The codebase provides reusable robotics utilities, visualization tools, numerical solvers, and experiment scripts.

---

## Key Features

* 2-DOF analytical inverse kinematics
* 3-DOF numerical inverse kinematics (DLS)
* Jacobian verification (analytical vs numerical)
* Manipulability landscape visualization
* Singularity avoidance analysis
* Kalman filtering for noisy sensor measurements
* Automated result generation and plotting

---

## Project Structure

```text
ND1-M7-Robotics/
├── docs/
├── notebooks/
├── results/
├── scripts/
├── src/
└── tests/
```

### Core Modules

```text
src/
├── robot_arm.py
├── robot_arm_2dof.py
├── ik_analytical.py
├── ik_numerical.py
├── jacobian.py
├── kalman_filter.py
├── sensor_simulator.py
└── robot_helpers.py
```

---

## Installation

### Create a Virtual Environment

```bash
python -m venv nd1_m7
source nd1_m7/bin/activate
```

### Install Dependencies

```bash
pip install -r requirements.txt
```

### Verify Installation

```bash
python -c "import numpy, scipy, matplotlib; print('Environment Ready')"
```

---

## Running Experiments

### Lab 1 — Analytical IK

```bash
python scripts/lab1_step3_two_solutions.py
python scripts/lab1_step4_validation.py
```

### Lab 2 — Numerical IK (DLS)

```bash
python scripts/lab2_pbl_main.py
```

### Lab 3 — Manipulability & Singularity Analysis

```bash
python scripts/lab3_step1_waypoints.py
python scripts/lab3_step2_manipulability.py
python scripts/lab3_step3_singularity_compare.py
python scripts/lab3_step4_animation.py
```

### Lab 4 — Kalman Filtering

```bash
python scripts/lab4_kalman_estimation.py
```

### Extension — 6-DOF Manipulator

```bash
python scripts/lab4_extension_6dof.py
```

---

## Algorithms

### Forward Kinematics

The robot pose is computed using Denavit-Hartenberg (DH) parameters:

```text
T = T01 · T12 · T23
```

### Damped Least Squares Inverse Kinematics

```text
θ(k+1) = θ(k) + α Jᵀ (J Jᵀ + λ² I)⁻¹ e
```

where:

* α = 0.85
* λ = 1e-4

### Manipulability

Following Yoshikawa's formulation:

```text
w = √det(JJᵀ)
```

A manipulability value approaching zero indicates a singular configuration.

---

## Generated Results

The generated plots and reports are stored in:

```text
results/
```

Typical outputs include:

* IK convergence plots
* Jacobian validation plots
* Manipulability maps
* Singularity comparison figures
* Animation GIFs
* Kalman filter estimation results

---

## References

* Yoshikawa, T. (1985). Manipulability of Robotic Mechanisms.
* Wampler, C. (1986). Manipulator Inverse Kinematic Solutions Based on Vector Formulations and Damped Least Squares.
* Lynch, K. M., & Park, F. C. Modern Robotics.

---

## License

MIT License
