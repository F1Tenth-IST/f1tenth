# F1TENTH Autonomous Racing Stack

This repository contains a complete autonomous racing stack for a **1/10-scale F1TENTH vehicle**, featuring a modular architecture for **perception, planning, and control** designed for embedded hardware (NVIDIA Jetson Orin Nano).

## 🏎️ Overview
The framework implements a hierarchical architecture to push the vehicle to its physical limits while maintaining real-time feasibility on resource-constrained systems.

### Key Features
* **Perception:** LiDAR and IMU data fusion via an **Extended Kalman Filter (EKF)** and **Cartographer-based SLAM** for drift-compensated localization at 50 Hz.
* **Planning:** Automated extraction of smooth reference centerlines from occupancy grids using **Cubic B-Splines**.
* **Control:** Real-time **Nonlinear Model Predictive Control (NMPC)** in curvilinear coordinates, supporting both Kinematic and Dynamic vehicle formulations.

---

## 🚀 Getting Started

To get up and running with the full stack:

- 📦 [Installation Guide](docs/installation.md) — Complete environment setup (Docker or native)
- ▶️ [Usage Guide](docs/usage.md) — Running simulation, stack bringup, and MPC
- 🧭 [Repository Documentation](docs/documentation.md) — Codebase structure and key components

---

## 🛠️ System Architecture
The pipeline is designed to run entirely onboard the vehicle using ROS 2 Humble.

* **Localization:** Fuses high-frequency IMU (100 Hz) and VESC state with LiDAR odometry (10 Hz) to provide low-latency pose estimates.
* **MPC Solvers:** Formulated with **CasADi** and generated via **ACADOS**, employing a Sequential Quadratic Programming (SQP) method for high-efficiency execution.
* **Dynamic Modeling:** The Dynamic MPC (DMPC) accounts for tire slip and saturation using a Pacejka Magic Formula and friction ellipse constraints.

---

## 📊 Experimental Results
The stack has been validated both in simulation and real-world indoor racing scenarios.

| Configuration | RMSE [m] | Std. Dev. [m] |
| :--- | :---: | :---: |
| Bicycle Odometry Only | 0.43 | 0.21 |
| Bic. Odom + VESC IMU | 0.099 | 0.045 |
| **Bic. Odom + Dual IMU (Optimal)** | **0.090** | **0.040** |

* **Kinematic MPC (KMPC):** Demonstrated 50-lap continuous reliability with a standard deviation in lap times of only 1.1%.
* **Dynamic MPC (DMPC):** Achieved up to 25% lap time reduction in simulation by exploiting tire-road friction limits.

---

## 📚 Citation
If you use this work or code in your research, please cite our paper:

**Real-Time Model Predictive Control for High-Speed Mobile Robots** *Duarte Domingues, Alberto Vale, and José Gaspar* IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC), 2026.

```bibtex
@inproceedings{domingues2026realtime,
  title={Real-Time Model Predictive Control for High-Speed Mobile Robots},
  author={Domingues, Duarte and Vale, Alberto and Gaspar, José},
  booktitle={IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)},
  year={2026}
}
```

## ⚙️ Hardware Requirements
* **Vehicle:** F1TENTH platform (Traxxas Slash 4X4 VXL chassis).
* **Compute:** NVIDIA Jetson Orin Nano (8 GB).
* **Sensors:** Hokuyo URG-10LX LiDAR, Xsens MTi-930 IMU, VESC MK6 Motor Controller.
