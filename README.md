# 3D Angle-Only Target Tracking

A C++ simulation framework for **3D angle-only target tracking** and comparison of nonlinear Bayesian filtering methods.

The simulation scenario is based on the paper:

> **Angle-only Filtering in 3D using Modified Spherical and Log Spherical Coordinates**

The project provides a common simulation environment for evaluating different state estimation methods under the same target trajectory and angle-only measurement conditions.

## Implemented Methods

* **EKF** — Extended Kalman Filter
* **UKF** — Unscented Kalman Filter
* **PF** — Particle Filter
* **UPF** — Unscented Particle Filter
* **PCRLB** — Posterior Cramér–Rao Lower Bound
* **MSC-EKF** — Modified Spherical Coordinates EKF *(work in progress)*

The filters are evaluated using **RMSE** and Monte Carlo simulations.

## Measurement Model

The target is observed using **angle-only measurements** consisting of bearing and elevation:

[
\mathbf{z}_k =
\begin{bmatrix}
\beta_k\
\epsilon_k
\end{bmatrix}
]

where:

* (\beta) — bearing/azimuth angle
* (\epsilon) — elevation angle

The target range is not directly measured.

## Simulation

The target is simulated in a 3D Cartesian coordinate system using a nonlinear measurement model with process and measurement noise.

The same simulated trajectory and measurement data are used to compare the different filtering methods.

The project evaluates:

* State estimation accuracy
* RMSE
* Filter convergence
* Robustness under noisy angle-only measurements
* Performance relative to PCRLB

## MSC-EKF

The **MSC-EKF** implementation is currently under development.

Current work includes:

* Cartesian → Modified Spherical Coordinates transformation
* Derivation of MSC state dynamics
* Relative acceleration transformation
* Continuous-time state equations
* State model discretization
* Jacobian derivation
* Validation against Cartesian-coordinate filters

## Requirements

* C++17
* Eigen
* Make
* Python 3
* NumPy
* Matplotlib

## How to Run

### 1. Build and run the simulation

From the project root:

```bash
make clean
make
make run
```

The simulation generates the RMSE data automatically.

The generated JSON files are stored in:

```text
json/
```

### 2. Plot RMSE

After running the simulation, use the Python scripts in the `python/` directory to visualize the results.

For example:

```bash
cd python
python3 plot_rmse.py
```

The plotting script reads the generated JSON files from:

```text
../json/
```

and generates the RMSE plots.

## Project Structure

```text
.
├── include/        # Header files
├── src/            # C++ source files
├── python/         # Python plotting scripts
├── json/           # Simulation output / RMSE data
├── main.cpp
├── Makefile
└── README.md
```

## Reference

**Angle-only Filtering in 3D using Modified Spherical and Log Spherical Coordinates**

This project is developed for **research and educational purposes**, with a focus on studying and comparing nonlinear filtering techniques for 3D angle-only target tracking.
