# PRO-Lab - Evaluation Overview

*What exactly is being evaluated: filters, sensor inputs, metrics, conditions.*
Task ID **2510331021 - Wrong Initialization** (initialise the filter with an
incorrect starting pose / uncertainty and compare recovery).

---

## 1. Filters under evaluation

All run in parallel on the **same** simulation, inputs, coordinate frame and
trajectory, so differences come only from the estimator (and its init error).

| Filter | State | Dim | Sensor inputs | Measurement model | Frame |
|---|---|---|---|---|---|
| **KF**     | `[x, y, yaw, vx, vy, ω]` | 6D | `/imu`, `/landmarks` | linear constant-velocity predict; landmark **triangulation** → one linear `(x,y)` position update. `/cmd_vel` deliberately **not** used (keeps it truly linear) | `odom` |
| **EKF**    | `[x, y, yaw]` | 3D | `/cmd_vel`, `/imu`, `/landmarks` | non-linear unicycle predict; range/bearing landmark update via Jacobian | `odom` |
| **EKF-LF** | `[x, y, yaw]` | 3D | `/cmd_vel`, `/imu`, `/scan`, `/map` | EKF with **direct scan-likelihood** update (linearised around current pose) | `map` |
| **PF**     | `[x, y, yaw]` | 3D | `/cmd_vel`, `/imu`, `/scan`, `/map` | augmented MCL particle filter + likelihood-field scan; non-linear / non-Gaussian | `map` |
| **AMCL**   | Nav2 internal | - | Nav2 internal | gold-standard **baseline** (Nav2). No filter consumes it - logged alongside for "our filters vs Nav2" | `map` |

KF uses different (higher) state dimension on purpose: promoting velocities
into the state keeps `F` a constant linear matrix, so it stays a *true* KF
instead of degrading into an EKF.

---

## 2. Sensor inputs (and their source)

| Topic | Source | Used by | Notes |
|---|---|---|---|
| `/imu` | gz IMU (bridged) | KF, EKF, EKF-LF, PF | orientation yaw + `angular_velocity.z` (gyro) |
| `/odom` | gz wheel odometry (bridged) | - / PF motion | dead-reckoning |
| `/scan` | gz 2D lidar (bridged) | EKF-LF, PF | likelihood-field against `/map` |
| `/cmd_vel` | scripted `trajectory_player` | EKF, EKF-LF, PF (control input) | bridged ROS→gz to actually move the robot |
| `/landmarks/observations` | `landmark_detector_node` | KF, EKF | **self-defined** landmarks: 3 posts at `(-5,2)`, `(3,-2)`, `(-2,-6)`; noisy range/bearing (σ_range = 0.10 m, σ_bearing = 0.05 rad, max_range = 6 m, 5 Hz) |
| `/ground_truth/pose` | `truth_relay_node` (TF `odom→base_footprint`) | metrics reference | = Gazebo's **true** robot pose; every error metric is measured against this |

---

## 3. Metrics (logged per tick to CSV, then aggregated)

| Metric | Meaning |
|---|---|
| `error_xy(t)` | Euclidean distance estimate ↔ truth |
| `error_yaw(t)` | wrapped yaw difference |
| `rmse_xy`, `rmse_yaw` | running RMSE up to time *t* |
| `converged` | `true` once `error_xy < 0.20 m` for ≥ 2 s continuously |
| `time_to_converge` | seconds until first convergence |
| `nees` | Normalized Estimation Error Squared - χ² consistency; mean ≈ state dim when well-calibrated, ≫ → overconfident |
| `pf_ess` | PF effective sample size (`1 / Σ wᵢ²`) - collapses under degeneracy |
| `runtime_us` | per-tick wall-clock cost (real-time / performance comparison) |

---

## 4. Experiment conditions

- **World:** `warehouse` (`nav2_minimal_tb4_sim`); TB4 spawns at truth pose
  `(x = -8.0, y = -0.5, yaw = 0.0)`.
- **Trajectory:** one deterministic scripted `/cmd_vel` sequence, identical for
  every run (~43 s: straight → in-place turns → arcs → backward → 90° corners →
  figure-8). Only the (wrong) initial state changes between runs.
- **Statistics:** 10 random seeds per scenario; seed perturbs the *sampled*
  initial state around `init_x/y/yaw`, the sim itself is deterministic.
- **Scenarios:** `correct_init` (baseline) + wrong-init variants
  (`offset_1m`, `offset_5m`, `wrong_yaw_pi2`, `overconfident_wrong`,
  `underconfident`, `kidnapped`).

### Mandatory experiments (assignment)
- **Q** (process-noise) variation - model confidence
- **R** (measurement-noise) variation - sensor trust
- **Runtime / performance** - computational cost, real-time capability
- **Ground-truth evaluation** - RMSE vs truth
- **Landmark detection** - self-defined landmarks (above)

---

## 5. Plots produced (`scripts/analyze_results.py`)

- **Per scenario:** `error_xy`, `error_yaw`, `nees`, `pf_ess`, `trajectory`
- **Aggregate:** `rmse_comparison`, `ttc_comparison`, `runtime_comparison`,
  `convergence_rate`
- Formats: `png` (slides/README), `pgf` (LaTeX/IEEE paper), `pdf`.
