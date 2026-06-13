# Wrong Initialization - Experiment Guide

Task assigned to student group **2B1: 2510331021** / **2B2: 2510331007**:

> *Initialize the filter with an incorrect starting pose and/or uncertainty.*

This document describes the experimental setup, the scenarios, the metrics
captured, and how to reproduce the results.

---

## What we measure

Each scenario starts the simulation with the robot at a known truth pose
**(x=−8.00, y=−0.50, yaw=0.0)** in the warehouse world. Five estimators run
in parallel and start from the **same believed pose** which may or may not
match the truth - the wrong-init experiment:

| Estimator | State | Inputs | Notes |
|---|---|---|---|
| **KF**     | `[x, y, yaw, vx, vy, ω]` (6D) | /imu, /landmarks         | Constant-velocity model in world frame - predict + measurements all linear. /cmd_vel is *not* consumed (would be non-linear in yaw); velocities are learned passively from sequential landmark/IMU updates |
| **EKF**    | `[x, y, yaw]` (3D)            | /cmd_vel, /imu, /landmarks | Textbook unicycle: non-linear predict around `cos(yaw)`, range/bearing landmark update via Jacobian |
| **EKF-LF** | `[x, y, yaw]` (3D)            | /cmd_vel, /imu, /scan, /map | Advanced EKF variant - direct scan-likelihood update around the current estimate. Expected to fail under big init errors (linearisation outside its trust region) |
| **PF**     | `[x, y, yaw]` (3D)            | /cmd_vel, /imu, /scan, /map | Full non-linear, non-Gaussian particle filter with augmented MCL + likelihood-field scan |
| **AMCL**   | (Nav2 internal)               | (Nav2 internal)             | External comparison track - no filter consumes its output, it just runs alongside so we can plot "our filters vs Nav2 standard" |

### Why different state dimensions per filter?

Each filter uses the state representation that matches its mathematical assumptions:

- **KF (6D)**: A linear KF requires `x' = F·x + w` with no products of state variables and no trig functions. The 3D unicycle predict `x' = x + v·cos(yaw)·dt` is non-linear in `yaw`, which would force an EKF. By promoting `(vx, vy, ω)` into the state - pure constant-velocity model in world frame - `F` becomes a constant 6×6 block-diagonal matrix and the filter stays *truly* linear. The price is no `/cmd_vel` consumption (body→world conversion is itself non-linear); the filter learns velocity from successive position updates.
- **EKF / EKF-LF (3D)**: Can handle non-linearity via Jacobian, so the compact 3D unicycle state is natural. Uses `/cmd_vel` directly as control input.
- **PF (3D)**: Compact state per particle (~500-3000 of them); non-linearity is handled by sampling, not linearisation. Same 3D unicycle as EKF.

The KF can't take a raw scan because the scan-to-pose mapping is non-linear
(small yaw changes redirect every beam). Triangulation sidesteps that:
each visible landmark + the current yaw estimate gives a Cartesian position
estimate; we average across visible landmarks and feed the result as one
linear `(x, y)` measurement. EKF-LF shows what happens when you try to
bypass that - it linearises the scan model directly and pays the price in
wrong-init scenarios.

To get statistical aussagekraft each scenario is run **10 times** with
different random seeds (`scripts/run_experiments.sh --seeds 10`). The seed
randomizes the filter's *sampled initial state* around `init_x/y/yaw` using
`init_spread_xy/yaw`; the simulation itself is deterministic.

We compare:

| Metric | Description |
|---|---|
| `error_xy(t)` | Euclidean distance between estimate and truth, per filter |
| `error_yaw(t)` | wrapped yaw difference |
| `rmse_xy`, `rmse_yaw` | running RMSE up to time *t* |
| `converged` | true once `error_xy < 0.20 m` for ≥ 2 s continuously |
| `time_to_converge` | seconds from experiment start to first convergence |
| `nees` | Normalized Estimation Error Squared - χ² consistency check; mean ≈ 3 (state dim) when filter is well-calibrated, much higher → overconfident |
| `pf_ess` | PF effective sample size (1 / Σ wᵢ²) - collapses with degeneracy |
| `runtime_us` | wall-clock per-tick cost (ties into "Runtime / Performance" comparison) |

Truth is read from TF (`odom → base_footprint`) by `truth_relay_node` and
republished as `/ground_truth/pose` (PoseStamped). The metrics node compares
each filter pose to this signal.

---

## Scenarios (config/scenarios/*.yaml)

| File | Init pose error | Init spread | Notes |
|---|---|---|---|
| `correct_init.yaml`        | 0 | tight (10 cm) | baseline - should converge instantly |
| `offset_1m.yaml`           | +1 m in X | 0.5 m | mild bias, spread covers truth |
| `offset_5m.yaml`           | +5 m in X | 1.0 m, N=1000 | bigger bias, PF needs more particles |
| `wrong_yaw_pi2.yaml`       | +π/2 yaw | 0.2 m / 0.3 rad | rotational error, hard for linearised filters |
| `overconfident_wrong.yaml` | +3 m, +2 m | tight 10 cm | **PF particle deprivation** - headline finding |
| `underconfident.yaml`      | 0 | huge (3 m, 1.5 rad) | slow convergence for KF/EKF |
| `kidnapped.yaml`           | uniform across map | N=3000 | global localization, KF/EKF expected to fail |

Edit any YAML to sweep parameters; they are loaded as ROS 2 parameters by
all three filter nodes.

---

## Running a single scenario

Inside the `prolab_jazzy` container (or any sourced ROS 2 Jazzy environment):

```bash
ros2 launch pro_lab_filters wrong_init_experiment.launch.py \
    scenario:=overconfident_wrong \
    duration_s:=60 \
    out_dir:=/tmp/pro_lab_results
```

Launch args:

| Arg | Default | Purpose |
|---|---|---|
| `scenario`     | `correct_init` | YAML file basename (without `.yaml`) |
| `duration_s`   | `0` (forever) | auto-shutdown after N seconds |
| `out_dir`      | `/tmp/pro_lab_results` | CSV destination |
| `world`        | `warehouse` | gz world (warehouse / depot / maze) |
| `x_pose`, `y_pose`, `yaw` | warehouse defaults | spawn pose (truth) |
| `use_rviz`     | `true` | start RViz |
| `use_nav2`     | `true` | bring up Nav2 (provides `/pose` measurement) |

Outputs in `out_dir`:

- `<scenario>_timeseries.csv` - per-step truth, estimates, errors, RMSE, ESS
- `<scenario>_summary.csv` - final RMSE, convergence flag, time-to-converge

---

## Running the full pipeline (scenarios × 10 seeds)

The orchestrator at `scripts/run_experiments.sh` loops over every scenario
and N seeds, spinning up Gazebo + all four estimators + the scripted
trajectory_player + metrics + csv_logger for each run. Between runs it tears
down and starts fresh so there's no state leakage.

```bash
# default: 10 seeds × every scenario × 60 s, RViz visible
bash scripts/run_experiments.sh

# fewer seeds, single scenario, longer run
bash scripts/run_experiments.sh --seeds 5 --scenario offset_5m --duration 90

# headless (faster, no RViz window)
bash scripts/run_experiments.sh --headless
```

Each run drops:

- `<scenario>_seed<NN>_timeseries.csv` - per-tick truth, estimate, error, RMSE, NEES, ESS, runtime
- `<scenario>_seed<NN>_summary.csv` - final RMSE, convergence flag, time-to-converge, runtime stats

These land in `./results/` directly - `docker/docker-compose.yml` bind-mounts
the host repo's `./results` to `/home/ros/results` inside the container, so
no `docker cp` needed after a sweep. (If you reconfigure compose, recreate
the container with `docker compose up -d --force-recreate` so the new mount
takes effect.)

Aggregate over seeds with `scripts/analyze_results.py` (mean ± std bars, per
filter, per scenario).

### Scripted trajectory

A C++ node `trajectory_player_node` publishes a deterministic /cmd_vel
sequence after a 14 s warm-up: straight → in-place left turn → arc-left →
in-place right turn → arc-right → backward → 90° corners → figure-8. About
43 s total, fits in the default 60 s window. Same trajectory for every run
so the only thing that changes across seeds is the (wrong) initial state.

Disable for manual teleop debugging via `use_trajectory:=false` on the launch
file.

---

## Visualization

### RViz

Launched automatically (unless `use_rviz:=false`). The bundled config
[`config/wrong_init.rviz`](../ws/src/pro_lab_filters/config/wrong_init.rviz)
shows:

- TurtleBot4 RobotModel
- Warehouse Map (Nav2) - `/map`
- LaserScan - `/scan`
- Green axes → ground truth (`/ground_truth/pose`)
- Blue arrow + covariance ellipse → KF (`/kf/pose`)
- Green-ish arrow + covariance ellipse → EKF (`/ekf/pose`)
- Purple arrow + covariance ellipse → EKF-LF (`/ekf_lf/pose`)
- Red arrow + covariance ellipse → PF (`/pf/pose`)
- Yellow arrow + covariance ellipse → AMCL baseline (`/amcl/pose` relayed)
- Red flat arrows → PF particle cloud (`/pf/particles`)

---

## Expected findings (hypotheses to verify in the report)

1. **`correct_init`** - all three filters track truth from t=0; RMSE ≈ sensor
   noise floor.
2. **`offset_1m`** - KF/EKF: smooth exponential pull-in (~1–2 s). PF: similar
   if the spread covers truth.
3. **`offset_5m`** - convergence times grow; PF needs N≥1000 particles to
   keep the truth in the support of the prior, otherwise tail-events
   dominate weighting.
4. **`wrong_yaw_pi2`** - KF (linear constant-velocity assumption) drifts
   noticeably; EKF and PF, both nonlinear-aware, recover faster.
5. **`overconfident_wrong`** - *headline result for this study*:
   - KF/EKF still converge eventually because the Kalman gain pulls the
     estimate toward each `/pose` update (asymptotically consistent).
   - **EKF-LF diverges**: the Jacobian ∂z/∂x is computed at a pose that's
     far from truth, so the gradient of the distance field at the predicted
     beam endpoints points "away from" rather than "toward" the right
     correction. Linearisation outside its trust region - exactly the
     failure mode classic textbooks warn about (Thrun §7.4).
   - PF cannot recover: all particles are far from truth, every weight is
     ≈ 0, resampling perpetuates the wrong cloud → divergent or stuck. PF
     ESS drops to 1.
6. **`underconfident`** - convergence is slow but reliable for all filters.
   PF benefits from the wide prior (truth is in support).
7. **`kidnapped`** - only PF (uniform init, N=3000) can globally localise;
   KF/EKF stay biased toward the (wrong) Gaussian prior centre.

These hypotheses become the experimental analysis section of the paper.

---

## Files added/changed for this experiment

| Path | Purpose |
|---|---|
| `ws/src/pro_lab_filters/include/pro_lab_filters/PF.h` | added Uniform init + ESS |
| `ws/src/pro_lab_filters/src/pf_node.cpp` | publish `/pf/particles`, `/pf/ess`, init_distribution param |
| `ws/src/pro_lab_filters/src/kf_node.cpp` | seed-based init sampling for 10-run sweeps |
| `ws/src/pro_lab_filters/src/ekf_node.cpp` | seed-based init sampling, landmark fusion |
| `ws/src/pro_lab_filters/src/ekf_lf_node.cpp` | EKF-LF: scan-likelihood update via shared LikelihoodField, direct /scan + /map consumption |
| `ws/src/pro_lab_filters/include/pro_lab_filters/EKF.h` | added `updateScanLikelihood` (Probabilistic Robotics §7.4) |
| `ws/src/pro_lab_filters/src/trajectory_player_node.cpp` | scripted /cmd_vel sequence |
| `ws/src/pro_lab_filters/src/truth_relay_node.cpp` | TF → `/ground_truth/pose` |
| `ws/src/pro_lab_filters/src/metrics_node.cpp` | per-filter error / RMSE / convergence + NEES |
| `ws/src/pro_lab_filters/scripts/csv_logger.py` | CSV time-series + summary, seed-suffixed |
| `ws/src/pro_lab_filters/scripts/run_wrong_init_batch.sh` | batch driver |
| `ws/src/pro_lab_filters/config/scenarios/*.yaml` | 7 scenarios |
| `ws/src/pro_lab_filters/config/wrong_init.rviz` | RViz layout (TB4 + map + scan + 4 poses + cov ellipses + particles) |
| `ws/src/pro_lab_filters/launch/wrong_init_experiment.launch.py` | experiment launcher (KF, EKF, PF, AMCL relay, trajectory_player, seed) |
| `ws/src/pro_lab_filters/launch/all_in_one.launch.py` | single-container launch: Gazebo + bridge + Nav2 + RViz |
| `scripts/run_experiments.sh` | top-level pipeline: scenarios × seeds, --headless toggle |
| `docker/Dockerfile` | unified container: Gazebo + bridge + Nav2 + RViz + filters |
| `ws/src/pro_lab_filters/CMakeLists.txt` | builds new executables |
| `ws/src/pro_lab_filters/package.xml` | adds tf2_ros, rclpy deps |
