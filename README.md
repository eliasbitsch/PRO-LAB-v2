# PRO-LAB - Bad Beginnings 🤖

**Which localization filter survives a wrong start?** A controlled, reproducible
comparison of five Bayesian filters (KF · EKF · EKF-LF · PF · AMCL) under seven
*deliberately wrong* initializations on a TurtleBot 4 in a perceptually
symmetric warehouse - ROS 2 Jazzy + Gazebo Harmonic.

> **Finding:** the *observation model*, not the filter, decides recovery.
> Filters that read **identifiable AprilTag landmarks** recover from large
> offsets, wrong heading, and even double kidnapping; **scan-based** filters
> (including the Nav2 AMCL baseline) are defeated by perceptual aliasing and
> diverge by up to **17 m**. Yet under a *correct* start all five agree to
> within 15 cm - so the landmark advantage is specifically **robustness to bad
> initialization**, not nominal accuracy.

> Elias Bitsch · FH Technikum Wien · Group 2B1 · Task **2510331021 - Wrong Initialization**

---

## The result (RMSE_xy [m], mean ± 1σ over 10 seeds)

| Scenario | KF 🟦 | EKF 🟩 | EKF-LF 🟪 | PF 🟥 | AMCL 🟧 |
|---|---|---|---|---|---|
| `correct_init` | **0.11** | 0.14 | 0.15 | 0.14 | 0.15 |
| `offset_5m` | **0.30** | 0.30 | 5.54 | 3.58 | 4.13 |
| `overconfident_wrong` | **0.29** | 0.38 | 5.05 | 3.40 | 2.98 |
| `kidnapped` | **2.62** | 2.99 | 17.47 | 9.21 | 15.18 |

🟦🟩 = landmark filters (AprilTag) · 🟪🟥🟧 = scan filters. Full 7-scenario table
and plots in [`results/wrong_init/fav_plots/`](results/wrong_init/fav_plots/),
write-up in [`ProbRob_Paper_Template_Englisch/paper.pdf`](ProbRob_Paper_Template_Englisch/).

---

## Quick start

Everything runs in the `prolab_jazzy` Docker container (ROS 2 Jazzy + Gazebo
Harmonic, NVIDIA GPU passthrough - see [`docker/`](docker/)). One launcher drives
every pipeline:

```bash
bash scripts/prolab.sh
```

```
    ██████╗ ██████╗  ██████╗      ██╗      █████╗ ██████╗
    ██╔══██╗██╔══██╗██╔═══██╗     ██║     ██╔══██╗██╔══██╗
    ██████╔╝██████╔╝██║   ██║     ██║     ███████║██████╔╝
    ██╔═══╝ ██╔══██╗██║   ██║     ██║     ██╔══██║██╔══██╗
    ██║     ██║  ██║╚██████╔╝     ███████╗██║  ██║██████╔╝
    ╚═╝     ╚═╝  ╚═╝ ╚═════╝      ╚══════╝╚═╝  ╚═╝╚═════╝

    Probabilistic Robotics  ·  Wrong Initialization
    ────────────────────────────────────────────────────────
      1  Live demo   2  Single run   3  Multi-seed sweep
      4  Analyze     k  Kill sim     q  Quit
```

First time only, provision the container (enables the camera and installs the
AprilTag library - these live outside the mounted workspace, so re-run after an
image rebuild):

```bash
docker exec -u 0 prolab_jazzy bash /home/ros/ws/src/pro_lab_filters/scripts/provision_container.sh
```

### The three pipelines

| | Script | What it does |
|---|---|---|
| 🎮 **Demo** | `scripts/run_demo.sh` | Manual RViz session: drive with the teleop panel, **kidnap** the robot with the Kidnap arrow tool, watch KF/EKF recover on their own while you rescue PF/AMCL with a **2D Pose Estimate**. |
| 🎯 **Single run** | `scripts/run_single.sh --filter all --scenario offset_5m` | One scenario, all filters, writes CSVs for inspection. |
| 📊 **Multi-seed sweep** | `scripts/run_wrong_init_sweep.sh` | The paper data: 7 scenarios × 10 seeds (~90 min), container-restarted between runs for a clean slate. |

Then analyze and build the paper:

```bash
python3 scripts/analyze_results.py --in results/sweep --out results/wrong_init/plots
bash scripts/copy_fav_plots.sh
( cd ProbRob_Paper_Template_Englisch && bash build.sh paper )   # → paper.pdf
```

---

## How it works

**Filters** - ROS-free core classes plus thin ROS 2 nodes, implemented from
first principles in C++:

* **KF** - 6-D constant-velocity linear Kalman filter (reference baseline).
* **EKF** - 3-D unicycle model, corrected by AprilTag range/bearing landmarks.
* **EKF-LF** - EKF corrected by a lidar **likelihood field** (scan matching).
* **PF** - Monte Carlo localization with a likelihood field and **augmented
  MCL** random-particle injection for kidnap recovery; re-seedable from a
  `/initialpose` (RViz "2D Pose Estimate").
* **AMCL** - the Nav2 adaptive Monte Carlo baseline.

**Honest landmark identity.** Each warehouse pillar carries a unique **AprilTag
36h11** marker. The robot's OAK-D camera detects the tags (AprilTag-3 + PnP) and
recovers their range, bearing, and **identity** - accurate to <1 cm / <0.25° vs.
ground truth. The identity is *measured*, never taken from an oracle; ground
truth is used **only** to score error, never as a filter input.

**Why scan filters fail.** The 8 pillars sit in a symmetric 2×4 grid at 7.5 m
spacing, so a one-bay displacement produces a near-identical scan (*perceptual
aliasing*). Scan filters lock onto a wrong-but-symmetric hypothesis and need a
human-supplied initial guess to escape; landmark filters sidestep this entirely
because the tag identity names the correct pillar.

---

## Repository layout

```
ws/src/pro_lab_filters/
├── include/pro_lab_filters/   KF.h · EKF.h · PF.h · LikelihoodField.h   (filter cores)
├── src/                       *_node.cpp - filters, AprilTag detector, metrics,
│                              csv_logger, amcl_relay/init, auto_kidnapper, …  (all C++)
├── launch/                    wrong_init_experiment.launch.py  (master)
├── config/                    scenarios/*.yaml · aruco/ (8× tag png+sdf+dae) · *.rviz
└── scripts/                   gen_marker_models.py · provision_container.sh
scripts/                       prolab.sh · run_demo.sh · run_single.sh ·
                               run_wrong_init_sweep.sh · analyze_results.py
ProbRob_Paper_Template_Englisch/   IEEE paper (paper.tex → paper.pdf)
results/                        sweep/ (raw CSVs) · wrong_init/ (plots)  - git-ignored
```

Per the assignment, the filter core logic is implemented from first principles in
C++; only the launch files and the plotting/analysis stay in Python.

### Scenarios

`correct_init` · `offset_1m` · `offset_5m` · `wrong_yaw_pi2` ·
`overconfident_wrong` · `underconfident` · `kidnapped` (two unannounced teleports).

---

## Documentation

* 📄 **Paper** - [`ProbRob_Paper_Template_Englisch/paper.pdf`](ProbRob_Paper_Template_Englisch/)
* 📈 **Plots** - [`results/wrong_init/fav_plots/`](results/wrong_init/fav_plots/)
* 📝 **Run protocol & notes** - [`docs/`](docs/)
