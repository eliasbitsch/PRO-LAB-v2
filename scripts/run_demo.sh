#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# LIVE DEMO — "Landmarks are king": kidnap + manual recovery in RViz.
# ─────────────────────────────────────────────────────────────────────────────
# Launches the full stack in MANUAL mode (no scripted trajectory): Gazebo GUI +
# RViz + all four research filters (KF, EKF, EKF-LF, PF) + the Nav2 AMCL
# baseline, all starting correctly localized. You then drive and kidnap the
# robot by hand and watch how the filters cope.
#
# Runs on the HOST and drives the prolab_jazzy container. A clean container
# restart first guarantees no stale nodes (see scripts/run_wrong_init_sweep.sh
# for why pkill alone is not enough).
#
# ── How to run ──────────────────────────────────────────────────────────────
#   bash scripts/run_demo.sh
# (needs `xhost +local:` once on the host so the container can open windows)
#
# ── How to use the demo (the story) ─────────────────────────────────────────
#   1. DRIVE:    use the "Teleop" panel in RViz (or teleop_twist_keyboard on
#                /cmd_vel_in). All filters track the robot.
#   2. KIDNAP:   pick the "Kidnap" tool in the RViz toolbar and click+drag an
#                arrow somewhere on the map — the robot teleports there. The
#                belief of every filter is now wrong.
#   3. RECOVER (landmarks are king): just keep driving. As soon as the camera
#                reads an AprilTag at the new location, KF and EKF snap back to
#                the truth on their own — no human help needed.
#   4. SCAN FILTERS NEED HELP: PF and AMCL stay lost in the symmetric warehouse
#                (perceptual aliasing). Give them a good initial guess with the
#                "2D Pose Estimate" tool (publishes /initialpose) — both re-seed
#                and converge. This is the textbook "particle filters need a
#                reasonable initial belief" point, shown live.
#
# Press Ctrl+C in this terminal (or just close the windows) to stop; re-run to
# restart cleanly.
# ─────────────────────────────────────────────────────────────────────────────
set -uo pipefail

CONTAINER=${CONTAINER:-prolab_jazzy}
SCENARIO=${SCENARIO:-correct_init}   # start correctly localized, then kidnap by hand
DISPLAY_ENV=${DISPLAY:-:1}

echo "── allowing local X connections for the container ──"
xhost +local: >/dev/null 2>&1 || echo "  (xhost not available — windows may not open)"

echo "── clean container restart (no stale nodes) ──"
docker restart "$CONTAINER" >/dev/null 2>&1
sleep 4

echo "── launching demo (Gazebo GUI + RViz + KF/EKF/EKF-LF/PF + AMCL, MANUAL) ──"
echo "   drive with the Teleop panel, kidnap with the Kidnap tool,"
echo "   help PF/AMCL with 2D Pose Estimate. KF/EKF recover by themselves."
docker exec -d "$CONTAINER" bash -lc "export DISPLAY=$DISPLAY_ENV; \
  source /opt/ros/jazzy/setup.bash; source /home/ros/ws/install/setup.bash; \
  ros2 launch pro_lab_filters wrong_init_experiment.launch.py \
    scenario:=$SCENARIO filter:=all \
    use_nav2:=true use_amcl:=true use_rviz:=true \
    use_trajectory:=false gz_gui:=true start_gz:=true duration_s:=0 \
    truth_tf:=true \
    > /home/ros/results/demo.log 2>&1"

echo "── demo starting; windows appear in ~15-20 s. Log: results/demo.log ──"
