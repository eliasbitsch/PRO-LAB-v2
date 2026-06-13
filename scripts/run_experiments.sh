#!/usr/bin/env bash
# Run every (scenario × seed) combination for `duration` seconds, dropping
# CSVs into out_dir for later RMSE / convergence analysis. Each run launches
# all four estimators in parallel (KF, EKF, PF, AMCL baseline), the scripted
# trajectory_player, and metrics + csv_logger. The launch file's
# auto-shutdown (duration_s arg) handles teardown.
#
# Usage:
#   bash scripts/run_experiments.sh                       # default 60s × 10 seeds × all scenarios
#   bash scripts/run_experiments.sh --duration 90 --seeds 5
#   bash scripts/run_experiments.sh --scenario offset_5m  # one scenario, all seeds
#   bash scripts/run_experiments.sh --seeds 1             # smoke test (deterministic)
#   bash scripts/run_experiments.sh --headless            # disable RViz (faster)
#
# Without --headless, RViz comes up so you can watch a run live. Gazebo is
# always headless inside the container (gz sim -s). Each run starts gz fresh
# (start_gz:=true) - collisions if we left the previous one alive.
#
# Stack expectation: docker compose has prolab_jazzy up. start_all.sh is not
# required; this script tears down between runs and starts everything itself.
set -uo pipefail

DURATION=40   # 12s warmup + 15s scripted path + ~13s settle
# /home/ros/results is bind-mounted to ./results on the host (see
# docker/docker-compose.yml), so CSVs land in the repo without a docker cp.
OUT_DIR="/home/ros/results"
SCENARIOS_FILTER="*"
SEEDS=10
HEADLESS=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)  DURATION="$2"; shift 2 ;;
    --out)       OUT_DIR="$2";  shift 2 ;;
    --scenario)  SCENARIOS_FILTER="$2"; shift 2 ;;
    --seeds)     SEEDS="$2"; shift 2 ;;
    --headless)  HEADLESS=1; shift ;;
    -h|--help)   sed -n '2,21p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

c_cyan() { printf '\033[36m%s\033[0m\n' "$*"; }
c_green(){ printf '\033[32m%s\033[0m\n' "$*"; }
c_red()  { printf '\033[31m%s\033[0m\n' "$*" >&2; }

# Pre-create ./results on host so docker doesn't auto-create it as root.
# Image is built with the host user's UID/GID (see docker/Dockerfile +
# docker/.env), so the container writes here as the host user - no chmod.
HOST_RESULTS_DIR="$(cd "$(dirname "$0")/.." && pwd)/results"
mkdir -p "$HOST_RESULTS_DIR"

# Discover scenarios from the installed share/ tree (single source of truth).
SCENARIOS=$(docker exec prolab_jazzy bash -lc \
  'ls /home/ros/ws/install/pro_lab_filters/share/pro_lab_filters/config/scenarios/ \
    | sed "s/\.yaml$//"' | tr -d '\r')

USE_RVIZ="true"
[[ $HEADLESS -eq 1 ]] && USE_RVIZ="false"

total=0
for s in $SCENARIOS; do
  [[ "$SCENARIOS_FILTER" != "*" && "$s" != "$SCENARIOS_FILTER" ]] && continue
  for ((seed=1; seed<=SEEDS; seed++)); do
    total=$((total+1))
  done
done

c_cyan "Running $total experiments (each ${DURATION}s, RViz=${USE_RVIZ})"
c_cyan "Outputs -> $OUT_DIR (inside container)"
echo

done_=0
for s in $SCENARIOS; do
  [[ "$SCENARIOS_FILTER" != "*" && "$s" != "$SCENARIOS_FILTER" ]] && continue
  for ((seed=1; seed<=SEEDS; seed++)); do
    done_=$((done_+1))
    c_cyan "[$done_/$total] scenario=$s  seed=$seed"
    docker exec prolab_jazzy bash -lc '
      pkill -9 -f wrong_init_experiment 2>/dev/null
      pkill -9 -f gz                  2>/dev/null
      pkill -9 -f parameter_bridge    2>/dev/null
      pkill -9 -f image_bridge        2>/dev/null
      pkill -9 -f kf_node             2>/dev/null
      pkill -9 -f ekf_node            2>/dev/null
      pkill -9 -f ekf_lf_node         2>/dev/null
      pkill -9 -f pf_node             2>/dev/null
      pkill -9 -f truth_relay         2>/dev/null
      pkill -9 -f map_odom_tf         2>/dev/null
      pkill -9 -f map_server          2>/dev/null
      pkill -9 -f lifecycle_manager   2>/dev/null
      pkill -9 -f csv_logger          2>/dev/null
      pkill -9 -f metrics_node        2>/dev/null
      pkill -9 -f landmark_detector   2>/dev/null
      pkill -9 -f trajectory_player   2>/dev/null
      pkill -9 -f cmd_vel_watchdog    2>/dev/null
      pkill -9 -f robot_teleporter    2>/dev/null
      pkill -9 -f robot_state         2>/dev/null
      pkill -9 -f rviz2               2>/dev/null
      pkill -9 -f amcl                2>/dev/null
      rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
      sleep 2; true
    ' >/dev/null 2>&1
    docker exec prolab_jazzy bash -lc "
      cd /home/ros/ws &&
      source /opt/ros/jazzy/setup.bash &&
      source install/setup.bash &&
      ros2 launch pro_lab_filters wrong_init_experiment.launch.py \
        scenario:=$s filter:=all duration_s:=$DURATION \
        seed:=$seed \
        use_rviz:=$USE_RVIZ start_gz:=true gz_gui:=false use_nav2:=true use_amcl:=true \
        use_trajectory:=true \
        out_dir:=$OUT_DIR \
        > /tmp/exp_${s}_seed${seed}.log 2>&1
    " || c_red "  run failed - see /tmp/exp_${s}_seed${seed}.log inside container"
  done
done

c_green "Done. CSVs in $OUT_DIR (mapped to ./results on host)."
echo "Plot:  python3 scripts/analyze_results.py --in ./results"
