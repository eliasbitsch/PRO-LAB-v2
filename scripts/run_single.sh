#!/usr/bin/env bash
# Run ONE experiment in the prolab_jazzy container with RViz visible.
# Cleans up stale processes first, then launches inside the container.
# Use this for interactive debugging - for the full statistical sweep
# use scripts/run_experiments.sh instead.
#
# Usage:
#   bash scripts/run_single.sh                       # default: kf, correct_init, no auto-shutdown
#   bash scripts/run_single.sh --filter ekf
#   bash scripts/run_single.sh --filter all --scenario offset_5m
#   bash scripts/run_single.sh --filter pf --duration 60
#   bash scripts/run_single.sh --no-rviz --headless  # background-friendly
#   bash scripts/run_single.sh --no-amcl --no-nav2   # minimal stack (KF/EKF/PF only)
#
# Filters:
#   kf      | ekf      | ekf_lf  | pf  | all (default: kf)
#
# After Ctrl+C, run this script again to relaunch - it cleans up by itself.
set -uo pipefail

FILTER="kf"
SCENARIO="correct_init"
DURATION=0
USE_RVIZ="true"
USE_NAV2="false"
USE_AMCL="false"
USE_TRAJECTORY="true"
SEED=0
OUT_DIR="/home/ros/results"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --filter)     FILTER="$2"; shift 2 ;;
    --scenario)   SCENARIO="$2"; shift 2 ;;
    --duration)   DURATION="$2"; shift 2 ;;
    --seed)       SEED="$2"; shift 2 ;;
    --no-rviz)    USE_RVIZ="false"; shift ;;
    --no-trajectory) USE_TRAJECTORY="false"; shift ;;
    --nav2)       USE_NAV2="true"; shift ;;
    --no-nav2)    USE_NAV2="false"; shift ;;
    --amcl)       USE_AMCL="true"; USE_NAV2="true"; shift ;;
    --no-amcl)    USE_AMCL="false"; shift ;;
    -h|--help)    sed -n '2,21p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

c_cyan() { printf '\033[36m%s\033[0m\n' "$*"; }
c_red()  { printf '\033[31m%s\033[0m\n' "$*" >&2; }

# Pre-create the bind-mounted results dir so docker doesn't auto-create
# it as root. Image runs as the host user, so nothing else needed.
HOST_RESULTS_DIR="$(cd "$(dirname "$0")/.." && pwd)/results"
mkdir -p "$HOST_RESULTS_DIR"

if ! docker ps --format '{{.Names}}' | grep -q '^prolab_jazzy$'; then
  c_red "container prolab_jazzy is not running. Start it with:"
  c_red "  cd docker && docker compose up -d"
  exit 1
fi

# Cleanup helper - kills everything ros/gz/rviz spawns inside the container.
# Idempotent, safe to call multiple times.
cleanup_container() {
  docker exec prolab_jazzy bash -c '
    pkill -9 -f "ros2 launch"     2>/dev/null
    pkill -9 -f gz                2>/dev/null
    pkill -9 -f rviz2             2>/dev/null
    pkill -9 -f kf_node           2>/dev/null
    pkill -9 -f ekf_node          2>/dev/null
    pkill -9 -f ekf_lf_node       2>/dev/null
    pkill -9 -f pf_node           2>/dev/null
    pkill -9 -f trajectory_player 2>/dev/null
    pkill -9 -f truth_relay       2>/dev/null
    pkill -9 -f metrics_node      2>/dev/null
    pkill -9 -f csv_logger        2>/dev/null
    pkill -9 -f landmark_detector 2>/dev/null
    pkill -9 -f map_server        2>/dev/null
    pkill -9 -f lifecycle_manager 2>/dev/null
    pkill -9 -f cmd_vel_watchdog  2>/dev/null
    pkill -9 -f map_odom          2>/dev/null
    pkill -9 -f static_transform  2>/dev/null
    pkill -9 -f robot_state       2>/dev/null
    pkill -9 -f parameter_bridge  2>/dev/null
    pkill -9 -f teleporter        2>/dev/null
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
    true
  ' >/dev/null 2>&1 || true
}

# Trap Ctrl+C and normal exit so the container's ros/gz processes don't
# survive after the host script dies. Without this, `docker exec` forwards
# SIGINT to the ros2 launch parent but its many children (gz, rviz2, the
# bridge, nodes) become orphans and keep eating CPU + holding ports.
on_exit() {
  local rc=$?
  trap - INT TERM EXIT
  c_cyan "→ Cleaning up container processes..."
  cleanup_container
  exit "$rc"
}
trap on_exit INT TERM EXIT

c_cyan "→ killing stale ros/gz processes in container..."
cleanup_container
sleep 1

c_cyan "→ filter=$FILTER  scenario=$SCENARIO  duration=${DURATION}s  rviz=$USE_RVIZ  seed=$SEED"
c_cyan "  nav2=$USE_NAV2  amcl=$USE_AMCL  trajectory=$USE_TRAJECTORY"
c_cyan "  results -> ./results/ (bind-mounted from container)"
echo

# -t allocates a TTY so SIGINT propagates from this shell to the in-container
# ros2 launch process; the trap above then guarantees its children get reaped
# even if launch's own shutdown sequence is incomplete.
docker exec -t prolab_jazzy bash -lc "
  source /opt/ros/jazzy/setup.bash &&
  source /home/ros/ws/install/setup.bash &&
  exec ros2 launch pro_lab_filters wrong_init_experiment.launch.py \
    scenario:=$SCENARIO \
    filter:=$FILTER \
    duration_s:=$DURATION \
    seed:=$SEED \
    use_rviz:=$USE_RVIZ \
    start_gz:=true \
    use_nav2:=$USE_NAV2 \
    use_amcl:=$USE_AMCL \
    use_trajectory:=$USE_TRAJECTORY \
    out_dir:=$OUT_DIR
"
