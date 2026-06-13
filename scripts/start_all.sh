#!/usr/bin/env bash
# PRO-LAB one-shot launcher.
#
# Idempotent: safe to run multiple times. Stops stale processes first,
# then brings everything up in dependency order with readiness checks
# instead of fixed sleeps. Aborts loudly on the first failure.
#
# Pipeline:
#   1) Stop any old gz sim, ros2 launch, container processes.
#   2) (no-op - Gazebo runs inside the container, see step 5)
#   3) Start docker stack (prolab only).
#   4) Build workspace inside the prolab container.
#   5) Launch wrong_init_experiment ROS stack inside the container.
#      Wait until the selected filter's pose topic actually publishes.
#   6) RViz opens via the launch file (use_rviz:=true).
#
# Anything broken? -> output points at the right log:
#     /tmp/build.log      (colcon build inside container)
#     /tmp/launch.log     (ros2 launch inside container)
#
# Usage:
#   bash scripts/start_all.sh
#   bash scripts/start_all.sh --scenario offset_5m
#   bash scripts/start_all.sh --filter kf
#   bash scripts/start_all.sh --skip-build

set -uo pipefail

# ---------- args -------------------------------------------------------------

SCENARIO="correct_init"
VIZ="rviz"               # rviz | none
FILTER="all"             # kf | ekf | pf | all
SKIP_BUILD=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)      SCENARIO="$2"; shift 2 ;;
    --viz)           VIZ="$2"; shift 2 ;;
    --rviz)          VIZ="rviz"; shift ;;
    --no-rviz)       VIZ="none"; shift ;;
    --filter)        FILTER="$2"; shift 2 ;;
    --skip-build)    SKIP_BUILD=1; shift ;;
    -h|--help)
      sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

case "$FILTER" in
  kf|ekf|pf|all) ;;
  *) echo "--filter must be kf|ekf|pf|all, got: $FILTER" >&2; exit 2 ;;
esac

case "$VIZ" in
  rviz|none) ;;
  *) echo "--viz must be rviz|none, got: $VIZ" >&2; exit 2 ;;
esac

USE_RVIZ=false
[[ "$VIZ" == "rviz" ]] && USE_RVIZ=true

REPO="$(cd "$(dirname "$0")/.." && pwd)"

# ---------- helpers ----------------------------------------------------------

c_cyan()  { printf '\033[36m%s\033[0m\n' "$*"; }
c_gray()  { printf '\033[90m%s\033[0m\n' "$*"; }
c_green() { printf '\033[32m%s\033[0m\n' "$*"; }
c_red()   { printf '\033[31m%s\033[0m\n' "$*" >&2; }

in_container() {
  # Run a bash one-liner inside prolab_jazzy, ROS sourced.
  docker exec prolab_jazzy bash -lc "
    source /opt/ros/jazzy/setup.bash
    [ -f /home/ros/ws/install/setup.bash ] && source /home/ros/ws/install/setup.bash
    $*
  "
}

wait_for() {
  # wait_for <timeout_sec> <description> <probe_cmd...>
  local timeout="$1"; shift
  local what="$1"; shift
  local deadline=$(( $(date +%s) + timeout ))
  local tries=0
  while (( $(date +%s) < deadline )); do
    tries=$((tries+1))
    if eval "$@" >/dev/null 2>&1; then
      c_gray "      ready after $tries tries"
      return 0
    fi
    sleep 2
  done
  c_red "timeout waiting for: $what (after ${timeout}s)"
  return 1
}

# ---------- 1) cleanup -------------------------------------------------------

c_cyan "[1/6] Cleaning up stale processes..."
# Kill any leftover host gz (only relevant on WSL setups; harmless on Linux).
pkill -9 -f 'gz sim'      2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
# Pkill from inside the container often misses launch-supervised respawns
# and leaks bridges/map_servers across runs, which causes duplicate /clock
# publishers and disconnected TF trees. Restarting the container itself is
# the only reliable nuke. Cheap (~3s), idempotent, and required.
if docker ps -q -f name=prolab_jazzy | grep -q .; then
  docker restart prolab_jazzy >/dev/null
fi
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 3

# ---------- 2) (gz now runs inside the container - see step 5) --------------

c_gray "[2/6] Skipping native Gazebo - gz starts inside prolab container via start_gz:=true."

# ---------- 3) Docker stack --------------------------------------------------

c_cyan "[3/6] Starting docker stack..."
( cd "$REPO/docker" && docker compose up -d prolab ) >/dev/null

wait_for 30 "prolab_jazzy container running" \
  "[ \"\$(docker inspect -f '{{.State.Running}}' prolab_jazzy 2>/dev/null)\" = true ]" \
  || exit 1

# Allow the container's X clients (RViz, gz GUI) to talk to the host X server.
# Idempotent - does nothing if X isn't reachable (e.g. headless server).
if [[ "$USE_RVIZ" == "true" ]] && command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

# Named volumes are created with root ownership by default. The container runs
# as uid 1001 (ros), so colcon can't write to build/log/install. Fix every run
# (idempotent - chown is a no-op if already correct).
docker exec --user root prolab_jazzy bash -lc '
  chown -R ros:ros /home/ros/ws/build /home/ros/ws/install /home/ros/ws/log /home/ros/.gz 2>/dev/null || true
' >/dev/null

# ---------- 4) Build ---------------------------------------------------------

if (( SKIP_BUILD == 0 )); then
  c_cyan "[4/6] Building workspace (colcon)..."
  if ! in_container "
        cd /home/ros/ws &&
        colcon build --symlink-install --packages-select pro_lab_filters \
          > /tmp/build.log 2>&1 && echo BUILD_OK
      " | grep -q BUILD_OK
  then
    c_red "BUILD FAILED - last lines of /tmp/build.log:"
    in_container "tail -30 /tmp/build.log" >&2 || true
    exit 1
  fi
else
  c_gray "[4/6] Skipping build (--skip-build)"
fi

# ---------- 5) Launch ROS stack ---------------------------------------------

c_cyan "[5/6] Launching wrong_init_experiment (scenario=$SCENARIO, filter=$FILTER, viz=$VIZ)..."
docker exec -d prolab_jazzy bash -lc "
  cd /home/ros/ws &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  ros2 launch pro_lab_filters wrong_init_experiment.launch.py \
    scenario:=$SCENARIO filter:=$FILTER \
    use_rviz:=$USE_RVIZ \
    start_gz:=true use_nav2:=false \
    > /tmp/launch.log 2>&1
"

# Probe the topic that actually corresponds to the selected filter; falls back
# to /pf/pose for the all-filters case to preserve previous behaviour.
case "$FILTER" in
  kf)  PROBE_TOPIC="/kf/pose" ;;
  ekf) PROBE_TOPIC="/ekf/pose" ;;
  pf|all) PROBE_TOPIC="/pf/pose" ;;
esac

c_gray "      waiting for $PROBE_TOPIC to publish (non-fatal)..."
# `ros2 topic echo --once` blocks until exactly one message arrives, which
# is much more reliable than `topic hz` (the latter needs to sample a
# whole window before printing "average rate", easily exceeding a short
# probe timeout even when the topic is publishing fine).
if ! wait_for 30 "$PROBE_TOPIC publishing" \
     "in_container 'timeout 5 ros2 topic echo --once $PROBE_TOPIC > /dev/null 2>&1'"
then
  c_red "WARN: $PROBE_TOPIC did not publish in 30s - opening viz anyway so you can debug."
  c_gray "      tail -f /tmp/launch.log to investigate."
fi

# ---------- 6) Browser -------------------------------------------------------

if [[ "$VIZ" == "rviz" ]]; then
  c_gray "[6/6] viz=rviz - RViz window opened by launch file"
else
  c_gray "[6/6] viz=none - skipping visualization"
fi

echo
c_green "Stack is up. scenario=$SCENARIO filter=$FILTER"
c_gray "Tail logs:"
c_gray "  docker exec prolab_jazzy tail -f /tmp/launch.log"
