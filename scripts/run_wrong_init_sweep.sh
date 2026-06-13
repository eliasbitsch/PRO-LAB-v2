#!/bin/bash
# Serial 10-seed x 7-scenario sweep for the wrong-init study (AprilTag pipeline).
#
# Runs on the HOST (orchestrates the container) because the only RELIABLE reset
# between runs is `docker restart` - pkill -f ros2 leaves the spawned node
# processes (ekf_node, apriltag_landmark_detector_node, ...) alive, and stacked
# launch generations corrupt the next run (/clock chaos, "moved backwards in
# time", duplicated nodes). Restarting the container is the only thing that
# guarantees a clean slate. Provisioning (camera + libapriltag) and the build/
# install volumes survive a restart, so it's cheap (~4 s).
#
# Usage:
#   bash scripts/run_wrong_init_sweep.sh
# Env overrides:
#   SEEDS="1 2 3"      seeds to run (default 1..10)
#   SCENARIOS="..."    space-separated scenario list (default all 7)
#   DURATION=60        sim seconds per run
#   RUN_TIMEOUT=200    max wall-seconds to wait for a run's summary CSV
#   OUT=/home/ros/results/sweep   container out dir (host: results/sweep)
set -u

CONTAINER=${CONTAINER:-prolab_jazzy}
SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}
DURATION=${DURATION:-60}
RUN_TIMEOUT=${RUN_TIMEOUT:-200}
OUT=${OUT:-/home/ros/results/sweep}
HOST_OUT=/home/elias/git/PRO-LAB/results/sweep   # mount of $OUT

read -r -a SCEN <<< "${SCENARIOS:-correct_init offset_1m offset_5m wrong_yaw_pi2 overconfident_wrong underconfident kidnapped}"
read -r -a SEED_ARR <<< "$SEEDS"

mkdir -p "$HOST_OUT"
docker exec "$CONTAINER" bash -lc "mkdir -p $OUT" >/dev/null 2>&1

total=$(( ${#SCEN[@]} * ${#SEED_ARR[@]} ))
count=0
ok=0
fail=0
start_epoch=$(date +%s)
echo "=== sweep: ${#SCEN[@]} scenarios x ${#SEED_ARR[@]} seeds = $total runs @ ${DURATION}s, out=$HOST_OUT ==="

for s in "${SCEN[@]}"; do
  for seed in "${SEED_ARR[@]}"; do
    count=$((count + 1))
    suffix=$(printf "_seed%02d" "$seed")
    summary="$HOST_OUT/${s}${suffix}_summary.csv"
    rm -f "$summary"                     # so we wait for the FRESH one
    elapsed=$(( $(date +%s) - start_epoch ))
    printf "[%d/%d | %dm elapsed] %s seed=%d ... " "$count" "$total" "$((elapsed/60))" "$s" "$seed"

    # hard reset to a clean process slate
    docker restart "$CONTAINER" >/dev/null 2>&1
    sleep 4

    docker exec -d "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ros/ws/install/setup.bash; ros2 launch pro_lab_filters wrong_init_experiment.launch.py scenario:=$s seed:=$seed filter:=all use_nav2:=true use_amcl:=true use_rviz:=false use_trajectory:=true gz_gui:=false start_gz:=true duration_s:=$DURATION out_dir:=$OUT > $OUT/${s}${suffix}.log 2>&1"

    waited=0
    while [ ! -f "$summary" ] && [ $waited -lt $RUN_TIMEOUT ]; do
      sleep 3; waited=$((waited + 3))
    done
    if [ -f "$summary" ]; then
      echo "ok (${waited}s)"; ok=$((ok + 1))
    else
      echo "TIMEOUT/MISSING after ${waited}s"; fail=$((fail + 1))
    fi
  done
done

docker restart "$CONTAINER" >/dev/null 2>&1   # leave a clean container
dur=$(( ($(date +%s) - start_epoch) / 60 ))
echo "=== sweep done in ${dur}m: $ok ok, $fail failed, out=$HOST_OUT ==="
echo "next: python3 scripts/analyze_results.py --in $HOST_OUT  &&  bash scripts/copy_fav_plots.sh"
