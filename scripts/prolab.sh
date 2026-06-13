#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# PRO-LAB launcher — a FastMCP-style terminal front-end for the whole study.
# Boxed banner + live container status + a menu that starts every pipeline
# (live demo, single run, multi-seed sweep, analysis, paper build).
#
#   bash scripts/prolab.sh
#
# Pure bash + ANSI, no dependencies. All actions just dispatch to the dedicated
# scripts in this folder, so the pipelines stay usable on their own too.
# ─────────────────────────────────────────────────────────────────────────────
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CONTAINER=${CONTAINER:-prolab_jazzy}

# ── colours ──────────────────────────────────────────────────────────────────
B="\033[1m"; D="\033[2m"; R="\033[0m"
CY="\033[36m"; GR="\033[32m"; YE="\033[33m"; RE="\033[31m"; MG="\033[35m"; BL="\033[34m"

c_status() {                       # → "running" / "stopped" (+ colour)
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then
    printf "%brunning%b" "$GR" "$R"
  elif docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then
    printf "%bstopped%b" "$YE" "$R"
  else
    printf "%bnot found%b" "$RE" "$R"
  fi
}

banner() {
  clear
  # blue→cyan gradient across the six logo rows (256-colour)
  local g1='\033[38;5;27m' g2='\033[38;5;33m' g3='\033[38;5;39m'
  local g4='\033[38;5;45m' g5='\033[38;5;51m' g6='\033[38;5;50m'
  echo
  printf "    %b██████╗ ██████╗  ██████╗      ██╗      █████╗ ██████╗ %b\n" "$g1" "$R"
  printf "    %b██╔══██╗██╔══██╗██╔═══██╗     ██║     ██╔══██╗██╔══██╗%b\n" "$g2" "$R"
  printf "    %b██████╔╝██████╔╝██║   ██║     ██║     ███████║██████╔╝%b\n" "$g3" "$R"
  printf "    %b██╔═══╝ ██╔══██╗██║   ██║     ██║     ██╔══██║██╔══██╗%b\n" "$g4" "$R"
  printf "    %b██║     ██║  ██║╚██████╔╝     ███████╗██║  ██║██████╔╝%b\n" "$g5" "$R"
  printf "    %b╚═╝     ╚═╝  ╚═╝ ╚═════╝      ╚══════╝╚═╝  ╚═╝╚═════╝ %b\n" "$g6" "$R"
  echo
  printf "    %bProbabilistic Robotics%b  %b·%b  %bWrong Initialization%b\n" \
         "$B$CY" "$R" "$D" "$R" "$B$MG" "$R"
  printf "    %b────────────────────────────────────────────────────────────%b\n" "$D" "$R"
  printf "      %bTask%b       2510331021 · Wrong Initialization\n" "$D" "$R"
  printf "      %bRobot%b      TurtleBot 4 · ROS 2 Jazzy · Gazebo Harmonic\n" "$D" "$R"
  printf "      %bFilters%b    %bKF%b · %bEKF%b · %bEKF-LF%b · %bPF%b · %bAMCL%b   %b(KF/EKF self-implemented, C++)%b\n" \
         "$D" "$R" "$BL" "$R" "$GR" "$R" "$MG" "$R" "$RE" "$R" "$YE" "$R" "$D" "$R"
  printf "      %bLandmarks%b  8× AprilTag 36h11, read by the OAK-D camera\n" "$D" "$R"
  printf "      %bContainer%b  %s  (%s)\n" "$D" "$R" "$CONTAINER" "$(c_status)"
  echo
}

menu() {
  printf "   %bPipelines%b\n" "$B" "$R"
  printf "     %b1%b  Live demo        kidnap + teleop in RViz, watch filters recover\n" "$CY" "$R"
  printf "     %b2%b  Single run       one scenario + filters, writes CSV + opens plots\n" "$CY" "$R"
  printf "     %b3%b  Multi-seed sweep 7 scenarios × 10 seeds (~90 min, the paper data)\n" "$CY" "$R"
  printf "     %b4%b  Analyze + plots  regenerate all plots from the sweep results\n" "$CY" "$R"
  printf "   %bUtilities%b\n" "$B" "$R"
  printf "     %bk%b  Kill sim         stop the running simulation and all nodes\n" "$CY" "$R"
  printf "     %bq%b  Quit\n" "$CY" "$R"
  echo
  printf "   %bselect ▸ %b" "$B" "$R"
}

demo_help() {
  printf "\n   %b┌─ LIVE DEMO — \"landmarks are king\" ─────────────────────────┐%b\n" "$MG" "$R"
  printf "   %b│%b  1. DRIVE   use the Teleop panel (or arrow keys) to move\n" "$MG" "$R"
  printf "   %b│%b  2. KIDNAP  pick the Kidnap tool, drag an arrow → robot warps\n" "$MG" "$R"
  printf "   %b│%b  3. LANDMARKS WIN  keep driving: KF/EKF snap back on their own\n" "$MG" "$R"
  printf "   %b│%b             as soon as the camera reads a tag\n" "$MG" "$R"
  printf "   %b│%b  4. SCAN NEEDS HELP  PF/AMCL stay lost (aliasing); rescue them\n" "$MG" "$R"
  printf "   %b│%b             with the 2D Pose Estimate tool (→ /initialpose)\n" "$MG" "$R"
  printf "   %b└────────────────────────────────────────────────────────────┘%b\n\n" "$MG" "$R"
}

kill_sim() {
  printf "\n   stopping the simulation (clean container reset)…\n"
  if docker restart "$CONTAINER" >/dev/null 2>&1; then
    printf "   %bsim stopped%b — container idle, ready to launch again\n" "$GR" "$R"
  else
    printf "   %bcould not reach container %s%b\n" "$RE" "$CONTAINER" "$R"
  fi
}

pause() { printf "\n   %bpress Enter to return to the menu…%b" "$D" "$R"; read -r _; }

while true; do
  banner
  menu
  read -r choice
  case "${choice:-}" in
    1) demo_help; bash "$HERE/run_demo.sh"; pause ;;
    2) printf "\n   scenario [correct_init]: "; read -r sc
       bash "$HERE/run_single.sh" --filter all --scenario "${sc:-correct_init}"; pause ;;
    3) printf "\n   %b⚠ runs ~90 min, 70 launches.%b start? [y/N] " "$YE" "$R"; read -r ok
       [[ "${ok:-}" == "y" ]] && bash "$HERE/run_wrong_init_sweep.sh"; pause ;;
    4) python3 "$HERE/analyze_results.py" --in "$HERE/../results/sweep" \
         --out "$HERE/../results/wrong_init/plots" && bash "$HERE/copy_fav_plots.sh"; pause ;;
    k|K) kill_sim; pause ;;
    q|Q|"") clear; printf "%bbye 👋%b\n" "$CY" "$R"; exit 0 ;;
    *) ;;
  esac
done
