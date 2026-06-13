#!/bin/bash
# Copies the 7 paper-essential plots into results/wrong_init/fav_plots/.
# Run AFTER analyze_results.py has populated results/wrong_init/plots/.
#
# Usage: bash scripts/copy_fav_plots.sh
set -e

SRC=/home/elias/git/PRO-LAB/results/wrong_init/plots
DST=/home/elias/git/PRO-LAB/results/wrong_init/fav_plots
mkdir -p "$DST"

FAVS=(
    # Aggregate — main results
    rmse_comparison.png
    convergence_rate.png
    runtime_comparison.png
    # Method illustration — lecture-style 3-panel, one per required filter
    correct_init_kf_explainer.png
    correct_init_ekf_explainer.png
    correct_init_pf_explainer.png
    # Error vs uncertainty pairing: actual error over time (hero scenario)
    # + the filters' own believed uncertainty (baseline scenario, incl. the
    # unbounded dead-reckoning growth for contrast)
    wrong_yaw_pi2_error_xy.png
    correct_init_uncertainty.png
    # Kidnapped: per-filter localization plots (K1/K2 markers + gradient paths)
    kidnapped_kf_localization.png
    kidnapped_ekf_localization.png
    kidnapped_ekf_lf_localization.png
    kidnapped_pf_localization.png
    kidnapped_amcl_localization.png
    # Raw aggregate data for tables
    all_summaries.csv
)

for f in "${FAVS[@]}"; do
    if [ -f "$SRC/$f" ]; then
        cp "$SRC/$f" "$DST/"
        echo "  copied $f"
    else
        echo "  MISSING $f"
    fi
done

echo "→ $(ls "$DST" | wc -l) files in $DST"
