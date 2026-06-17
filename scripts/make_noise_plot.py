#!/usr/bin/env python3
"""Noise-sensitivity figure for the mandatory Q/R experiments.

Two panels share the y-axis (mean NEES, log scale):
  left  = process-noise Q axis   (q_low -> default -> q_high)
  right = measurement-noise R axis (r_low -> default -> r_high)
correct_init serves as the common "default" midpoint on both axes.

NEES (Normalized Estimation Error Squared) over the 3-DOF pose (x, y, yaw) is
the consistency diagnostic: NEES ~ 3 is consistent, NEES >> 3 means the filter
is over-confident (covariance too small), NEES << 3 means it is conservative.
This is exactly the "model confidence (Q) / sensor trust (R)" question the task
asks: too little noise -> over-confident; too much -> conservative.

Reads the per-seed timeseries CSVs in results/noise_qr/ and writes
results/noise_qr/noise_sensitivity.png.
"""
import glob, csv, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

IN = "results/noise_qr"
FILTERS = ["kf", "ekf", "ekf_lf", "pf", "amcl"]
LABELS  = {"kf": "KF", "ekf": "EKF", "ekf_lf": "EKF-LF", "pf": "PF", "amcl": "AMCL"}
COLOURS = {"kf": "tab:blue", "ekf": "tab:green", "ekf_lf": "tab:purple",
           "pf": "tab:red", "amcl": "tab:orange"}
DOF = 3.0
YCAP = 60.0   # display cap; bars above are clipped and annotated

def mean_nees():
    """Per (scenario, filter): mean over time per run, then mean over seeds."""
    from collections import defaultdict
    acc = defaultdict(lambda: defaultdict(list))
    for p in glob.glob(f"{IN}/*_timeseries.csv"):
        s = re.match(r".*/(.+?)_seed\d+_timeseries\.csv", p).group(1)
        with open(p) as f:
            rows = list(csv.DictReader(f))
        for fl in FILTERS:
            col = f"{fl}_nees"
            vals = [float(x[col]) for x in rows
                    if x.get(col) not in (None, "", "nan") and x[col] == x[col]]
            vals = [v for v in vals if np.isfinite(v)]
            if vals:
                acc[s][fl].append(float(np.mean(vals)))
    return {s: {fl: (np.mean(v) if v else np.nan) for fl, v in d.items()}
            for s, d in acc.items()}

def panel(ax, scenarios, ticklabels, title, N, data):
    x = np.arange(len(scenarios))
    w = 0.16
    for i, fl in enumerate(FILTERS):
        vals = [data.get(s, {}).get(fl, np.nan) for s in scenarios]
        disp = [min(v, YCAP) if np.isfinite(v) else np.nan for v in vals]
        bars = ax.bar(x + (i - (N - 1) / 2) * w, disp, w,
                      label=LABELS[fl], color=COLOURS[fl], zorder=3)
        for b, v in zip(bars, vals):
            if np.isfinite(v) and v > YCAP:
                txt = f"{v:.0f}" if v < 1e3 else f"{v:.0e}".replace("e+0", "e").replace("e+", "e")
                ax.annotate(txt, (b.get_x() + b.get_width() / 2, YCAP * 1.05),
                            ha="center", va="bottom", fontsize=7.5, rotation=90,
                            color=b.get_facecolor(), zorder=6, fontweight="bold")
    ax.axhline(DOF, ls="--", lw=1.2, color="0.35", zorder=2)
    ax.text(ax.get_xlim()[1], DOF, " consistent (dof=3)", va="center",
            ha="left", fontsize=8, color="0.35")
    ax.set_yscale("log")
    ax.set_xticks(x); ax.set_xticklabels(ticklabels)
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.grid(axis="y", which="both", alpha=0.25, zorder=0)

def main():
    data = mean_nees()
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(11, 4.4), sharey=True)
    N = len(FILTERS)
    panel(axL, ["q_low", "correct_init", "q_high"],
          ["Q low", "default", "Q high"],
          "Process noise Q - model confidence", N, data)
    panel(axR, ["r_low", "correct_init", "r_high"],
          ["R low", "default", "R high"],
          "Measurement noise R - sensor trust", N, data)
    axL.set_ylabel("mean NEES  (log scale)")
    # one shared legend, outside on the right
    axR.legend(loc="upper left", bbox_to_anchor=(1.13, 1.0), frameon=False,
               fontsize=9)
    over = "over-confident"
    axL.annotate(f"$\\uparrow$ {over}", (-0.42, YCAP), fontsize=8, color="0.4",
                 ha="left", va="top")
    fig.suptitle("Filter consistency vs. process/measurement noise "
                 "(10 seeds, NEES over x, y, yaw)", fontsize=13,
                 fontweight="bold", y=1.02)
    fig.tight_layout()
    out = f"{IN}/noise_sensitivity.png"
    fig.savefig(out, dpi=200, bbox_inches="tight", facecolor="white")
    print("wrote", out)

if __name__ == "__main__":
    main()
