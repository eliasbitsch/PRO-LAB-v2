# PRO-LAB Presentation — Slide Outline & Speaker Notes

**Talk: 10 minutes · ~12 slides · ~50 s each.** Figures are in
`results/wrong_init/fav_plots/`. Drop them into your SharePoint deck slide by
slide; the *Notes* are what to say.

---

## 1 · Title  (20 s)
**Bad Beginnings — Why the Observation Model, Not the Filter, Decides
Localization Recovery under Wrong Initialization**
Elias Bitsch · Probabilistic Robotics Lab, FH Technikum Wien · Task 2510331021

> *Notes:* "My task was Wrong Initialization. I compared five localization
> filters when the robot starts with a *wrong* belief about where it is — and
> found something a bit surprising about what actually makes a filter recover."

---

## 2 · Motivation — the problem  (1 min)
- Localization = recursive Bayesian filtering (KF, EKF, particle filters).
- Textbook assumption: the initial belief already covers the true pose.
- Reality breaks it: powered up at an unknown aisle, operator clicks a wrong
  "2D pose estimate", a forklift shoves the robot → **kidnapped-robot problem**.
- Warehouses are **symmetric** → many places look identical to a range sensor
  (*perceptual aliasing*).

> *Notes:* "Filters are great once they're tracking — but the same mechanism
> that makes them efficient, collapsing onto one hypothesis, works against them
> when the start is wrong. And warehouses are deliberately regular, so a laser
> scan one bay over looks almost identical. That's the setting I stress-test."

---

## 3 · Research question & contribution  (45 s)
- Two answers to "where am I?": **match scans** (Nav2 AMCL) vs **read
  identifiable landmarks** (AprilTags).
- **Question:** when the start is wrong, does a fancier *filter* help — or does
  *what you measure* decide recovery?
- **Contribution:** controlled, 10-seed comparison of 5 filters × 7 adverse
  scenarios; 4 filters self-implemented in C++; honest camera-read tag identity.

> *Notes:* "The key question: is recovery about the estimator, or about the
> observation model? I isolate that by giving every filter the same motion and
> the same wrong start, and only changing what they measure."

---

## 4 · The five filters  (45 s)
| Filter | Sensing | Belief |
|---|---|---|
| KF | landmark (AprilTag) | Gaussian (linear) |
| EKF | landmark (AprilTag) | Gaussian (unicycle) |
| EKF-LF | lidar likelihood field | Gaussian |
| PF | lidar likelihood field | particles (+ augmented MCL) |
| AMCL | lidar (Nav2 baseline) | particles |

> *Notes:* "KF and EKF read the tags; EKF-LF, PF and AMCL match the laser scan.
> Gaussian filters are one hypothesis; particle filters are many. I implemented
> the first four from scratch; AMCL is the Nav2 baseline."

---

## 5 · Method — system & honest landmarks  (1 min)
- TurtleBot 4 · ROS 2 Jazzy · Gazebo Harmonic · symmetric 2×4 pillar grid (7.5 m).
- Each pillar wears a unique **AprilTag 36h11**; the OAK-D camera reads ID +
  range + bearing (AprilTag-3 + PnP), **<1 cm / <0.25°** vs ground truth.
- **Identity is *measured*, never an oracle.** Ground truth only scores error.
- *[figure: a Gazebo/RViz screenshot of the robot seeing a tag]*

> *Notes:* "Important for honesty: the robot really *reads* the tag identity
> with its camera — I don't hand it the answer. Ground truth is only used to
> grade the error afterwards, never fed into a filter."

---

## 6 · The localization principle  (45 s)
- *[figure: `correct_init_ekf_explainer.png`]*
- Dead reckoning drifts and its **uncertainty grows unbounded** — it doesn't
  know it's wrong. Landmarks collapse the covariance and re-anchor the belief.

> *Notes:* "This is what a landmark buys: not raw accuracy, but *bounded*
> uncertainty. Odometry alone has no idea it's drifting; a tag re-anchors it."

---

## 7 · Result — recovery from a wrong start  (1 min)
- *[figure: `rmse_comparison.png`]*
- Correct start → all 5 agree within 15 cm (sensing doesn't matter).
- 5 m offset / overconfident wrong → **landmark filters 0.3 m, scan filters
  3–5 m**. The picture *inverts* as the error grows.

> *Notes:* "When the start is correct, everybody's fine — so the modality is
> irrelevant. But push the initialization wrong and the landmark filters stay
> at 30 cm while the scan filters blow up to several metres."

---

## 8 · Result — the kidnapped robot  (1 min)
- *[figures: `kidnapped_ekf_localization.png` + `kidnapped_amcl_localization.png`]*
- Two teleports. EKF snaps onto the truth once it reads a tag at the new
  location; AMCL/PF stay lost (up to **17 m**) — trapped by symmetry.

> *Notes:* "Here the robot is teleported twice. The EKF re-localizes the moment
> the camera sees a tag — left plot, estimate lands on the true pose. AMCL on
> the right is hopelessly lost: every aisle looks the same."

---

## 9 · Why scan filters fail + landmarks are king  (1 min)
- *[figure: `convergence_rate.png`]*
- Perceptual aliasing: a one-bay shift ≈ identical scan → scan filters converge
  to a **wrong-but-symmetric** hypothesis.
- **Landmarks are king:** KF/EKF recover *autonomously* (tag identity names the
  pillar). PF/AMCL need a **human "2D pose estimate"** to escape — the textbook
  "particle filters need a good initial guess".

> *Notes:* "Convergence rate makes it crisp: under kidnapping KF/EKF succeed in
> 40–60 % of seeds, scan filters in zero. And the scan filters only recover if
> *I* hand them a pose estimate — the landmark filters never need that help."

---

## 10 · Live demo  (1 min)
- Drive (teleop) → **kidnap** with the RViz arrow → KF/EKF snap back on their
  own → rescue PF/AMCL with **2D Pose Estimate**.
- Launcher: `bash scripts/prolab.sh` → Live demo.

> *Notes:* (Show it live if possible.) "Watch: I kidnap the robot. The landmark
> estimates jump right back onto it. The particle filters stay scattered until
> I give them a pose estimate — then they snap back too."

---

## 11 · Conclusion & outlook  (45 s)
- **The observation model, not the filter, decides recovery** from a wrong start.
- Caveat (honest): under nominal conditions landmarks add noise, not accuracy —
  the benefit is robustness + bounded uncertainty.
- Future: we kept the modalities *separate by design* to isolate the effect;
  the natural next step is to **fuse** them (scan for accuracy, tags for
  recovery), plus hardware tests and active perception (turn the camera toward
  an expected tag after a likelihood collapse).

> *Notes:* "Takeaway: don't reach for a fancier filter to survive a bad start —
> reach for an identifiable measurement. Tags cost you environment
> instrumentation, but they turn an intractable recovery into an easy one."

---

### Backup slides (if asked)
- Runtime / Big-O: `runtime_comparison.png` — Gaussian landmark filters are
  cheapest (O(n³)) vs O(M·R) for the samplers.
- Error-over-time band: `wrong_yaw_pi2_error_xy.png`.
- Full RMSE table (7 scenarios × 5 filters, mean ± 1σ) — see the paper.
