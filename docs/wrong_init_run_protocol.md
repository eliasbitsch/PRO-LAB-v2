# Wrong-Init Experiment - Run Protocol

## Run-Konfiguration

| Parameter | Wert |
|---|---|
| Datum | 2026-06-08 |
| Seeds pro Szenario | 1 (deterministisch) |
| Dauer pro Szenario | 60 s (50 s nutzbar, 10 s Boot-Overhead) |
| Szenarien | 7 |
| Modus | headless (`USE_RVIZ=false`) |
| Container | `prolab_jazzy` (ROS 2 Jazzy + Gazebo Harmonic) |
| Roboter | TurtleBot4 (Unicycle / Diff-Drive) |
| Welt | `nav2_minimal_tb4_sim/warehouse.sdf` + 8 I-Beam-Säulen 2×4-Grid |
| Bewegungs-Skript | 3-Punkt-Wende (S-Kurve → Nord → Reverse → Nord-Ost), 32 s reine Bewegung |

## Szenarien

| # | Szenario | Filter-Init-Pose | Filter-Init-σ | Truth-Spawn |
|---|---|---|---|---|
| 1 | `correct_init` | (0, 0, 0) | xy=0.10, yaw=0.05 | (0, 0, 0) |
| 2 | `offset_1m` | (1, 0, 0) | xy=0.50, yaw=0.10 | (0, 0, 0) |
| 3 | `offset_5m` | (5, 0, 0) | xy=1.00, yaw=0.20 | (0, 0, 0) |
| 4 | `wrong_yaw_pi2` | (0, 0, π/2) | xy=0.20, yaw=0.30 | (0, 0, 0) |
| 5 | `overconfident_wrong` | (3, 0, 0) | xy=0.05, yaw=0.02 (zu eng!) | (0, 0, 0) |
| 6 | `underconfident` | (0, 0, 0) | xy=2.00, yaw=0.50 (zu weit) | (0, 0, 0) |
| 7 | `kidnapped` | uniform 30×30 m, 3000 Partikel | - | (0, 0, 0), +2 Teleports @ t=18s,28s |

## Filter-Übersicht

| Filter | State | Predict-Input | Update-Inputs | Quelle |
|---|---|---|---|---|
| **KF** | `[x, y, yaw, vx, vy, ω]` (6D Constant Velocity) | autonom (CV) | `/odom` (velocity anchor), `/imu` (yaw + ω), Landmarks (triangulation → pose) | Linear KF Thrun §3.2 |
| **EKF** | `[x, y, yaw]` (3D Unicycle) | `/odom` velocity (Thrun §5.4 odometry control) | `/imu` (yaw), Landmarks (range/bearing) | EKF Thrun §3.3 |
| **EKF-LF** | `[x, y, yaw]` | `/odom` velocity | `/scan` direct via Likelihood-Field | EKF + Thrun §6.4 |
| **PF** | 1500 Partikel | `/odom` velocity + motion noise | `/scan` Likelihood-Field, low-variance resample | Monte Carlo Localization Thrun §8.3 |
| **AMCL** | adaptive Partikel | `/odom` motion model | `/scan` beam model | nav2_amcl (Baseline) |

## Sanity-Check der CSVs

| Szenario | Zeilen | Dauer | Truth-Sprünge | Landmark-Ticks |
|---|---:|---:|---:|---:|
| correct_init | 996 | 55.1 s | 0 ✓ | 67% |
| offset_1m | 1011 | 49.7 s | 0 ✓ | 70% |
| offset_5m | 982 | 50.1 s | 0 ✓ | 90% |
| wrong_yaw_pi2 | 975 | 49.6 s | 0 ✓ | 88% |
| overconfident_wrong | 981 | 50.0 s | 0 ✓ | 23%* |
| underconfident | 992 | 49.5 s | 0 ✓ | 89% |
| kidnapped | 980 | 54.9 s | **2 ✓** (geplant) | 90% |

*`overconfident_wrong` hat geringere Detection-Rate weil die Filter mit zu engem σ falsche Landmark-Assoziationen verwerfen.

## Hauptergebnisse - Final RMSE_xy [m]

> ⚠️ **Wichtig:** Die Werte sind aus den per-Tick `<filter>_err_xy`-Spalten der Timeseries-CSVs **neu berechnet** als `√(Σ err²/n)`. Die `*_final_rmse_xy`-Spalten im Summary-CSV waren wegen eines metrics_node → csv_logger Sample-Drop unzuverlässig. `analyze_results.py` überschreibt sie jetzt automatisch.

| Szenario | KF | EKF | EKF-LF | PF | **AMCL** |
|---|---:|---:|---:|---:|---:|
| correct_init | 0.28 | 0.29 | **0.14** | 1.00 | **0.07** |
| offset_1m | 0.78 | 0.70 | 0.69 | **0.27** | 0.49 |
| offset_5m | 4.69 | 4.73 | 5.72 | **3.50** | 4.09 |
| wrong_yaw_pi2 | 3.69 | 1.69 | **0.14** | 0.34 | 0.39 |
| overconfident_wrong | 3.39 | 3.39 | 3.85 | 3.49 | **2.83** |
| underconfident | 0.80 | 0.70 | **0.23** | 0.40 | 0.55 |
| kidnapped | 7.42 | 7.19 | **4.23** | 13.11 | 14.68 |

> **Update 2026-06-09:** Die obigen Zahlen sind aus dem **dritten Lauf** auf NVIDIA RTX 3080 Ti mit korrigierten Szenario-YAMLs. Davor zwei Datensätze verworfen wegen (a) iGPU-Lidar-Noise und (b) Szenario-Init-Koordinaten relativ zur alten Spawn-Pose (-8, -0.5) statt neuer (0, 0).

## Beobachtungen

**1. AMCL gewinnt baseline (correct_init 0.07m)** - bestätigt Goldstandard-Charakter.

**2. PF gewinnt offset_1m / offset_5m** (0.28m, 3.57m) - bei reinem xy-Offset deckt die Partikelwolke die Truth gut ab und konvergiert schnell.

**3. EKF-LF dominiert in 3 Hard-Fällen:**
- `wrong_yaw_pi2`: **0.14m** (KF/EKF mit Landmark-Triangulation hängen bei 3.6/1.7m, weil falscher yaw die Triangulation verzerrt)
- `underconfident`: **0.23m** (deterministisches Scan-Matching ist immun gegen riesige Init-Covariance)
- `kidnapped`: **4.80m** (Scan-Matching findet die neue Pose direkt; AMCL/PF sind durch ihre alten Partikel-Wolken festgefahren)

**4. PF divergiert bei kidnapped (13.07m)** - klassisches Particle-Deprivation: nach Resampling sind alle Partikel nahe der "alten" Pose, kein einziger an der neuen kidnap-Pose. **Lehrbuch-Ergebnis.**

**5. AMCL versagt bei kidnapped (14.17m)** - globale Lokalisierung aus 30×30m Box + zwei aufeinanderfolgende Teleports überfordert sample-basierten Filter.

**6. KF und EKF sind nahezu identisch** in den meisten Szenarien (Input + Update-Path gleich); EKF gewinnt knapp bei `wrong_yaw_pi2` weil sein nichtlineares Modell den IMU-Yaw direkt nutzt. Hauptunterschied **bleibt State-Dimension (6D vs 3D) → Runtime-Plot.**

## Bugs während der Erstellung gefunden + behoben

### Bug 1 - EKF-Drift während Warmup (✓ behoben)

**Symptom:** EKF sprang in den ersten 0.17s einmalig um 1.26m nach (0.09, -1.26) und blieb dort für die 14s Warmup → biased running RMSE auf 5.40m statt 0.29m.

**Ursache:** EKF subscribt auf `/cmd_vel` als Predict-Input. nav2's `controller_server` publiziert während seiner Bringup-Phase kurz non-zero auf `/cmd_vel` (umgeht den cmd_vel_watchdog). EKF integriert das fälschlich.

**Fix:** EKF auf `/odom` umgestellt (Thrun §5.4 *Odometry Motion Model* statt §5.3 *Velocity Motion Model*). Gleicher Input wie KF - fair für den Vergleich. Während Warmup sind Wheel-Encoder = 0 → keine Phantom-Bewegung.

Geändert: [ekf_node.cpp:75-79](../ws/src/pro_lab_filters/src/ekf_node.cpp#L75-L79).

### Bug 2 - Summary-CSV-Werte unzuverlässig (✓ behoben 2026-06-09)

**Symptom:** `<filter>_final_rmse_xy` in den Summary-CSVs stimmte für viele Filter/Szenarien nicht mit der per-Tick `<filter>_err_xy`-Spalte überein (Faktoren bis 30× Diskrepanz).

**Root cause:** Zwei unabhängige Ursachen, die zusammen die Pipeline ruiniert haben:

1. **NaN-Kontamination durch AMCL** - Wenn AMCL nicht publizierte (Bug 4-Fall), schluckte `metrics_node`'s `sse_xy` einen NaN und propagierte ihn auf alle nachfolgenden Samples. Dadurch wurde `final_rmse_xy` für jeden Filter im Run irgendwann NaN-poisoned.
2. **Sample-Rate-Mismatch bei AMCL** - AMCL publiziert mit ~2 Hz, `csv_logger` snapshotet mit 20 Hz. Eine naive Recompute aus der Timeseries gewichtet jeden AMCL-Wert mit seiner Dwell-Time (bis zu 100×). Das ist eine andere RMSE-Definition als die "eine Stichprobe pro Messung", die `metrics_node` benutzt.

**Fix:**
- (1) durch Bug 4-Fix (AMCL publiziert jetzt überall) automatisch gelöst.
- (2) [analyze_results.py:537-562](../scripts/analyze_results.py#L537-L562) entfernt konsekutive Duplikate vor der RMSE-Berechnung. Stichproben-RMSE matched jetzt metrics_node's Ausgabe auf <1% genau für alle Filter.

### Bug 3 - Ground Truth via AMCL kontaminiert (✓ behoben)

**Symptom:** Im kidnapped-Szenario zeigte die Truth-CSV 32 wilde Sprünge bis (-15.8, 11.1) - außerhalb der Karte.

**Ursache:** `truth_relay_node` las `map → base_footprint` aus TF, was AMCL's `map → odom`-Schätzung enthält. Bei globaler Lokalisierung mit 3000 uniformen Partikeln flackert AMCL initial wild → Truth flackert mit.

**Fix:** Neuer [gz_truth_relay.cpp](../ws/src/pro_lab_filters/src/gz_truth_relay.cpp) subscribt direkt via gz transport auf `/world/warehouse/dynamic_pose/info`, filtert nach Entity-Name `turtlebot4`. Komplett unabhängig von AMCL.

### Bug 4 - Szenario-YAMLs referenzieren alte Spawn-Pose (✓ behoben 2026-06-09)

**Symptom:** AMCL publizierte für `wrong_yaw_pi2` keine Pose (`amcl_x` = NaN in der ganzen CSV). KF zeigte fluktuierende RMSE-Werte (10.3m im iGPU-Run vs 0.45m im NVIDIA-Run) ohne erklärbaren Grund.

**Ursache:** Die Szenario-YAMLs (`offset_1m`, `offset_5m`, `wrong_yaw_pi2`, `overconfident_wrong`, `underconfident`) hatten `init_x/init_y`-Werte relativ zur alten Spawn-Pose **(-8, -0.5)** - z.B. `init_x = -8.0` für "filter init am Truth". Der Spawn wurde aber auf **(0, 0)** umgestellt. Folge: alle Filter wurden **8m daneben** initialisiert.

AMCL bei wrong_yaw_pi2: init bei (-8, -0.5) mit σ=0.5m → Partikel-Cloud im falschen Map-Bereich → Scan matched nichts → keine Konvergenz.

**Fix:** Alle 5 YAMLs umgestellt auf (0, 0)-relative Werte. Konsistente Semantik:
- `offset_1m`: init = (+1, 0)
- `offset_5m`: init = (+5, 0)
- `wrong_yaw_pi2`: init = (0, 0, π/2) - **nur** yaw falsch
- `overconfident_wrong`: init = (+3, +2) mit σ=0.05
- `underconfident`: init = (0, 0) mit σ=2.0

### Bug 5 - `ros_gz_bridge` verwirft Entity-Namen (umgangen)

**Symptom:** Beim Versuch (Bug 3 zu fixen) über die ROS-Bridge: TFMessage-Konvertierung dropt das `name`-Feld aus Pose_V → kein Filtern möglich.

**Workaround:** Bug 3-Fix benutzt gz transport C++ direkt, nicht die Bridge.

## Reproducibility

```bash
# Container starten
docker start prolab_jazzy

# Vollen Batch laufen lassen (~7 min headless)
docker exec prolab_jazzy bash -c "
  cd /home/ros/ws && source install/setup.bash &&
  DURATION=60 USE_RVIZ=false \
    bash src/pro_lab_filters/scripts/run_wrong_init_batch.sh
"

# Plots generieren + auf 6 essentials kürzen
python3 scripts/analyze_results.py \
  --in results/wrong_init --out results/wrong_init/plots
```

## Nächste Schritte

- **Multi-Seed-Sweep (10 Seeds × 7 Szenarien, ~70 min)** für statistische Fehlerbalken - Vorgabe aus dem Task-Spec
- **Bug 2 root-cause fixen** in metrics_node oder csv_logger QoS (sauberer wäre besser als der Workaround)
- **Paper-Schreibarbeit** mit den 6 Plots + dieser Tabelle

## iGPU → NVIDIA RTX 3080 Ti Migration (2026-06-09)

| Aspekt | iGPU (alt) | NVIDIA RTX (neu) |
|---|---|---|
| `/scan` Lidar-Rendering | Mesa Software-Pfad | Hardware via EGL/CUDA |
| Beam-Genauigkeit | ±5-10 cm Jitter zwischen Frames | Sub-cm exakt |
| Landmark-Detektor | viele False-Positives | saubere Cluster |
| KF Triangulation | mit Noise → falsche Pose | mit präzisen Messungen → korrekte Pose |
| Effekt | Filter werden bestraft durch Sensorrauschen | Filter zeigen ihre echten Eigenschaften |

**Setup-Änderungen für NVIDIA-Pfad:**
- `docker-compose.yml`: `__EGL_VENDOR_LIBRARY_FILENAMES` auf NVIDIA gepinnt
- `wrong_init_experiment.launch.py` + `all_in_one.launch.py`: Server-Mode mit `--headless-rendering` (= EGL-Pfad statt GLX/iGPU)
- `run_wrong_init_batch.sh`: `gz_gui:=false` wenn `USE_RVIZ=false`

GUI bleibt absichtlich auf iGPU (NVIDIA-GLX crasht unter XWayland mit GLXBadFBConfig). Headless-Batch läuft auf der RTX.
