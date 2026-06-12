# Configuration presets — research basis

The live viewer (`test.py`) ships two presets, selectable from the **Config
preset** dropdown. Selecting one sets every slider (detection + tracking +
control); the UI keeps overriding live afterwards.

## How the values were derived

Two inputs: (1) **on-robot measurement** of this exact RPLidar A1, and
(2) **published parameters** from production 2D-LiDAR people trackers.

### Measured on our robot (40 static scans)
| Quantity | Measured |
|----------|----------|
| Angular resolution Δθ | **0.499°** (not 1°) → ~8000 samples/scan |
| Points on a 0.12 m leg | 13 @1 m · 6 @2 m · 4 @3 m · 3 @4 m |
| Range noise σ_r | 1 mm (<1 m) · 3.3 mm (1–2 m) · 18 mm (2–3 m) · 40 mm (3–5 m) |

Consequence: `min_points` caps detection range. At 0.5°, `min_points=12`
only detects to ~1 m; `min_points=4` reaches ~3 m. Noise ≈ 1 % of range,
matching the A1 datasheet (±1 % ≤3 m).

### Literature (detector)
| Source | jump dist | min pts | leg-pair max | leg width |
|--------|-----------|---------|--------------|-----------|
| Arras/Mozos 2007–10 (Freiburg) | **0.15 m** | ≥3 (≥4 heuristic) | none (temporal) | 0.05–0.50 m blob |
| ROS `leg_detector` (wg-perception) | 0.06 m | 5 | 1.0 m | learned |
| Leigh `leg_tracker` ICRA 2015 | **0.13 m** | 3 (5 noisy) | 0.8 m | learned |
| SPENCER blob detector | 0.10 m | 3 | — | 0.05–0.50 m |
| Garcia Pereira 2010 (FSM legs) | — | — | 0.15–0.45 m | single 0.05–0.15 m, pair 0.15–0.32 m |
| Anthropometry @18 cm height | — | — | stance 0.17 m, stride ≤0.7 m | calf Ø ~0.11 m, ankle ~0.07 m |

### Literature (Kalman tracker)
| Source | process noise q | gate | assoc | confirm/delete | meas σ | moving cutoff |
|--------|-----------------|------|-------|----------------|--------|---------------|
| SPENCER `srl_nn_tracker` | **0.01** default, 0.1 paper, ≤1.5 tuned | **χ²=9.21 (99 %)** | greedy NN (GNN opt.) | 6 hits / 10–50 occl. | 0.1–0.32 m | **0.2 m/s** |
| Leigh `leg_tracker` | diag, σ=0.5·dt | z=1.645 (95 % 1-D) | GNN (Hungarian) | 0.5 m travelled / cov>0.81 | 0.1 m (0.5 assoc.) | none |
| wg `leg_detector` | pos 0.05, vel 1.0 | — | NN | 0.5–2.0 s timeout | 0.05 m | — |
| Human walking | accel 0.68 (max 1.44) m/s² | — | — | — | — | 1.2–1.5 m/s (≈0.8 indoor) |

Sources (read directly): Arras et al. ICRA 2007; Mozos et al. IJSR 2010;
github.com/angusleigh/leg_tracker; github.com/spencer-project/spencer_people_tracking;
github.com/wg-perception/people; Garcia Pereira ICINCO 2010; SLAMTEC A1
datasheet; Bohannon 1997 (gait speed); NHANES (calf/ankle anthropometry).

---

## Preset values

| Slider | `default` (lab-tuned) | `test-1-research` | Why research value |
|--------|----------------------|-------------------|--------------------|
| distance_threshold | 0.30 | **0.13** | Leigh 0.13 / Mozos 0.15 — right scale for 0.5°+noise; 0.30 over-merges |
| leg_radius | 0.20 | **0.06** | calf radius ~0.056 m → single-leg width gate ≈0.12 m (real) |
| max_leg_width | 0.60 | **0.45** | Garcia stance-apart max 0.45 m; covers standing + moderate stride |
| circularity_min | 0.80 | **0.20** | wall ≲0.1, leg ≳0.6; 0.20 rejects walls without dropping noisy legs |
| min_points | 12 | **4** | Arras min-4; reaches ~3 m on our 0.5° sensor (12 → only ~1 m) |
| max_range (view) | 2.0 | **4.0** | see people approaching from farther |
| gate χ² (T1) | 5.40 | **9.21** | SPENCER 99 % gate; 95 % churns IDs at our cm-level R |
| process noise q | 1.0 | **0.5** | between SPENCER paper 0.1 and responsive 1.0 (accel ~0.7 m/s²) |
| prediction horizon | 3.0 | **2.0** | people predictable ~1–2 s ahead |
| moving cutoff | 0.30 | **0.30** | SPENCER 0.2 / crowd-flow 0.4 → 0.3 |

Control (Stage 3 CBF-QP) values are shared by both presets (the research was
detector/tracker-focused); they follow the `exp_cbf_public.ipynb` lab:
lookahead L=0.30 m, γ=2.0, ω-weight 0.10 ("steer before brake"). person 0.30 m
+ robot 0.18 m base safety radius, inflated by 2σ of track uncertainty.

> **Trade-off to call out in the demo:** the `default` preset (min_points 12,
> max_leg_width 0.60) was hand-tuned to suppress clutter in one specific room
> and only sees people to ~1 m. `test-1-research` detects to ~3 m and is the
> better general choice; raise `min_points` if a noisy room produces clutter.

---

## Recorded-session tuning campaign (June 2026)

Seven recorded lidar sessions (`/tmp/t*_*.npz`, 60–90 s each), each analyzed
offline and A/B-tuned on the SAME data before changing any default:

| # | Scenario | Key finding → default baked |
|---|----------|------------------------------|
| T1 | empty-ish room baseline | far clutter churn → `DETECT_MAX_RANGE 3.5 m` |
| T2 | person at 1/2/3 m + transit | leg-split duplicate takeover → `max_leg_width 0.65`, moving-track spawn guard |
| T3 | continuous walking, reversals | coasting-away tracks → coast damping 0.6/cycle, `q=1.0` validated (pred 8/19/50 cm @0.3/0.6/1.0 s) |
| T4 | two people stand/cross/side-by-side | occlusion-shadow ghosts → `occluded` flag: shadow-cut clusters update tracks but never spawn them |
| T5 | walk behind furniture, brush a bag | lidar sees under tables — full occlusion never happened; 0.4 s flicker tracks → `confirm_hits 4` |
| T6 | robot moves, people stand | pairing alternation = ±15 cm centroid hop → phantom ~1 m/s on standing person; net-displacement speed clamp (80→39 cm/s, walkers untouched) |
| T7 | robot moves + person walks | 99–100 % walker coverage in all robot modes, 3/5 cm prediction — defaults validated, no change |

**Recorder lesson:** Create3 "odometry teleports" (3.82 m!) in recordings were
the recorder's own /odom DDS discovery delay (pose placeholder [0,0,0] until
first message). Recorders must gate scan capture on the first odom message.
The dashboard keeps a 0.35 m/0.5 rad odom-jump guard as cheap insurance —
a real frame jump corrupts every odom-frame track, so reset beats trusting it.
