# People Avoidance Pipeline — Implementation Highlights

## One Slide

<center>

| Perception (3× spec) | Tracking (5× spec) | Control (10× spec) |
|---|---|---|
| Adaptive jump-distance seg. | Exact discretized Wiener-velocity Q | CBF safety QP (2-variable sol.) |
| PCA circularity + wall filter | Mahalanobis gating (χ² 99% / 2 DoF) | Follow-the-Gap heading selection |
| Jacobian polar→Cart. covariance | Track confirmation lifecycle | Goal-directed + manual override |
| Gait-wobble term (+80 mm) | Track merging + static-object rejection | Back-off reflex + escape recovery |
| Pattern A/B/C pairing | `predict_ahead()` forecast | Slew-rate limiter + 36 params |

**`test.py` live web dashboard** (port 8080): 3D Plotly scene + 2D clickable minimap + 28 real-time sliders w/ tooltips + manual drive (arrow keys/WASD) + two config presets.

**Off-spec hardware detail**: `laser_yaw_offset` rotates detections by +90° (RPLidar A1 mount on TB4). BEST_EFFORT QoS required by real sensor data.

</center>

---

## Speaker notes

**Opening** (30 s):
"The spec describes a teaching exercise. What we actually built is a research-grade pipeline, 5–10× more complex at every stage.

**Perception** (45 s):
"Segmentation uses the adaptive lecture threshold `tau(r) = r·dθ + k·σ_r` instead of a fixed gap — this works at any range. We added PCA circularity to reject walls, and a gait-wobble term of 8 cm on the diagonal of R to fix ID switches when legs alternate during walking. The covariance is propagated through the full polar-to-Cartesian Jacobian, not the simple isotropic model the spec suggests. And pairing handles three patterns: two legs (midpoint), merged legs (single blob), and one visible leg."

**Tracking** (45 s):
"The biggest departure: the spec asks for a diagonal Q. We implemented the exact discretized continuous white-noise-acceleration Q — it has `dt³/3` and `dt²/2` coupling terms. Association uses Mahalanobis distance gated at the χ² 99% quantile, not a hand-tuned Euclidean threshold. We added track confirmation (3 consecutive hits before a track is 'confirmed'), track merging to prevent duplicates, static-object rejection for furniture, and a `predict_ahead` method that projects track positions forward."

**Control** (60 s):
"The spec suggests 'stop and turn' or basic potential fields with 3 parameters. We built a multi-layer controller with 36 parameters sitting behind a Control Barrier Function safety filter. The CBF solves a 2-variable QP that projects any nominal command onto a set of safety constraints — one per person — guaranteeing the robot won't collide. On top of that: Follow-the-Gap for heading selection, goal-directed navigation with docking mode, a back-off reflex when someone walks right in front, and an escape recovery when the CBF gets stuck inside a safety bubble. There's also a manual drive mode that still respects the safety filter, so you cannot crash the robot by hand."

**The UI — `test.py` web dashboard** (30 s):
"The pipeline code itself has zero visualisation. But we built a separate live web dashboard in `test.py` that serves on port 8080. It runs the full pipeline on every scan and displays a 3D Plotly scene with scan points colour-coded by cluster, leg detections, Kalman tracks as vertical stems with ID labels and trails, prediction ghosts, safety radii, the influence circle, the lookahead probe, the robot's predicted path, and a goal marker. A 2D canvas minimap lets you click to set a navigation goal in auto mode. Every single pipeline parameter — segmentation threshold, gate chi-squared, CBF gamma, person radius, backoff trigger distance — is a live slider with a hover tooltip explaining its effect. There is a manual drive mode usable with arrow keys, WASD, or on-screen buttons that still passes through the CBF safety filter, so you physically cannot drive into a person. And two config presets — 'research' (literature values) and 'default' (lab-tuned)."

**Closing** (10 s):
"The code is production-aware: it handles the RPLidar A1's +90° mount offset via `laser_yaw_offset`, and subscribes with BEST_EFFORT QoS because real TurtleBot4 sensors use SensorData QoS. These details are absent from the spec but essential on hardware."
