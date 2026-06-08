# Development Guide — People Avoidance Pipeline

This document describes the pipeline you will implement, the exact data contract
between stages, and how to test your work incrementally.

For environment setup see `SIMULATION_SETUP.md`.

---

## Pipeline

```
/scan  (sensor_msgs/LaserScan  ·  ~5 Hz  ·  RPLidar A1  ·  laser frame)
    │
    ▼─────────────────────────────────────────────── Stage 1  leg_detection.py
    │  segment_scan()    split scan into contiguous point clusters
    │  detect_legs()     pair clusters → person positions + covariance
    │
    │  Output per scan:  List[LegMeasurement]
    │    x, y           position in laser frame (metres)
    │    Rxx, Rxy, Ryy  2×2 observation covariance R  (m²)
    │
    ▼─────────────────────────────────────────────── Stage 2  tracking.py
    │  KalmanTracker.update(measurements)
    │    predict()       constant-velocity propagation
    │    associate()     Hungarian-algorithm data association
    │    update step     KF correction equations
    │    lifecycle       spawn new tracks / prune lost tracks
    │
    │  Output:  List[Track]
    │    m  [x, y, vx, vy]   state mean   shape (4,)
    │    P                   state cov    shape (4,4)
    │
    ▼─────────────────────────────────────────────── Stage 3  controller.py
    │  obstacle_radius(track)                uncertainty → safety radius
    │  compute_velocity(tracks, x, y, θ)     avoidance policy → Twist
    │
    │  Output:  geometry_msgs/Twist
    │    linear.x   forward speed  (m/s)
    │    angular.z  rotation rate  (rad/s)
    │
    ▼
/cmd_vel  (geometry_msgs/Twist)
```

`people_avoidance_node.py` wires the stages together and reads the robot pose
from `/odom`.  **You do not edit the node** — only the three stage files.

---

## Data contract

### `LegMeasurement`  (leg_detection.py)

| Field | Type | Meaning |
|-------|------|---------|
| `x` | float | Person x in the **laser frame** (m) |
| `y` | float | Person y in the **laser frame** (m) |
| `Rxx` | float | R[0,0] — x variance (m²) |
| `Rxy` | float | R[0,1] = R[1,0] — cross-term (m²) |
| `Ryy` | float | R[1,1] — y variance (m²) |

### `Track`  (tracking.py)

| Field | Type | Meaning |
|-------|------|---------|
| `m` | ndarray (4,) | Mean **[x, y, vx, vy]** in the odom frame |
| `P` | ndarray (4,4) | State covariance |
| `track_id` | int | Unique identifier |
| `misses` | int | Consecutive unmatched frames |

Observation model: `H = [[1,0,0,0],[0,1,0,0]]` — position only.

### `Twist` output

| Field | Meaning |
|-------|---------|
| `linear.x` | Forward speed (m/s); positive = forward |
| `angular.z` | Rotation rate (rad/s); positive = left |

Safe default (stub): both zero → robot stops.

---

## What to implement

### Stage 1a — `segment_scan()`

Split a `(N,2)` point array into contiguous clusters.

```
current = [points[0]]
for i in 1 … N-1:
    if ‖points[i] − points[i-1]‖ > distance_threshold:
        emit current;  current = []
    current.append(points[i])
emit current
```

### Stage 1b — `detect_legs()`

FH-style leg-pair detection:

1. Call `scan_to_cartesian(scan)` (provided).
2. Call `segment_scan(points)`.
3. Mark segments whose width ≈ `2 × leg_radius` as leg candidates.
4. Pair candidates whose centre-to-centre distance ≤ `max_leg_width`; emit
   one `LegMeasurement` at the pair midpoint.
5. Assign `Rxx = Ryy = (σ × range)²`, `Rxy = 0` (isotropic range-noise model).

---

### Stage 2a — `KalmanTracker.__init__()`

Define constant-velocity matrices and store as `self.F`, `self.Q`:

```
F = [[1, 0, dt, 0 ],      Q = diag([σ_p², σ_p², σ_v², σ_v²])
     [0, 1,  0, dt],
     [0, 0,  1,  0],      σ_p ≈ 0.05 m    σ_v ≈ 0.10 m/s
     [0, 0,  0,  1]]
```

### Stage 2b — `predict()`

```
for each track:
    m = F @ m
    P = F @ P @ F.T + Q
```

### Stage 2c — `associate()`

1. Build cost matrix `C[i,j]` = Euclidean distance between track `i`'s
   predicted position and measurement `j`'s position (Mahalanobis is better).
2. Solve: `from scipy.optimize import linear_sum_assignment; r,c = linear_sum_assignment(C)`
3. Gate: reject pair `(i,j)` if `C[i,j] > threshold` (e.g. 1.5 m).

### Stage 2d — `update()`

For each matched `(track_i, meas_j)`:
```
z = [meas.x, meas.y]
R = [[meas.Rxx, meas.Rxy], [meas.Rxy, meas.Ryy]]
y = z − H @ m
S = H @ P @ H.T + R
K = P @ H.T @ inv(S)
m = m + K @ y
P = (I − K @ H) @ P          # Joseph form is more numerically stable
```
Reset `misses=0` for matched tracks, increment for unmatched.  
Spawn `_spawn_track(meas)` for unmatched measurements.  
Delete tracks where `misses > max_misses`.

---

### Stage 3a — `obstacle_radius()`

```python
pos_cov = track.P[:2, :2]
λ_max   = np.linalg.eigvalsh(pos_cov)[-1]
return sigma_scale * math.sqrt(max(λ_max, 0.0))
```

The radius grows while the filter is uncertain, shrinks as it converges.

### Stage 3b — `compute_velocity()`

Suggested approaches (choose one):

**Simple reactive:**  if any `dist(robot, person) < r_i`, stop and turn away.

**Potential fields:**
```
F_rep = Σ  k / dist² × (robot_pos − person_pos) / dist
           (active only while dist < influence_radius)
v, ω  = convert F_rep to differential-drive velocities
```

Clip final commands:
```python
cmd.linear.x  = float(np.clip(v,  0.0,  max_linear_speed))
cmd.angular.z = float(np.clip(ω, -max_angular_speed, max_angular_speed))
```

---

## Incremental testing

You don't need all stages to test partial work:

| Stage done | Observable result |
|------------|-------------------|
| None | Node starts; `/cmd_vel` publishes zeros |
| Stage 1 | Add a `self.get_logger().info(str(measurements))` in the node — you'll see detections when a cylinder is within range |
| Stage 2 | Tracks appear; add debug prints for `len(tracks)` |
| Stage 3 | Robot moves/stops/turns when pedestrians approach |

---

## Commands

```bash
# Rebuild after editing (fast — only changed files)
cd ~/ros2_ws
colcon build --packages-select people_avoidance --symlink-install

# Run with debug logging
ros2 launch people_avoidance people_avoidance.launch.py \
    --ros-args --log-level people_avoidance_node:=debug

# Override parameters without editing code
ros2 launch people_avoidance people_avoidance.launch.py \
    distance_threshold:=0.08 max_linear_speed:=0.3 obstacle_radius_scale:=3.0

# Send a test velocity to confirm the robot moves
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Visualise the scan
ros2 topic echo /scan | grep -E "^ranges" | head -3
```

---

## Parameters

All declared with defaults in `people_avoidance_node.py`.  Override at launch.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `scan_topic` | `/scan` | LiDAR input |
| `cmd_vel_topic` | `/cmd_vel` | Velocity output |
| `odom_topic` | `/odom` | Robot pose |
| `laser_frame` | `rplidar_link` | Laser frame id (TB4) |
| `dt` | `0.1` s | KF timestep |
| `max_misses` | `5` | Track deletion threshold |
| `distance_threshold` | `0.1` m | Segmentation gap |
| `leg_radius` | `0.10` m | Single-leg expected radius |
| `max_leg_width` | `0.25` m | Max leg-pair separation |
| `max_linear_speed` | `0.2` m/s | Forward speed cap |
| `max_angular_speed` | `1.0` rad/s | Rotation rate cap |
| `obstacle_radius_scale` | `2.0` | Uncertainty inflation k |

---

## What the simulated people look like in the LiDAR

Each pedestrian model has **two cylinders** (two legs, r = 0.10 m, 0.25 m
apart).  At 1 m range the RPLidar A1 (1° resolution) produces ~6 points per
leg, separated by 1–2 empty beams — exactly the pattern `segment_scan` +
`detect_legs` must find.

```
Top-view LiDAR cross-section

         ●●     gap     ●●
        ●  ●           ●  ●        ← left_leg        right_leg
         ●●             ●●

         ←——— 0.25 m ———→
```

---

## File map

```
src/people_avoidance/people_avoidance/
├── leg_detection.py          Stage 1  ← implement here
├── tracking.py               Stage 2  ← implement here
├── controller.py             Stage 3  ← implement here
└── people_avoidance_node.py  wiring   (do not edit)

src/pedestrian_sim/
├── pedestrian_sim/pedestrian_sim_node.py   walking people simulator
├── launch/simulation.launch.py             full environment launch
└── worlds/simple_room.sdf                  10×10 m dependency-free room

docker/
├── Dockerfile                builds the complete environment
├── docker-compose.yml        sim / avoidance / all profiles
└── entrypoint.sh             sources ROS + workspace on container start
```
