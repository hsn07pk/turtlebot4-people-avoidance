# 2D Multi-Target Tracking with Nearest Neighbor Kalman Filter — Pipeline

## Overview

This notebook extends a single-target 2D Kalman filter demo to a **multi-target tracking** scenario using the **Nearest Neighbor (NN)** data association algorithm. Multiple targets are simulated with the same motion model, their measurements are combined and shuffled (unknown data association), and independent Kalman filters track each target with NN measurement-to-track assignment.

---

## Phase 0 — Setup (original cells, unchanged)

### Motion model
Each target follows a **constant velocity + process noise** model (continuous white noise acceleration):

```
A = [[1, 0, dt, 0],       x(t+dt) = x(t) + vx(t)·dt
     [0, 1, 0, dt],       y(t+dt) = y(t) + vy(t)·dt
     [0, 0, 1, 0],        vx(t+dt) = vx(t)
     [0, 0, 0, 1]]        vy(t+dt) = vy(t)
```

Process noise `Q` models random accelerations (`q = 1` controls intensity). The sensor measures only position: `H = [[1,0,0,0],[0,1,0,0]]` with Gaussian noise `R = s²·I` (`s = 0.5`).

### Reused existing functions
- `generate_ssm(m₀, A, Q, H, R, steps)` — simulates true states and noisy measurements for one target
- `kalman_predict(m, P, A, Q)` — Kalman filter prediction step
- `kalman_update(m, P, H, R, y)` — Kalman filter update step
- `kalman_distance(m, P, H, R, y)` — squared Mahalanobis distance between prediction and measurement
- `rmse(x, y)` — root mean squared error

---

## Phase 1 — Multi-target simulation

### Parameter
`n_targets = 4` (dynamic — change this value to scale the simulation).

### Target generation
For each target `i` (0..n_targets-1):
```
angle = 2π·i / n_targets
initial position:  (4·cos(angle), 4·sin(angle))
initial velocity:   tangential to the circle (magnitude ≈ 1 unit/step)
```

All targets start on a circle of radius 4, each moving tangentially. `generate_ssm` produces `steps = 100` time steps per target.

### Measurement combination
At each time step, all `n_targets` measurements (one per target) are **stacked and shuffled** (`np.random.shuffle`). The result `combined_observations[t]` is an `(n_targets, 2)` array where the row ordering is random — the tracker does **not** know which measurement belongs to which target.

### Ground truth visualization
`plot_multitarget` shows the true trajectories (one colour per target) and their measurements. This is the only plot that has access to ground truth — the tracker does not use it.

---

## Phase 2 — Nearest Neighbor Multi-Target Tracker

### Chi-squared gating
The squared Mahalanobis distance between a track prediction and a measurement follows a χ² distribution with `df = N = 2`. A 95% confidence threshold (**≈ 5.991**) defines the validation gate: measurements with distance exceeding this threshold are considered too unlikely and are ignored for that track.

### NN measurement-to-track assignment — `nearest_neighbor_assignment`

```
Input:
  ms[t]    — predicted state mean for track t (position + velocity)
  Ps[t]    — predicted state covariance for track t
  measurements — (K, 2) array of anonymous measurements at current time step
  gate_threshold — χ² threshold (5.99)

Algorithm:
  1. Build cost matrix C[t, m]:
     For each (track t, measurement m) pair:
       C[t, m] = kalman_distance(ms[t], Ps[t], H, R, measurements[m])
       if C[t, m] > gate_threshold → C[t, m] = ∞ (gate reject)

  2. Collect all valid pairs (C[t, m] < ∞), sort by ascending cost.

  3. Greedy assignment:
     Iterate sorted pairs. Assign (t, m) only if:
       - track t is still unassigned, AND
       - measurement m is still unassigned.
     Each measurement is assigned to at most one track,
     each track receives at most one measurement.

Output:
  assignments[t] = m (measurement index assigned to track t)
                or -1 (no measurement in gate → coasting)
```

### NN Multi-Target Kalman Filter — `nn_multitarget_filter`

Main loop, executed for each time step:

```
For each time step i:

  1. PREDICT — for each track t:
     ms[t], Ps[t] = kalman_predict(ms[t], Ps[t], A, Q)
     → predicted position = prior position + velocity·dt (plus added uncertainty from Q)

  2. NN ASSOCIATION:
     assignments = nearest_neighbor_assignment(ms, Ps, H, R, combined_obs[i], gate_threshold)
     → each track gets the closest measurement within its gate

  3. UPDATE or COAST — for each track t:
     if assignments[t] >= 0:
       ms[t], Ps[t] = kalman_update(ms[t], Ps[t], H, R, measurement)
       → correct position and velocity estimates with the assigned measurement
     else:
       → COASTING: keep the prediction as the current estimate
         (without a measurement, uncertainty grows but the state estimate stays the prediction)
```

**Key point**: coasting occurs when no measurement falls within a track's gate. With threshold 5.99 and large P₀ in our simulation (no clutter), this is unlikely. In real scenarios with false alarms it is common.

### Measurement-driven track initialization

**Before** running the filter, tracks are spawned from the **first frame of measurements**:

```
first_frame = combined_observations[0]   # n_targets anonymous measurements

For each measurement i (0..n_targets-1):
  m₀[i] = [measurement.x, measurement.y, 0, 0]   ← position from measurement, velocity unknown = 0
  P₀[i] = 10·I                                     ← high initial uncertainty
```

The filter is then called on frames **1..steps-1** (frame 0 has already been consumed for initialization). The step-0 initial states are prepended as the step-0 estimates.

This is realistic: in production the tracker has no prior knowledge of target positions — it discovers them from the first sensor returns.

---

## Phase 3 — Output and evaluation

### Tracker output plot — `plot_tracker_output`
Shows the **raw tracker output** — one subplot per track, with:
- Track estimate (dashed orange line)
- All measurements as light grey background scatter
- No track-to-target matching, no ground truth
- Track IDs are arbitrary labels

This is exactly what a real system would produce. Track 4 may happen to follow physical object A, Track 1 may follow object B — the tracker does not know or care. What matters is that every measurement is consistently associated to some track and that each track follows a coherent trajectory over time.

### RMSE evaluation — for benchmarking only
Only computed for **evaluation** (requires ground truth, not available in production):

1. **Track-to-target matching**: for each true target, find the track whose final position is closest (greedy assignment).
2. Compute `rmse(true_position, estimate_position)` for each matched pair.
3. Print which track was matched to which target.

The matching is purely for performance benchmarking — a real tracking system has no ground truth to match against.

---

## Why it works

| Step | What happens |
|------|-------------|
| Frame 0 | 4 anonymous measurements → 4 tracks are born, each at a different position |
| Frame 1 | Predict (same position, v=0) → each track is closest to ONE target → NN assigns correctly |
| Frame 2+ | Update corrects velocity, prediction now anticipates motion → association becomes even more robust |
| Always | If two targets were to cross, the gate would prevent accidental swaps (unless measurements are truly ambiguous) |

---

## Files

- `Copy_of_2d_kalman_demo.ipynb` — the complete notebook
- Original cells (0–21) are unchanged
- New cells (22–39) implement multi-target simulation, NN tracker, and evaluation
