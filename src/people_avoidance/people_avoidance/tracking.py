"""
tracking.py — Stage 2 of the people-avoidance pipeline.

Input : List[LegMeasurement]  (one per scan, from leg_detection.py)
Output: List[Track]           (maintained across scans)

Each track i models one person as a Gaussian:
    X^i_t ~ N(m^i_t, P^i_t)

State vector  m = [x, y, vx, vy]   (position + velocity in the **laser/robot
frame** — measurements come from leg_detection in the laser frame and are not
transformed, so the controller treats the robot as the origin).
Covariance    P is 4 × 4.

Students implement:
  - KalmanTracker.__init__()   : define F and Q
  - KalmanTracker.predict()    : constant-velocity propagation
  - KalmanTracker.associate()  : measurement-to-track matching
  - KalmanTracker.update()     : KF update + track lifecycle management

Implementation follows the UBISS 2026 course material:
  - Särkkä, "Bayesian and Kalman filtering" + "Dynamics and measurements":
    Wiener-velocity (constant-velocity) model with the exact discretized
    process noise Q(dt) used in the course's 2d_kalman_demo.ipynb.
  - Särkkä, "Multiple target tracking" (pp. 31–33): one KF per target,
    Mahalanobis distance d² = vᵀS⁻¹v as association cost, gating, and joint
    assignment with the Hungarian algorithm; tracks deleted after going
    too long without an update.
  - Kucner, "2D LiDAR People Detection" Module 4: χ²₂ Mahalanobis gate
    (γ = 9.21, the 99 % entry of the lecture's Eq. 7 table — see __init__),
    track lifecycle NEW →(3 hits)→ CONFIRMED, delete after 5 misses.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from .leg_detection import LegMeasurement


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class Track:
    """
    Single-person track: state X^i_t ~ N(m^i_t, P^i_t).

    Attributes
    ----------
    m        : Mean state vector [x, y, vx, vy], shape (4,).
    P        : State covariance matrix, shape (4, 4).
    track_id : Unique integer identifier assigned at track creation.
    misses   : Number of consecutive scans without a matched measurement.
               The tracker deletes a track when misses > max_misses.
    hits     : Number of consecutive scans with a matched measurement
               (reset to 0 on a miss).
    confirmed: Latched True once hits reaches the confirmation threshold
               (lecture Module 4: NEW →(H_conf consecutive hits)→ CONFIRMED).
               Lets downstream stages ignore single-frame ghost detections.

    Observation model
    -----------------
    Only position is observed.  The measurement matrix H projects the 4-D
    state to a 2-D observation:

        H = [[1, 0, 0, 0],
             [0, 1, 0, 0]]

    so that  z = H @ m + noise,  noise ~ N(0, R).
    """
    m:         np.ndarray   # shape (4,): [x, y, vx, vy]
    P:         np.ndarray   # shape (4, 4)
    track_id:  int
    misses:    int = 0
    hits:      int = 0
    confirmed: bool = False
    static:    bool = False   # True once persistence shows it has not moved
                              # (lecture Module 4 p.198: static-object rejection)


# ---------------------------------------------------------------------------
# Kalman tracker
# ---------------------------------------------------------------------------

class KalmanTracker:
    """
    Multi-target constant-velocity Kalman filter with data association.

    Lifecycle of each call to update()
    -----------------------------------
    1. predict()    — propagate all tracks forward by dt.
    2. associate()  — match measurements to tracks.
    3. KF update    — correct matched tracks with measurements.
    4. Spawn        — create new tracks for unmatched measurements.
    5. Prune        — delete tracks that have been missed too many times.

    Typical usage
    -------------
    tracker = KalmanTracker(dt=0.1)
    for measurements in scan_stream:          # measurements: List[LegMeasurement]
        tracker.update(measurements)
        active_tracks = tracker.get_tracks()  # List[Track]
    """

    # Observation matrix H: extracts (x, y) from the 4-D state.
    H: np.ndarray = np.array(
        [[1, 0, 0, 0],
         [0, 1, 0, 0]],
        dtype=float,
    )

    def __init__(
        self,
        dt: float = 0.1,
        max_misses: int = 5,
        process_noise_density: float = 1.0,
        gate_chi2: float = 9.21,
        confirm_hits: int = 3,
        init_vel_std: float = 1.5,
        merge_dist: float = 0.35,
        merge_vel: float = 0.3,
    ) -> None:
        """
        Args:
            dt:         Time step between scans (seconds).
            max_misses: Delete a track after this many consecutive missed frames.
            process_noise_density:
                        Spectral density q of the white-noise acceleration
                        (m²/s³).  q = 1.0 matches the course 2d_kalman_demo;
                        larger q lets tracks follow manoeuvres faster at the
                        cost of noisier velocity estimates.
            gate_chi2:  Mahalanobis gate γ on d².  Innovations are 2-D, so
                        d² ~ χ²₂ under a correct association.  γ = 9.21 (99 %,
                        lecture Eq. 7 table).  With centimetre-level R the
                        95 % gate (5.99) is only ~12 cm wide — a track whose
                        velocity is still converging overshoots it and spawns
                        duplicate tracks; 99 % trades a negligible clutter
                        acceptance for robust association.
            confirm_hits:
                        Consecutive hits needed to mark a track confirmed
                        (lecture Module 4: H_conf = 3).
            init_vel_std:
                        1σ uncertainty on the (unobserved) initial velocity
                        of a new track (m/s).  ~1 m/s covers walking people.
            merge_dist / merge_vel:
                        Two tracks closer than merge_dist (m) with velocities
                        within merge_vel (m/s) are duplicates of one target
                        and get merged (lecture: "track merging").  The
                        velocity guard keeps genuinely crossing people apart —
                        their tracks meet in position but not in velocity.
        """
        self.dt = dt
        self.max_misses = max_misses
        self.gate_chi2 = gate_chi2
        self.confirm_hits = confirm_hits
        self.init_vel_std = init_vel_std
        self.merge_dist = merge_dist
        self.merge_vel = merge_vel
        # Static-object rejection (lecture Module 4 p.198): a confirmed track
        # whose accumulated displacement stays below `static_dist` over the
        # last `static_window` scans is flagged static (chair legs, posts…).
        self.static_window = 20          # N ≈ 2 s at 10 Hz
        self.static_dist = 0.12          # δ_min (m)
        self._pos_hist: Dict[int, list] = {}
        self.tracks: List[Track] = []
        self._next_id: int = 0
        # Measurement indices that fell inside at least one track's gate in
        # the most recent associate() call — used by the spawn rule.
        self._gated_meas_idxs: set = set()

        # Constant-velocity (Wiener velocity) state transition:
        #
        #     F = [[1, 0, dt,  0],
        #          [0, 1,  0, dt],
        #          [0, 0,  1,  0],
        #          [0, 0,  0,  1]]
        self.F = np.array(
            [[1.0, 0.0,  dt, 0.0],
             [0.0, 1.0, 0.0,  dt],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]],
            dtype=float,
        )

        # Process noise covariance Q — the exact discretization of the
        # continuous white-noise-acceleration model (Särkkä, "Dynamics and
        # measurements" p. 24; identical to the course 2d_kalman_demo):
        #
        #     Q = q · [[dt³/3,     0, dt²/2,     0],
        #              [    0, dt³/3,     0, dt²/2],
        #              [dt²/2,     0,    dt,     0],
        #              [    0, dt²/2,     0,    dt]]
        #
        # The dt³/dt² coupling terms tie position noise to velocity noise,
        # which a plain diagonal Q ignores.
        q = process_noise_density
        self.Q = q * np.array(
            [[dt**3 / 3.0, 0.0,         dt**2 / 2.0, 0.0],
             [0.0,         dt**3 / 3.0, 0.0,         dt**2 / 2.0],
             [dt**2 / 2.0, 0.0,         dt,          0.0],
             [0.0,         dt**2 / 2.0, 0.0,         dt]],
            dtype=float,
        )

    # ------------------------------------------------------------------
    # Stage 2a — predict
    # ------------------------------------------------------------------

    def predict(self) -> None:
        """
        Propagate every active track forward one time step.

        For each track i apply the constant-velocity model:

            m^i_t|t-1  =  F  @  m^i_t-1
            P^i_t|t-1  =  F  @  P^i_t-1  @  F.T  +  Q

        This is called automatically at the start of every update() cycle.
        """
        for track in self.tracks:
            track.m = self.F @ track.m
            track.P = self.F @ track.P @ self.F.T + self.Q
            # Keep P symmetric against floating-point drift.
            track.P = 0.5 * (track.P + track.P.T)

    # ------------------------------------------------------------------
    # Stage 2b — data association
    # ------------------------------------------------------------------

    def associate(
        self,
        measurements: List[LegMeasurement],
    ) -> List[Tuple[int, int]]:
        """
        Match measurements to existing tracks (global nearest-neighbour).

        Args:
            measurements: LegMeasurement list from the current scan.

        Returns:
            List of (track_index, measurement_index) pairs where
            track_index  indexes into self.tracks and
            meas_index   indexes into measurements.

            Unmatched measurements → passed to update() for track spawning.
            Unmatched tracks       → miss counter incremented in update().

        Method (lecture "Gated NN Association", Module 4 p. 83 + Särkkä MTT
        pp. 31–33):

        1. Cost matrix C[i, j] = squared Mahalanobis distance between track
           i's predicted measurement and measurement j:

               d²_ij = v.T @ inv(S_ij) @ v
               where v    = [meas.x - m[0], meas.y - m[1]]
                     S_ij = H @ P @ H.T + R_j

           Unlike a Euclidean cost, d² weighs the innovation by both the
           track's and the detection's uncertainty, so the gate adapts to
           range (far detections have larger R → wider gate).

        2. Gate: any pair with d² > gate_chi2 is forbidden (cost set high).
           γ = χ²₂ quantile (9.21 → 99 %): a correct association passes the
           gate with that probability.

        3. Solve the joint assignment with the Hungarian algorithm
           (scipy.optimize.linear_sum_assignment) and drop assignments the
           solver was forced to make through a gated-out pair.

        If there are no tracks or no measurements, return [].
        """
        n_tracks = len(self.tracks)
        n_meas = len(measurements)
        self._gated_meas_idxs = set()
        if n_tracks == 0 or n_meas == 0:
            return []

        BIG = 1e6   # finite "forbidden" cost — keeps the LP feasible
        C = np.full((n_tracks, n_meas), BIG, dtype=float)

        for i, track in enumerate(self.tracks):
            z_pred = self.H @ track.m                      # predicted (x, y)
            P_pos = self.H @ track.P @ self.H.T            # 2×2 position cov
            for j, meas in enumerate(measurements):
                v = np.array([meas.x, meas.y]) - z_pred    # innovation
                R = np.array([[meas.Rxx, meas.Rxy],
                              [meas.Rxy, meas.Ryy]])
                S = P_pos + R                              # innovation cov
                d2 = float(v @ np.linalg.solve(S, v))      # Mahalanobis²
                if d2 <= self.gate_chi2:
                    C[i, j] = d2
                    self._gated_meas_idxs.add(j)

        row_ind, col_ind = linear_sum_assignment(C)
        return [
            (int(i), int(j))
            for i, j in zip(row_ind, col_ind)
            if C[i, j] <= self.gate_chi2                   # drop forced pairs
        ]

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _spawn_track(self, meas: LegMeasurement) -> None:
        """Initialise a new Track from an unmatched measurement."""
        m = np.array([meas.x, meas.y, 0.0, 0.0], dtype=float)

        # Position uncertainty is seeded from the measurement's own
        # covariance (what the detector actually knows about this point);
        # velocity is unobserved, so it gets a generous prior of
        # init_vel_std² that the filter shrinks as evidence arrives.
        P = np.diag([
            max(meas.Rxx, 1e-4),
            max(meas.Ryy, 1e-4),
            self.init_vel_std ** 2,
            self.init_vel_std ** 2,
        ]).astype(float)

        self.tracks.append(Track(m=m, P=P, track_id=self._next_id))
        self._next_id += 1

    # ------------------------------------------------------------------
    # Stage 2c — full update cycle
    # ------------------------------------------------------------------

    def update(self, measurements: List[LegMeasurement]) -> None:
        """
        Run one complete tracking cycle: predict → associate → KF update.

        Steps
        -----
        1. Call self.predict() to propagate all tracks.
        2. Call self.associate(measurements) to get matched pairs.
        3. For each matched (track_index, meas_index) pair, apply the KF
           update equations:

               z   =  np.array([meas.x, meas.y])
               R   =  np.array([[meas.Rxx, meas.Rxy],
                                 [meas.Rxy, meas.Ryy]])
               y   =  z  -  H @ m              # innovation
               S   =  H  @  P  @  H.T  +  R   # innovation covariance
               K   =  P  @  H.T  @  inv(S)    # Kalman gain
               m   =  m  +  K  @  y            # updated mean
               P   =  (I-KH) P (I-KH).T + K R K.T   # Joseph form

           The Joseph form is algebraically identical to P - K S K.T but
           stays positive-semidefinite under floating-point rounding.

        4. For matched tracks: reset track.misses = 0 (and count the hit).
           For unmatched tracks: increment track.misses by 1.

        5. Spawn a new Track (via self._spawn_track) for every measurement
           that was NOT matched to any existing track.

        6. Remove tracks from self.tracks where track.misses > self.max_misses.
        """
        self.predict()
        assignments = self.associate(measurements)

        matched_track_idxs = {ti for ti, _ in assignments}
        matched_meas_idxs  = {mi for _, mi in assignments}

        I4 = np.eye(4)

        # Step 3 — KF measurement update for every matched pair.
        for ti, mi in assignments:
            track = self.tracks[ti]
            meas = measurements[mi]

            z = np.array([meas.x, meas.y], dtype=float)
            R = np.array([[meas.Rxx, meas.Rxy],
                          [meas.Rxy, meas.Ryy]], dtype=float)

            v = z - self.H @ track.m                       # innovation
            S = self.H @ track.P @ self.H.T + R            # innovation cov
            # K = P Hᵀ S⁻¹, via a solve on symmetric S (no explicit inverse)
            K = np.linalg.solve(S, self.H @ track.P).T

            track.m = track.m + K @ v
            I_KH = I4 - K @ self.H
            track.P = I_KH @ track.P @ I_KH.T + K @ R @ K.T   # Joseph form
            track.P = 0.5 * (track.P + track.P.T)

        # Step 4 — hit/miss bookkeeping and confirmation lifecycle.
        for idx, track in enumerate(self.tracks):
            if idx in matched_track_idxs:
                track.misses = 0
                track.hits += 1
                if track.hits >= self.confirm_hits:
                    track.confirmed = True
            else:
                track.misses += 1
                track.hits = 0      # confirmation needs *consecutive* hits

        # Step 5 — spawn new tracks, but ONLY from measurements that fell
        # outside EVERY existing gate (Särkkä MTT p. 33: "if no target
        # satisfies this, this is either new target or outlier").  An
        # unmatched measurement that lies inside some track's gate is most
        # likely a duplicate detection of an already-tracked person — turning
        # those into tracks causes duplicate-track churn.
        #
        # Walking-leg guard (measured on a recorded stride, frames 196-216 of
        # the T2 session): mid-stride the front leg lands ~0.5 m ahead of the
        # tracked body centre — outside every gate — and would spawn a
        # duplicate that then steals the track when the legs come together.
        # So ALSO suppress spawning near a confirmed MOVING track (its other
        # leg zone). Static tracks don't suppress, so a new person appearing
        # next to furniture is still picked up.
        for mi, meas in enumerate(measurements):
            if mi in matched_meas_idxs or mi in self._gated_meas_idxs:
                continue
            near_moving = False
            for t in self.tracks:
                if not t.confirmed:
                    continue
                if (np.hypot(t.m[2], t.m[3]) > 0.2 and
                        np.hypot(t.m[0] - meas.x, t.m[1] - meas.y) < 0.6):
                    near_moving = True
                    break
            if not near_moving:
                self._spawn_track(meas)

        # Step 6 — prune tracks that have coasted too long.
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

        # Step 7 — merge duplicate tracks of the same target (lecture p. 21:
        # "track merging").  Same position AND same velocity = one person;
        # crossing people share position only briefly but never velocity.
        self._merge_duplicate_tracks()

        # Step 8 — static-object rejection (lecture Module 4 p. 198).  Flag a
        # confirmed track as static when its displacement over the last
        # `static_window` scans stays below `static_dist` (furniture, posts).
        live_ids = {t.track_id for t in self.tracks}
        self._pos_hist = {tid: h for tid, h in self._pos_hist.items()
                          if tid in live_ids}
        for t in self.tracks:
            h = self._pos_hist.setdefault(t.track_id, [])
            h.append((float(t.m[0]), float(t.m[1])))
            if len(h) > self.static_window:
                del h[0]
            if t.confirmed and len(h) >= self.static_window:
                dx = h[-1][0] - h[0][0]
                dy = h[-1][1] - h[0][1]
                t.static = float(np.hypot(dx, dy)) < self.static_dist
            else:
                t.static = False

    def _merge_duplicate_tracks(self) -> None:
        """Collapse track pairs that agree in position and velocity."""
        removed: set = set()
        for i in range(len(self.tracks)):
            ti = self.tracks[i]
            if ti.track_id in removed:
                continue
            for j in range(i + 1, len(self.tracks)):
                tj = self.tracks[j]
                if tj.track_id in removed:
                    continue
                dp = float(np.hypot(ti.m[0] - tj.m[0], ti.m[1] - tj.m[1]))
                dv = float(np.hypot(ti.m[2] - tj.m[2], ti.m[3] - tj.m[3]))
                if dp >= self.merge_dist:
                    continue
                older, newer = (ti, tj) if ti.track_id < tj.track_id else (tj, ti)
                if not newer.confirmed:
                    # A newborn this close to an existing track is a duplicate
                    # detection, not a new person — absorb it immediately
                    # (its velocity estimate is still meaningless, so no
                    # state adoption; the older track's estimate stands).
                    removed.add(newer.track_id)
                elif dv < self.merge_vel:
                    # Two mature tracks agreeing in position AND velocity =
                    # one target. Keep the older identity; adopt the fresher
                    # estimate. (Crossing people agree in position only —
                    # the velocity guard keeps them apart.)
                    fresher = ti if ti.misses <= tj.misses else tj
                    older.m = fresher.m.copy()
                    older.P = fresher.P.copy()
                    older.misses = fresher.misses
                    older.hits = max(ti.hits, tj.hits)
                    older.confirmed = True
                    removed.add(newer.track_id)
        if removed:
            self.tracks = [t for t in self.tracks if t.track_id not in removed]

    # ------------------------------------------------------------------
    # Prediction of future positions (course task 2.2)
    # ------------------------------------------------------------------

    def predict_ahead(self, horizon: float) -> Dict[int, np.ndarray]:
        """
        Forecast each track's future positions by iterating the KF
        prediction step, WITHOUT modifying the tracker state.

        Iterates m ← F m (and P ← F P Fᵀ + Q for the uncertainty) for
        n = round(horizon / dt) steps, exactly as the course task asks:
        "try how well you can predict the locations of the people by
        iterating the Kalman filter prediction step for each target".

        Args:
            horizon: How far into the future to predict (seconds).

        Returns:
            Dict mapping track_id → array of shape (n_steps, 3) whose rows
            are [x, y, σ_pos] — predicted position and the 1σ position
            uncertainty (max eigenvalue of the position covariance) at each
            future step.  The growing σ shows how confidence decays with
            prediction horizon.
        """
        n_steps = max(1, int(round(horizon / self.dt)))
        out: Dict[int, np.ndarray] = {}

        for track in self.tracks:
            m = track.m.copy()
            P = track.P.copy()
            rows = np.empty((n_steps, 3))
            for k in range(n_steps):
                m = self.F @ m
                P = self.F @ P @ self.F.T + self.Q
                sigma = float(np.sqrt(max(
                    np.linalg.eigvalsh(P[:2, :2])[-1], 0.0)))
                rows[k] = (m[0], m[1], sigma)
            out[track.track_id] = rows

        return out

    # ------------------------------------------------------------------
    # Read-only access
    # ------------------------------------------------------------------

    def get_tracks(self) -> List[Track]:
        """Return a snapshot of the current active track list."""
        return list(self.tracks)
