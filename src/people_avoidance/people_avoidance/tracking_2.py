"""
tracking_2.py — alternative Stage 2 tracker: greedy Nearest-Neighbour KF.

Faithful ROS port of the approach in notebooks/Copy_of_2d_kalman_demo.ipynb
(+ notebooks/pipeline_explanation.md): the course 2D Kalman demo extended to
multiple targets with greedy NN data association.

How it differs from tracking.py (the Hungarian/lifecycle tracker):

  - Association : greedy nearest-neighbour — all gated (track, measurement)
                  pairs are sorted by ascending Mahalanobis distance and
                  assigned first-come-first-served (notebook
                  `nearest_neighbor_assignment`), instead of solving the
                  joint assignment with the Hungarian algorithm.
  - Gate        : chi2.ppf(0.95, df=2) ≈ 5.991 (the notebook's 95 % gate).
  - Update      : the demo's plain  P = P − K S Kᵀ  form (not Joseph).
  - Lifecycle   : measurement-driven birth (a measurement no track claims
                  becomes a new track at [x, y, 0, 0] with P₀ = 10·I, the
                  demo's prior); a track with no gated measurement COASTS —
                  the prediction is kept as the estimate.  There is no
                  confirmation logic, no duplicate merging, and no
                  spawn-gate suppression.

Liveness addition (not in the notebook, which runs on a fixed-length
simulation): tracks that have coasted for more than `max_misses`
consecutive scans are dropped, so a live feed does not accumulate tracks
without bound.  Set max_misses very high to approximate the notebook's
never-delete behaviour.

The public interface mirrors tracking.py (update / get_tracks /
predict_ahead), so the two trackers are drop-in interchangeable.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
from scipy.stats import chi2

from .leg_detection import LegMeasurement


@dataclass
class Track:
    """Greedy-NN track. Same field names as tracking.Track for drop-in use.

    `confirmed` is always True from birth — this approach has no
    confirmation stage (every measurement-born track is a track).
    `hits`/`misses` count matched / consecutively-coasted scans.
    """
    m:         np.ndarray   # shape (4,): [x, y, vx, vy]
    P:         np.ndarray   # shape (4, 4)
    track_id:  int
    misses:    int = 0
    hits:      int = 0
    confirmed: bool = True


class GreedyNNTracker:
    """
    Multi-target tracker: one Kalman filter per track, greedy NN association.

    Per scan (notebook `nn_multitarget_filter`):
      1. PREDICT every track:      m ← A m,  P ← A P Aᵀ + Q
      2. ASSOCIATE greedily:       sort gated Mahalanobis pairs ascending,
                                   assign while both sides are free
      3. UPDATE matched tracks;    COAST unmatched ones (keep prediction)
      4. BIRTH a track from every measurement no track claimed
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
        max_misses: int = 30,
        process_noise_density: float = 1.0,
        gate_prob: float = 0.95,
        init_var: float = 10.0,
    ) -> None:
        """
        Args:
            dt:          Time step between scans (seconds).
            max_misses:  Liveness addition — drop a track after this many
                         consecutive coasted scans (notebook never deletes).
            process_noise_density:
                         Spectral density q of the Wiener-velocity Q
                         (q = 1.0 in the notebook).
            gate_prob:   Gate confidence; threshold = chi2.ppf(gate_prob, 2).
                         0.95 → 5.991, the notebook's gate.
            init_var:    Diagonal of the birth covariance P₀ = init_var·I
                         (the demo uses 10·I).
        """
        self.dt = dt
        self.max_misses = max_misses
        self.gate_chi2 = float(chi2.ppf(gate_prob, df=2))
        self.init_var = init_var
        self.tracks: List[Track] = []
        self._next_id: int = 0

        # Same Wiener-velocity model as the demo (cells 8-9).
        self.F = np.array(
            [[1.0, 0.0,  dt, 0.0],
             [0.0, 1.0, 0.0,  dt],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]],
            dtype=float,
        )
        q = process_noise_density
        self.Q = q * np.array(
            [[dt**3 / 3.0, 0.0,         dt**2 / 2.0, 0.0],
             [0.0,         dt**3 / 3.0, 0.0,         dt**2 / 2.0],
             [dt**2 / 2.0, 0.0,         dt,          0.0],
             [0.0,         dt**2 / 2.0, 0.0,         dt]],
            dtype=float,
        )

    # ------------------------------------------------------------------

    def predict(self) -> None:
        """kalman_predict for every track:  m ← F m,  P ← F P Fᵀ + Q."""
        for track in self.tracks:
            track.m = self.F @ track.m
            track.P = self.F @ track.P @ self.F.T + self.Q

    def associate(
        self,
        measurements: List[LegMeasurement],
    ) -> List[Tuple[int, int]]:
        """
        Greedy NN assignment (notebook `nearest_neighbor_assignment`).

        Build the gated Mahalanobis cost matrix, collect all valid
        (cost, track, measurement) triples, sort ascending, and assign
        greedily while both the track and the measurement are still free.

        Returns (track_index, measurement_index) pairs.
        """
        n_tracks = len(self.tracks)
        n_meas = len(measurements)
        if n_tracks == 0 or n_meas == 0:
            return []

        costs = np.full((n_tracks, n_meas), np.inf)
        for t, track in enumerate(self.tracks):
            z_pred = self.H @ track.m
            P_pos = self.H @ track.P @ self.H.T
            for j, meas in enumerate(measurements):
                v = np.array([meas.x, meas.y]) - z_pred
                R = np.array([[meas.Rxx, meas.Rxy],
                              [meas.Rxy, meas.Ryy]])
                S = P_pos + R
                d2 = float(v @ np.linalg.solve(S, v))
                if d2 < self.gate_chi2:               # gate reject otherwise
                    costs[t, j] = d2

        valid_pairs = sorted(
            (costs[t, j], t, j)
            for t in range(n_tracks)
            for j in range(n_meas)
            if np.isfinite(costs[t, j])
        )

        assignments: List[Tuple[int, int]] = []
        used_tracks: set = set()
        used_meas: set = set()
        for _, t, j in valid_pairs:
            if t not in used_tracks and j not in used_meas:
                assignments.append((t, j))
                used_tracks.add(t)
                used_meas.add(j)
        return assignments

    def _spawn_track(self, meas: LegMeasurement) -> None:
        """Measurement-driven birth: [x, y, 0, 0] with P₀ = init_var·I."""
        m = np.array([meas.x, meas.y, 0.0, 0.0], dtype=float)
        P = np.eye(4, dtype=float) * self.init_var
        self.tracks.append(Track(m=m, P=P, track_id=self._next_id))
        self._next_id += 1

    def update(self, measurements: List[LegMeasurement]) -> None:
        """One cycle: predict → greedy associate → update/coast → birth."""
        self.predict()
        assignments = self.associate(measurements)

        matched_track_idxs = {ti for ti, _ in assignments}
        matched_meas_idxs = {mi for _, mi in assignments}

        # Update matched tracks — the demo's kalman_update form.
        for ti, mi in assignments:
            track = self.tracks[ti]
            meas = measurements[mi]
            z = np.array([meas.x, meas.y], dtype=float)
            R = np.array([[meas.Rxx, meas.Rxy],
                          [meas.Rxy, meas.Ryy]], dtype=float)
            S = self.H @ track.P @ self.H.T + R
            K = np.linalg.solve(S, self.H @ track.P).T   # P Hᵀ S⁻¹
            track.m = track.m + K @ (z - self.H @ track.m)
            track.P = track.P - K @ S @ K.T              # demo form
            track.misses = 0
            track.hits += 1

        # Coast unmatched tracks: estimate stays the prediction.
        for idx, track in enumerate(self.tracks):
            if idx not in matched_track_idxs:
                track.misses += 1
                track.hits = 0

        # Birth: every unclaimed measurement becomes a new track.
        for mi, meas in enumerate(measurements):
            if mi not in matched_meas_idxs:
                self._spawn_track(meas)

        # Liveness addition: drop tracks that coasted too long.
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

    # ------------------------------------------------------------------

    def predict_ahead(self, horizon: float) -> Dict[int, np.ndarray]:
        """Iterated KF prediction (same contract as tracking.predict_ahead)."""
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

    def get_tracks(self) -> List[Track]:
        """Return a snapshot of the current active track list."""
        return list(self.tracks)
