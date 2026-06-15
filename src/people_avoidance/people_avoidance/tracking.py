"""
Stage 2: multi-target constant-velocity Kalman filter with gated
nearest-neighbour data association, track lifecycle, and merging.

State m = [x, y, vx, vy] in the laser/robot frame; covariance P is 4x4.
Input: List[LegMeasurement] per scan; output: List[Track] across scans.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from .leg_detection import LegMeasurement


@dataclass
class Track:
    """
    Single-person track: state X ~ N(m, P), position-only observation.

    Attributes
    ----------
    m        : Mean state vector [x, y, vx, vy], shape (4,).
    P        : State covariance matrix, shape (4, 4).
    track_id : Unique integer identifier assigned at track creation.
    misses   : Consecutive scans without a matched measurement; the track is
               deleted when misses > max_misses.
    hits     : Consecutive scans with a matched measurement (reset to 0 on a miss).
    confirmed: Latched True once hits reaches the confirmation threshold; lets
               downstream stages ignore single-frame ghost detections.
    static   : True once persistence shows the track has not moved.
    """
    m:         np.ndarray   # shape (4,): [x, y, vx, vy]
    P:         np.ndarray   # shape (4, 4)
    track_id:  int
    misses:    int = 0
    hits:      int = 0
    confirmed: bool = False
    static:    bool = False   # True once persistence shows it has not moved


class KalmanTracker:
    """Multi-target constant-velocity Kalman filter with gated NN data association."""

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
        confirm_hits: int = 4,
        init_vel_std: float = 1.5,
        merge_dist: float = 0.35,
        merge_vel: float = 0.45,
    ) -> None:
        """
        Args:
            dt:         Time step between scans (seconds).
            max_misses: Delete a track after this many consecutive missed frames.
            process_noise_density:
                        Spectral density q of the white-noise acceleration (m²/s³);
                        larger q follows manoeuvres faster but noisier.
            gate_chi2:  Mahalanobis gate γ on d² (innovations are 2-D, d² ~ χ²₂);
                        9.21 ≈ the 99% quantile.
            confirm_hits: Consecutive hits needed to mark a track confirmed.
            init_vel_std: 1σ prior on the unobserved initial velocity (m/s).
            merge_dist / merge_vel:
                        Two tracks closer than merge_dist (m) with velocities
                        within merge_vel (m/s) are merged as one target; the
                        velocity guard keeps crossing people apart.
        """
        self.dt = dt
        self.max_misses = max_misses
        self.gate_chi2 = gate_chi2
        self.confirm_hits = confirm_hits
        self.init_vel_std = init_vel_std
        self.merge_dist = merge_dist
        self.merge_vel = merge_vel
        self.coast_damp = 0.6     # per-cycle velocity decay while unmatched
        # Static-object rejection: flag a confirmed track static when its
        # displacement over the last static_window scans stays below static_dist.
        self.static_window = 20          # N ≈ 2 s at 10 Hz
        self.static_dist = 0.12          # metres
        self._pos_hist: Dict[int, list] = {}
        # Net-displacement speed clamp: a near-stationary centroid that hops
        # (leg-pairing alternation) reads as phantom velocity; rescale a track's
        # velocity to its net displacement rate when the two disagree.
        self.speed_clamp  = True
        self.clamp_window = 8
        self.tracks: List[Track] = []
        self._next_id: int = 0
        # Measurement indices that fell inside at least one track's gate in the
        # most recent associate() call; used by the spawn rule.
        self._gated_meas_idxs: set = set()

        # Constant-velocity (Wiener-velocity) state transition.
        self.F = np.array(
            [[1.0, 0.0,  dt, 0.0],
             [0.0, 1.0, 0.0,  dt],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]],
            dtype=float,
        )

        # Exact discretized white-noise-acceleration process noise Q: the
        # dt³/3, dt²/2 coupling ties position noise to velocity noise (a plain
        # diagonal Q ignores it).
        q = process_noise_density
        self.Q = q * np.array(
            [[dt**3 / 3.0, 0.0,         dt**2 / 2.0, 0.0],
             [0.0,         dt**3 / 3.0, 0.0,         dt**2 / 2.0],
             [dt**2 / 2.0, 0.0,         dt,          0.0],
             [0.0,         dt**2 / 2.0, 0.0,         dt]],
            dtype=float,
        )

    def predict(self) -> None:
        """
        Propagate every active track forward one time step (constant-velocity model).

        Coast damping: while a track is unmatched (misses > 0) its velocity decays
        each cycle, keeping it near the last sighting so a returning person falls
        back inside its gate.
        """
        for track in self.tracks:
            if track.misses > 0:
                track.m = track.m.copy()
                track.m[2] *= self.coast_damp
                track.m[3] *= self.coast_damp
            track.m = self.F @ track.m
            track.P = self.F @ track.P @ self.F.T + self.Q
            # Keep P symmetric against floating-point drift.
            track.P = 0.5 * (track.P + track.P.T)

    def associate(
        self,
        measurements: List[LegMeasurement],
    ) -> List[Tuple[int, int]]:
        """
        Match measurements to existing tracks (gated global nearest-neighbour).

        Args:
            measurements: LegMeasurement list from the current scan.

        Returns:
            List of (track_index, meas_index) pairs indexing self.tracks and
            measurements. Unmatched measurements spawn tracks in update();
            unmatched tracks have their miss counter incremented there.

        Cost is squared Mahalanobis distance d² = vᵀS⁻¹v, which weighs the
        innovation by both track and detection uncertainty (so the gate adapts
        to range); pairs with d² > gate_chi2 are gated out, and the Hungarian
        algorithm solves the joint assignment.
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

    def _spawn_track(self, meas: LegMeasurement) -> None:
        """Initialise a new Track from an unmatched measurement."""
        m = np.array([meas.x, meas.y, 0.0, 0.0], dtype=float)

        # Seed position uncertainty from the measurement covariance; velocity
        # is unobserved, so give it a broad init_vel_std² prior.
        P = np.diag([
            max(meas.Rxx, 1e-4),
            max(meas.Ryy, 1e-4),
            self.init_vel_std ** 2,
            self.init_vel_std ** 2,
        ]).astype(float)

        self.tracks.append(Track(m=m, P=P, track_id=self._next_id))
        self._next_id += 1

    def update(self, measurements: List[LegMeasurement]) -> None:
        """Run one tracking cycle: predict → associate → KF update → spawn → prune → merge."""
        self.predict()
        assignments = self.associate(measurements)

        matched_track_idxs = {ti for ti, _ in assignments}
        matched_meas_idxs  = {mi for _, mi in assignments}

        I4 = np.eye(4)

        # KF measurement update for every matched pair.
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

        # Hit/miss bookkeeping and confirmation lifecycle.
        for idx, track in enumerate(self.tracks):
            if idx in matched_track_idxs:
                track.misses = 0
                track.hits += 1
                if track.hits >= self.confirm_hits:
                    track.confirmed = True
            else:
                track.misses += 1
                track.hits = 0      # confirmation needs *consecutive* hits

        # Spawn only from measurements outside EVERY existing gate; one inside a
        # gate is most likely a duplicate detection of a tracked person. Also
        # suppress spawning near a confirmed MOVING track (its other leg zone)
        # to avoid mid-stride duplicates; static tracks don't suppress, so a new
        # person next to furniture is still picked up.
        for mi, meas in enumerate(measurements):
            if mi in matched_meas_idxs or mi in self._gated_meas_idxs:
                continue
            if getattr(meas, 'occluded', False):
                continue            # never spawn a track from a shadow-cut blob
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

        # Prune tracks that have coasted too long.
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

        # Merge duplicate tracks: same position AND velocity = one person;
        # crossing people share position only briefly, never velocity.
        self._merge_duplicate_tracks()

        # Flag a confirmed track static when its displacement over the last
        # static_window scans stays below static_dist (furniture, posts).
        live_ids = {t.track_id for t in self.tracks}
        self._pos_hist = {tid: h for tid, h in self._pos_hist.items()
                          if tid in live_ids}
        for t in self.tracks:
            h = self._pos_hist.setdefault(t.track_id, [])
            h.append((float(t.m[0]), float(t.m[1])))
            if len(h) > self.static_window:
                del h[0]
            if self.speed_clamp and len(h) >= 4:
                k = min(len(h), self.clamp_window)
                rate = float(np.hypot(h[-1][0] - h[-k][0],
                                      h[-1][1] - h[-k][1])) / (k * self.dt)
                sp = float(np.hypot(t.m[2], t.m[3]))
                if sp > max(2.0 * rate, rate + 0.15):
                    sc = (rate / sp) if sp > 1e-6 else 0.0
                    t.m[2] *= sc
                    t.m[3] *= sc
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
                    # A newborn this close is a duplicate detection, not a new
                    # person — absorb it (no state adoption, its velocity is
                    # still meaningless).
                    removed.add(newer.track_id)
                elif dv < self.merge_vel:
                    # Two mature tracks agreeing in position AND velocity = one
                    # target; keep the older identity, adopt the fresher estimate.
                    fresher = ti if ti.misses <= tj.misses else tj
                    older.m = fresher.m.copy()
                    older.P = fresher.P.copy()
                    older.misses = fresher.misses
                    older.hits = max(ti.hits, tj.hits)
                    older.confirmed = True
                    removed.add(newer.track_id)
        if removed:
            self.tracks = [t for t in self.tracks if t.track_id not in removed]

    def predict_ahead(self, horizon: float) -> Dict[int, np.ndarray]:
        """
        Forecast each track's future positions by iterating the KF prediction
        step over round(horizon / dt) steps, WITHOUT modifying tracker state.

        Args:
            horizon: How far into the future to predict (seconds).

        Returns:
            Dict mapping track_id → array of shape (n_steps, 3) whose rows are
            [x, y, σ_pos]: predicted position and the 1σ position uncertainty
            (max eigenvalue of the position covariance) at each future step.
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

    def get_tracks(self) -> List[Track]:
        """Return a snapshot of the current active track list."""
        return list(self.tracks)
