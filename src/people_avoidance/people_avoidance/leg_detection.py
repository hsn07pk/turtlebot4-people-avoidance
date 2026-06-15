"""
Stage 1 — LiDAR leg detection: LaserScan -> List[LegMeasurement].

Adaptive jump-distance segmentation, PCA shape gating, and a polar->Cartesian
Jacobian covariance for each detected person.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import math
import numpy as np
from sensor_msgs.msg import LaserScan


# Sensor / model constants
RANGE_STD          = 0.02    # sigma_r, range noise std (m)
SEG_NOISE_K        = 3.0     # k in tau(r) = r*dtheta + k*sigma_r
MIN_CLUSTER_POINTS = 3       # n_min — drop noise clusters; still detects legs to ~3.5 m
CIRCULARITY_MIN    = 0.15    # reject clearly wall-like (elongated) clusters
PAIR_MIN_DIST      = 0.05    # min centre-to-centre to call two clusters a leg pair (m)
COV_EPS            = 1e-9    # diagonal floor so R stays positive-definite for the KF
DETECT_MAX_RANGE   = 3.5     # range cap — far returns flicker too hard to cluster
GAIT_STD           = 0.08    # gait wobble ~8 cm: person-centre noise, not sensor noise


@dataclass
class LegMeasurement:
    """
    One detected person: 2-D position with observation covariance, laser frame
    (x forward, y left), metres / m².

    x, y     : person position.
    Rxx, Ryy : variances along x and y; Rxy: cross-covariance term.
    R = [[Rxx, Rxy], [Rxy, Ryy]] is the symmetric 2x2 covariance for the KF.
    """
    x:   float
    y:   float
    Rxx: float
    Rxy: float
    Ryy: float
    occluded: bool = False   # edge is occlusion-cut: may update a track, never spawn one


def scan_to_cartesian(scan: LaserScan) -> np.ndarray:
    """
    Convert LaserScan polar readings to (x, y) points in the laser frame.

    Invalid ranges (inf, nan, out-of-bounds) are dropped. Returns an (N, 2)
    array, possibly empty.
    """
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ranges = np.array(scan.ranges, dtype=float)

    valid = (
        np.isfinite(ranges)
        & (ranges > scan.range_min)
        & (ranges < scan.range_max)
    )
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


# Stage 1a — segmentation

def segment_scan(
    points: np.ndarray,
    distance_threshold: float = 0.1,
    angle_increment: float | None = None,
    sigma_r: float = RANGE_STD,
    k: float = SEG_NOISE_K,
) -> List[np.ndarray]:
    """
    Split a scan-ordered point cloud into contiguous clusters.

    Two consecutive points join the same segment when their gap is below the
    threshold. When *angle_increment* is given, the threshold is adaptive:
    tau(r) = r * Δθ + k * sigma_r at the previous beam's range — it grows with
    range (point spacing ~ r·Δθ) and floors at distance_threshold up close.

    Args:
        points:             (N, 2) Cartesian scan points, in scan order.
        distance_threshold: Gap floor / fixed threshold when angle_increment is None (m).
        angle_increment:    Beam angular step Δθ (rad); enables the adaptive threshold.
        sigma_r:            Range noise std (m).
        k:                  Noise multiplier (default 3).

    Returns:
        List of (K_i, 2) arrays, one per segment. Empty if points is empty.
    """
    n = len(points)
    if n == 0:
        return []

    segments: List[np.ndarray] = []
    current = [points[0]]

    for i in range(1, n):
        gap = float(np.linalg.norm(points[i] - points[i - 1]))

        if angle_increment is not None:
            # Adaptive threshold tau(r)=r*dtheta+k*sigma_r, floored by distance_threshold.
            r_prev = float(np.linalg.norm(points[i - 1]))
            threshold = max(distance_threshold, r_prev * float(angle_increment) + k * sigma_r)
        else:
            threshold = distance_threshold

        if gap > threshold:
            segments.append(np.asarray(current))
            current = []

        current.append(points[i])

    segments.append(np.asarray(current))
    return segments


# Stage 1b — leg detection and covariance assignment

def _cluster_shape(seg: np.ndarray) -> Tuple[float, float]:
    """
    PCA geometric features of a cluster.

    Returns:
        width        : extent along the principal axis (m).
        circularity  : lambda_min / lambda_max in [0, 1]; ~1 round blob (leg),
                       ~0 elongated run (wall).
    """
    npts = seg.shape[0]
    if npts < 2:
        return 0.0, 0.0
    centred = seg - seg.mean(axis=0)
    cov = (centred.T @ centred) / npts
    evals, evecs = np.linalg.eigh(cov)          # ascending: evals[0] <= evals[1]
    lam_min, lam_max = float(evals[0]), float(evals[1])
    e1 = evecs[:, 1]                            # principal axis
    proj = centred @ e1
    width = float(proj.max() - proj.min())
    circularity = (lam_min / lam_max) if lam_max > 1e-12 else 0.0
    return width, circularity


def _cluster_covariance(
    centroid: np.ndarray, n: int, sigma_theta: float
) -> np.ndarray:
    """
    Centroid covariance: the polar->Cartesian Jacobian covariance
    Sigma_xy = J * diag(sigma_r^2, sigma_theta^2) * J^T at (r, theta), divided
    by n (covariance-of-the-mean: more points -> tighter estimate).
    """
    r = float(np.linalg.norm(centroid))
    theta = float(np.arctan2(centroid[1], centroid[0]))
    ct, st = np.cos(theta), np.sin(theta)
    sr2 = RANGE_STD ** 2
    r2t2 = (r * sigma_theta) ** 2
    Sxx = sr2 * ct * ct + r2t2 * st * st
    Sxy = (sr2 - r2t2) * st * ct
    Syy = sr2 * st * st + r2t2 * ct * ct
    sigma_xy = np.array([[Sxx, Sxy], [Sxy, Syy]], dtype=float)
    return sigma_xy / max(n, 1) + COV_EPS * np.eye(2)


def _to_measurement(p: np.ndarray, R: np.ndarray) -> LegMeasurement:
    """Pack a (position, 2x2 covariance) pair into a LegMeasurement."""
    g = GAIT_STD ** 2
    return LegMeasurement(
        x=float(p[0]), y=float(p[1]),
        Rxx=float(R[0, 0]) + g, Rxy=float(R[0, 1]), Ryy=float(R[1, 1]) + g,
    )


def detect_legs(
    scan: LaserScan,
    distance_threshold: float = 0.1,
    leg_radius: float = 0.10,
    max_leg_width: float = 0.25,
) -> List[LegMeasurement]:
    """
    Detect people from a LaserScan via geometric leg-pair matching: segment,
    gate clusters by width/circularity, pair candidates within max_leg_width
    (person at the midpoint), and assign each a Jacobian covariance R.

    Args:
        scan:               Incoming LaserScan message.
        distance_threshold: Segmentation gap threshold (m).
        leg_radius:         Expected radius of a single leg segment (m).
        max_leg_width:      Max centre-to-centre distance of two paired legs (m).

    Returns:
        List of LegMeasurement.
    """
    points = scan_to_cartesian(scan)
    if points.shape[0] == 0:
        return []
    # Range cap — far returns flicker too hard to form stable leg clusters.
    points = points[np.hypot(points[:, 0], points[:, 1]) <= DETECT_MAX_RANGE]
    if points.shape[0] == 0:
        return []

    # Occlusion-cut: a cluster edge whose neighbouring beam is much closer
    # continues behind a nearer object; such edges sweep moving wall ghosts as
    # an occluder passes, so clusters cut on either edge are flagged.
    _ranges = np.asarray(scan.ranges, dtype=float)
    _nbeams = len(_ranges)

    def _occlusion_cut(pt) -> bool:
        ang = math.atan2(pt[1], pt[0])
        idx = int(round((ang - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= _nbeams:
            return False
        r_here = float(np.hypot(pt[0], pt[1]))
        for j in (idx - 1, idx - 2, idx + 1, idx + 2):
            if 0 <= j < _nbeams and np.isfinite(_ranges[j]):
                if _ranges[j] < r_here - 0.30:
                    return True
        return False

    # Bearing noise sigma_theta = dtheta / sqrt(12) from the angular step.
    dtheta = float(scan.angle_increment) if scan.angle_increment else 0.0
    if dtheta <= 0.0:
        dtheta = np.radians(1.0)                 # safe fallback
    sigma_theta = dtheta / np.sqrt(12.0)

    # Stage 1a: adaptive segmentation
    segments = segment_scan(
        points,
        distance_threshold=distance_threshold,
        angle_increment=dtheta,
        sigma_r=RANGE_STD,
        k=SEG_NOISE_K,
    )

    # Stage 1b(i): classify each cluster as a leg candidate by width / circularity.
    single_leg_width = 2.0 * leg_radius
    width_max = max(1.8 * single_leg_width, max_leg_width)   # legs-together allowance
    # Absolute floor: reject clusters far thinner than any leg (wires, thin poles).
    width_min = 0.03
    candidates = []  # each: {'p': centroid(2,), 'n': int, 'R': (2,2)}
    for seg in segments:
        n = seg.shape[0]
        if n < MIN_CLUSTER_POINTS:               # drop noise clusters
            continue
        width, circularity = _cluster_shape(seg)
        if width > width_max:                    # too wide -> wall / large object
            continue
        if n >= 4 and width < width_min:         # too thin -> pole/wire, not a leg
            continue
        # Wall filter: reject only wall-scale clusters (>0.45 m) that are clearly
        # linear and well-sampled; a leg's thin front arc must not be dropped here.
        if n >= 5 and width > max(0.45, single_leg_width) and circularity < CIRCULARITY_MIN:
            continue
        centroid = seg.mean(axis=0)
        R = _cluster_covariance(centroid, n, sigma_theta)
        occl = bool(_occlusion_cut(seg[0]) or _occlusion_cut(seg[-1]))
        candidates.append({'p': centroid, 'n': n, 'R': R, 'occl': occl})

    if not candidates:
        return []

    # Stage 1b(ii): assemble persons. A paired candidate (within
    # [PAIR_MIN_DIST, max_leg_width]) gives a person at the midpoint with
    # R = 1/4 (R_A + R_B); an unpaired candidate is a person on its own.
    measurements: List[LegMeasurement] = []
    used = [False] * len(candidates)

    for a in range(len(candidates)):
        if used[a]:
            continue
        best, best_d = -1, None
        for b in range(a + 1, len(candidates)):
            if used[b]:
                continue
            d = float(np.linalg.norm(candidates[a]['p'] - candidates[b]['p']))
            if PAIR_MIN_DIST <= d <= max_leg_width and (best_d is None or d < best_d):
                best, best_d = b, d
        if best >= 0:
            used[a] = used[best] = True
            mid = 0.5 * (candidates[a]['p'] + candidates[best]['p'])
            R = 0.25 * (candidates[a]['R'] + candidates[best]['R'])
            mm = _to_measurement(mid, R)
            mm.occluded = candidates[a]['occl'] and candidates[best]['occl']
            measurements.append(mm)
        else:
            used[a] = True
            mm = _to_measurement(candidates[a]['p'], candidates[a]['R'])
            mm.occluded = candidates[a]['occl']
            measurements.append(mm)

    return measurements
