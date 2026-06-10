"""
leg_detection.py — Stage 1 of the people-avoidance pipeline.

Input : sensor_msgs/LaserScan
Output: List[LegMeasurement]

Students implement:
  - segment_scan()   : split the point cloud into contiguous clusters
  - detect_legs()    : identify leg-pair candidates and assign covariance R

Implementation follows "People Detection and Tracking from 2D LiDAR"
(T. Kucner, Aalto/UBISS 2026):
  - Module 1 — polar->Cartesian Jacobian covariance  (the R matrix)
  - Module 2 — adaptive jump-distance segmentation    tau(r) = r*dtheta + k*sigma_r
  - Module 3 — geometric pattern matching (width / circularity) + leg pairing
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# Sensor / model constants  (Kucner, UBISS 2026 — running example)
# ---------------------------------------------------------------------------
RANGE_STD          = 0.02    # sigma_r, range noise std (m)        [Module 1, p.12]
SEG_NOISE_K        = 3.0     # k in tau(r) = r*dtheta + k*sigma_r  [Eq.(4), p.60]
MIN_CLUSTER_POINTS = 2       # n_min, drop single-point clusters   [p.61]
CIRCULARITY_MIN    = 0.15    # reject clearly wall-like clusters (wall ratio <~0.1) [p.123]
PAIR_MIN_DIST      = 0.05    # min centre-to-centre to call two clusters a leg pair (m)
COV_EPS            = 1e-9    # tiny diagonal floor so R stays positive-definite for the KF


@dataclass
class LegMeasurement:
    """
    One detected person expressed as a 2-D position with observation covariance.

    Coordinate frame: the laser frame (x forward, y left).

    The symmetric 2x2 measurement covariance matrix R is stored as three
    independent entries so the Kalman filter can use them directly:

        R = [[Rxx, Rxy],
             [Rxy, Ryy]]

    Fields
    ------
    x, y  : Person position in the laser frame (metres).
    Rxx   : Variance along x (m²).
    Rxy   : Cross-covariance term (m²).
    Ryy   : Variance along y (m²).
    """
    x:   float
    y:   float
    Rxx: float
    Rxy: float
    Ryy: float


# ---------------------------------------------------------------------------
# Utility (provided — students do not need to modify this)
# ---------------------------------------------------------------------------

def scan_to_cartesian(scan: LaserScan) -> np.ndarray:
    """
    Convert LaserScan polar readings to (x, y) Cartesian points in the laser frame.

    Invalid ranges (inf, nan, out-of-bounds) are dropped.

    Returns:
        Array of shape (N, 2).  May be empty (shape (0, 2)) if no valid readings.
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
    return np.column_stack((r * np.cos(a), r * np.sin(a)))  # (N, 2)


# ---------------------------------------------------------------------------
# Stage 1a — segmentation
# ---------------------------------------------------------------------------

def segment_scan(
    points: np.ndarray,
    distance_threshold: float = 0.1,
    angle_increment: float | None = None,
    sigma_r: float = RANGE_STD,
    k: float = SEG_NOISE_K,
) -> List[np.ndarray]:
    """
    Split a sorted point cloud into contiguous clusters.

    Two consecutive points belong to the same segment when their Euclidean
    distance is below *distance_threshold* (metres).

    Args:
        points:             (N, 2) array of Cartesian scan points, in scan order.
        distance_threshold: Maximum gap between consecutive points in a segment (m)
                            — used when *angle_increment* is None (fixed-threshold mode).
        angle_increment:    Angular step Δθ between beams (rad).  When provided, the
                            gap test uses the adaptive jump-distance threshold from
                            the lecture, Eq. (4):
                                tau(r) = r * Δθ + k * sigma_r
                            evaluated at the previous beam's range r = ||p_{i-1}||.
                            This grows with range (point spacing ~ r·Δθ) and falls
                            back to the noise floor k·sigma_r up close.
        sigma_r:            Range noise std used in the adaptive threshold (m).
        k:                  Noise multiplier in the adaptive threshold (default 3).

    Returns:
        List of (K_i, 2) arrays, one per segment.  Empty list if points is empty.

    FH-style segmentation (single O(N) pass):
            current_segment = [points[0]]
            for i in range(1, len(points)):
                if dist(points[i], points[i-1]) > threshold:
                    yield current_segment
                    current_segment = []
                current_segment.append(points[i])
            yield current_segment
    """
    n = len(points)
    if n == 0:
        return []

    segments: List[np.ndarray] = []
    current = [points[0]]

    for i in range(1, n):
        gap = float(np.linalg.norm(points[i] - points[i - 1]))

        if angle_increment is not None:
            # Adaptive jump-distance threshold — lecture Eq. (4): tau(r)=r*dtheta+k*sigma_r,
            # floored by distance_threshold so that parameter still has a tuning effect
            # (raise it to merge nearby clusters; lower it to let the adaptive term rule).
            r_prev = float(np.linalg.norm(points[i - 1]))   # range = distance from sensor (origin)
            threshold = max(distance_threshold, r_prev * float(angle_increment) + k * sigma_r)
        else:
            threshold = distance_threshold

        if gap > threshold:
            segments.append(np.asarray(current))   # close off this segment
            current = []                            # start a new one

        current.append(points[i])

    segments.append(np.asarray(current))            # don't forget the last segment
    return segments


# ---------------------------------------------------------------------------
# Stage 1b — leg detection and covariance assignment
# ---------------------------------------------------------------------------

def _cluster_shape(seg: np.ndarray) -> Tuple[float, float]:
    """
    Geometric features of a cluster via PCA  (lecture Module 3, pp. 122-123).

    Returns:
        width        : extent along the principal axis (m).
        circularity  : lambda_min / lambda_max  in [0, 1].
                       ~1 for a round blob (leg / pole), ~0 for an elongated run (wall).
    """
    npts = seg.shape[0]
    if npts < 2:
        return 0.0, 0.0
    centred = seg - seg.mean(axis=0)
    cov = (centred.T @ centred) / npts          # 2x2 point-distribution covariance
    evals, evecs = np.linalg.eigh(cov)          # ascending: evals[0] <= evals[1]
    lam_min, lam_max = float(evals[0]), float(evals[1])
    e1 = evecs[:, 1]                            # principal axis (largest eigenvalue)
    proj = centred @ e1
    width = float(proj.max() - proj.min())
    circularity = (lam_min / lam_max) if lam_max > 1e-12 else 0.0
    return width, circularity


def _cluster_covariance(
    centroid: np.ndarray, n: int, sigma_theta: float
) -> np.ndarray:
    """
    Centroid covariance Sigma_k ~= Sigma_xy / n   (lecture Eq. 2 + Eq. 5).

    Sigma_xy = J * diag(sigma_r^2, sigma_theta^2) * J^T, evaluated at the cluster
    centroid (r, theta):
        Rxx = sr2*cos^2 + r2t2*sin^2
        Rxy = (sr2 - r2t2)*sin*cos
        Ryy = sr2*sin^2 + r2t2*cos^2
    with sr2 = sigma_r^2 and r2t2 = (r*sigma_theta)^2.  Dividing by n is the
    covariance-of-the-mean: more points -> tighter estimate.
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
    return LegMeasurement(
        x=float(p[0]), y=float(p[1]),
        Rxx=float(R[0, 0]), Rxy=float(R[0, 1]), Ryy=float(R[1, 1]),
    )


def detect_legs(
    scan: LaserScan,
    distance_threshold: float = 0.1,
    leg_radius: float = 0.10,
    max_leg_width: float = 0.25,
) -> List[LegMeasurement]:
    """
    Detect people from a LaserScan using geometric leg-pair matching (FH style).

    Pipeline
    --------
    1. Convert scan to Cartesian via scan_to_cartesian()  [provided].
    2. Segment the point cloud into contiguous clusters via segment_scan()  [TODO].
    3. For each segment whose width matches a single leg (≈ leg_radius), mark
       it as a leg candidate.
    4. Pair leg candidates whose midpoint separation is ≤ max_leg_width; the
       person position is the midpoint of the two leg centres.
    5. Assign observation covariance R for each detected person:
       a simple range-dependent isotropic model works as a starting point —
       Rxx = Ryy = σ², Rxy = 0, where σ grows with distance from the sensor.

    Args:
        scan:               Incoming LaserScan message.
        distance_threshold: Segmentation gap threshold (m).
        leg_radius:         Expected radius of a single leg segment (m).
        max_leg_width:      Maximum distance between the centres of two paired
                            leg segments (m).

    Returns:
        List of LegMeasurement.  Returns [] until students implement the body.
    """
    points = scan_to_cartesian(scan)
    if points.shape[0] == 0:
        return []

    # Angular step (rad) and bearing noise sigma_theta = dtheta / sqrt(12)  [Module 1, p.30].
    dtheta = float(scan.angle_increment) if scan.angle_increment else 0.0
    if dtheta <= 0.0:
        dtheta = np.radians(1.0)                 # safe fallback (running example uses 1 deg)
    sigma_theta = dtheta / np.sqrt(12.0)

    # ── Stage 1a: adaptive segmentation (lecture Eq. 4) ──────────────────────
    segments = segment_scan(
        points,
        distance_threshold=distance_threshold,
        angle_increment=dtheta,
        sigma_r=RANGE_STD,
        k=SEG_NOISE_K,
    )

    # ── Stage 1b(i): classify each cluster as a leg candidate ────────────────
    #   width gate  — single leg ≈ 2*leg_radius; allow legs-together (Pattern B).
    #   circularity — reject clearly elongated runs (walls).
    single_leg_width = 2.0 * leg_radius
    width_max = max(1.8 * single_leg_width, max_leg_width)   # legs-together allowance
    candidates = []  # each: {'p': centroid(2,), 'n': int, 'R': (2,2)}
    for seg in segments:
        n = seg.shape[0]
        if n < MIN_CLUSTER_POINTS:               # n_min filter — drop noise clusters
            continue
        width, circularity = _cluster_shape(seg)
        if width > width_max:                    # too wide -> wall / large object
            continue
        # Secondary wall filter: only drop a cluster that is BOTH wider than a single
        # leg AND clearly linear AND well-sampled (a wall fragment).  A narrow,
        # leg-sized cluster is never rejected on circularity — a real leg's front arc
        # can look thin (low circularity) yet still be a leg.
        if n >= 5 and width > single_leg_width and circularity < CIRCULARITY_MIN:
            continue
        centroid = seg.mean(axis=0)
        R = _cluster_covariance(centroid, n, sigma_theta)
        candidates.append({'p': centroid, 'n': n, 'R': R})

    if not candidates:
        return []

    # ── Stage 1b(ii): assemble persons from leg candidates ───────────────────
    #   Pattern A — two candidates within [PAIR_MIN_DIST, max_leg_width] -> one person
    #               at the midpoint, R = 1/4 (R_A + R_B)              [lecture p.142].
    #   Pattern B/C — an unpaired candidate is a person on its own
    #               (legs together / single visible leg).
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
            measurements.append(_to_measurement(mid, R))
        else:
            used[a] = True
            measurements.append(_to_measurement(candidates[a]['p'], candidates[a]['R']))

    return measurements
