"""
leg_detection.py — Stage 1 of the people-avoidance pipeline.

Input : sensor_msgs/LaserScan
Output: List[LegMeasurement]

Students implement:
  - segment_scan()   : split the point cloud into contiguous clusters
  - detect_legs()    : identify leg-pair candidates and assign covariance R
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np
from sensor_msgs.msg import LaserScan


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
# TODO Stage 1a — segmentation
# ---------------------------------------------------------------------------

def segment_scan(
    points: np.ndarray,
    distance_threshold: float = 0.1,
) -> List[np.ndarray]:
    """
    Split a sorted point cloud into contiguous clusters.

    Two consecutive points belong to the same segment when their Euclidean
    distance is below *distance_threshold* (metres).

    Args:
        points:             (N, 2) array of Cartesian scan points, in scan order.
        distance_threshold: Maximum gap between consecutive points in a segment (m).

    Returns:
        List of (K_i, 2) arrays, one per segment.  Empty list if points is empty.

    TODO(student): implement FH-style segmentation.
        Pseudocode:
            current_segment = [points[0]]
            for i in range(1, len(points)):
                if dist(points[i], points[i-1]) > distance_threshold:
                    yield current_segment
                    current_segment = []
                current_segment.append(points[i])
            yield current_segment
    """
    # TODO(student): replace this stub with the real implementation
    return []


# ---------------------------------------------------------------------------
# TODO Stage 1b — leg detection and covariance assignment
# ---------------------------------------------------------------------------

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

    segments = segment_scan(points, distance_threshold=distance_threshold)

    # TODO(student): identify leg-like segments from `segments`
    #   Hint: a segment is leg-like if its width (max distance between any two
    #         points) is approximately 2 * leg_radius.

    # TODO(student): pair leg segments into person detections
    #   Hint: compute the distance between every pair of leg-candidate centroids;
    #         keep pairs whose distance ≤ max_leg_width.

    # TODO(student): for each paired detection, create a LegMeasurement:
    #   position = midpoint of the two leg centroids
    #   Rxx, Ryy = range_to_midpoint ** 2 * angle_variance  (tune the constant)
    #   Rxy = 0.0  (or derive from bearing noise if you want full R)

    return []
