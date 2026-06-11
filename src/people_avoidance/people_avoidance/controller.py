"""
controller.py — Stage 3 of the people-avoidance pipeline.

Input : List[Track], robot pose (x, y, theta)
Output: geometry_msgs/Twist  published on /cmd_vel

Students implement:
  - obstacle_radius()    : derive a safety radius from track covariance
  - compute_velocity()   : avoidance policy → linear + angular velocity command

Approach — Control-Barrier-Function (CBF) safety filter
-------------------------------------------------------
Built directly from the UBISS 2026 control material:
  - Deka, "Nonlinear controls": CBF safety filter
        min ||u - u_nom||²   s.t.   ḣ(x,u) + γ·h(x) ≥ 0
  - exp_cbf_public.ipynb (Dubins car = differential drive): the LOOKAHEAD
    barrier that fixes the unicycle relative-degree problem.

Frame.  The node hands us tracks expressed in the **laser/robot frame**
(robot at the origin, heading +x).  So the robot's Dubins state is simply
(0, 0, 0): the lookahead point is P = (L, 0), and a track's stored velocity
is its velocity *relative to the robot* — exactly the quantity a
moving-obstacle CBF needs.

Per person i (centre p_i, relative velocity v_i, radius r_i) the lookahead
barrier and its time-derivative are

    E   = P - p_i = (L - p_ix, -p_iy)
    h_i = ‖E‖² - (r_i + L)²
    ḣ_i = 2v(s_i + L) + 2Lω·q_i  - 2 E·v_i          (last term = motion of p_i)
          with s_i = -p_ix,  q_i = -p_iy   (robot at origin, θ = 0)

The CBF condition ḣ_i + γ h_i ≥ 0 is linear in u = (v, ω):

    -2(s_i + L)·v  - 2L q_i·ω   ≤   γ h_i - 2 E·v_i      (one QP row per person)

The nominal command u_nom is a pure-pursuit go-to-goal controller; the QP
projects it onto the intersection of all the per-person safe half-planes
and the actuator box [0, v_max] × [−ω_max, ω_max].  With one row per person
the filter avoids any number of people simultaneously.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Twist

from .tracking import Track


# ---------------------------------------------------------------------------
# Controller configuration (sensible defaults; overridable via set_params)
# ---------------------------------------------------------------------------
_CFG = {
    'lookahead': 0.30,        # L — virtual probe distance ahead of the robot (m)
    'gamma': 2.0,             # CBF class-K gain γ (small=cautious, large=hugs edge)
    'w_omega': 0.1,           # QP weight on ω  (<1 ⇒ "steer before brake")
    'person_radius': 0.30,    # physical half-width of a person (m)
    'robot_radius': 0.18,     # TurtleBot4 footprint radius (m)
    'goal_tolerance': 0.20,   # stop when within this distance of the goal (m)
    'k_omega': 2.0,           # pure-pursuit heading gain
    'use_prediction': True,   # advance each person by v·t_pred before gating
    't_pred': 0.3,            # prediction lookahead for the barrier (s)
    'influence': 3.0,         # ignore people farther than this (m); None = all
    'escape_v_frac': 0.3,     # "blocked" if filtered v < this × nominal v
    'escape_dist': 1.5,       # only escape-turn when a person is within this (m)
    'escape_omega_frac': 0.6, # escape turn rate as a fraction of ω_max
}

# Navigation goal in the ODOM frame.  None → auto-initialise to a point
# `_AUTO_GOAL_AHEAD` metres straight ahead of the robot on the first call,
# so the node "drives from A to B" out of the box; set_goal() overrides it.
_goal_odom: Optional[Tuple[float, float]] = None
_auto_goal_done: bool = False
_AUTO_GOAL_AHEAD = 3.0


def set_goal(x: float, y: float) -> None:
    """Set the navigation goal (odom frame).  Demo/launch hook."""
    global _goal_odom, _auto_goal_done
    _goal_odom = (float(x), float(y))
    _auto_goal_done = True


def clear_goal() -> None:
    """Forget the goal → the controller drives straight forward."""
    global _goal_odom, _auto_goal_done
    _goal_odom = None
    _auto_goal_done = True


def set_params(**kw) -> None:
    """Override controller config entries (e.g. gamma, lookahead)."""
    _CFG.update({k: v for k, v in kw.items() if k in _CFG})


def _angle_wrap(a: float) -> float:
    """Wrap an angle to (-π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Stage 3a — uncertainty-aware obstacle radius
# ---------------------------------------------------------------------------

def obstacle_radius(track: Track, sigma_scale: float) -> float:
    """
    Derive a conservative obstacle radius from the track's positional uncertainty.

    The radius grows when the Kalman filter is uncertain (large P) and shrinks
    as the estimate converges, giving an implicit safety margin that inflates
    when we are unsure where the person is.

        radius = sigma_scale × √(λ_max(P[:2, :2]))

    Args:
        track:        An active Track with state covariance P (4×4).
        sigma_scale:  Scaling factor k on the positional standard deviation.

    Returns:
        Obstacle radius in metres (always ≥ 0).
    """
    pos_cov = np.asarray(track.P)[:2, :2]
    lambda_max = float(np.linalg.eigvalsh(pos_cov)[-1])
    return float(sigma_scale) * math.sqrt(max(lambda_max, 0.0))


# ---------------------------------------------------------------------------
# Tiny exact 2-variable QP  (no external solver dependency)
# ---------------------------------------------------------------------------

def _solve_qp_2d(
    v_nom: float, w_nom: float, w_omega: float,
    rows: List[Tuple[float, float, float]],
    v_min: float, v_max: float, w_max: float,
) -> Tuple[float, float]:
    """
    Solve   min (v-v_nom)² + w_omega·(ω-w_nom)²
            s.t.  a·v + b·ω ≤ c          for every (a, b, c) in `rows`
                  v_min ≤ v ≤ v_max,  -w_max ≤ ω ≤ w_max

    Exact 2-D method: whiten the cost to an isotropic Euclidean projection
    (ω' = √w_omega·ω), then take the nearest feasible point among the
    interior optimum, the projection onto each single constraint line, and
    the intersection of every constraint pair — the classic 2-D QP vertex
    enumeration.  Constraint count is tiny (≤ ~people+4), so this is fast and
    has no solver dependency (cvxopt is unavailable offline on the VM).
    """
    sw = math.sqrt(max(w_omega, 1e-9))      # guard ω-weight 0 (div-by-zero)
    # Box constraints as rows in (v, ω):  v≤v_max, -v≤-v_min, ω≤w_max, -ω≤w_max
    all_rows = list(rows) + [
        (1.0, 0.0, v_max), (-1.0, 0.0, -v_min),
        (0.0, 1.0, w_max), (0.0, -1.0, w_max),
    ]
    # Whitened constraints a·v + (b/sw)·ω' ≤ c
    W = [(a, b / sw, c) for (a, b, c) in all_rows]
    target = np.array([v_nom, sw * w_nom])

    def feasible(p, eps=1e-7):
        return all(a * p[0] + b * p[1] <= c + eps for (a, b, c) in W)

    candidates = [target]                                   # interior optimum
    for (a, b, c) in W:                                     # edge projections
        n2 = a * a + b * b
        if n2 < 1e-12:
            continue
        t = (c - (a * target[0] + b * target[1])) / n2
        candidates.append(target + min(t, 0.0) * np.array([a, b]))
    for i in range(len(W)):                                 # vertex intersections
        a1, b1, c1 = W[i]
        for j in range(i + 1, len(W)):
            a2, b2, c2 = W[j]
            det = a1 * b2 - a2 * b1
            if abs(det) < 1e-9:
                continue
            vx = (c1 * b2 - c2 * b1) / det
            vy = (a1 * c2 - a2 * c1) / det
            candidates.append(np.array([vx, vy]))

    best, best_cost = None, float('inf')
    for p in candidates:
        if not feasible(p):
            continue
        cost = (p[0] - target[0]) ** 2 + (p[1] - target[1]) ** 2
        if cost < best_cost:
            best, best_cost = p, cost
    if best is None:
        # No feasible point (over-constrained) — safest fallback is to stop.
        return 0.0, 0.0
    return float(best[0]), float(best[1]) / sw


# ---------------------------------------------------------------------------
# Stage 3b — avoidance policy (pure-pursuit nominal + CBF-QP safety filter)
# ---------------------------------------------------------------------------

def compute_velocity(
    tracks: List[Track],
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    max_linear_speed: float = 0.2,
    max_angular_speed: float = 1.0,
    obstacle_radius_scale: float = 2.0,
) -> Twist:
    """
    Compute a velocity command that drives toward the goal while avoiding all
    tracked people, via a CBF-QP safety filter.

    Pipeline
    --------
    1. nominal  : pure-pursuit toward the goal (odom goal rotated into the
                  robot frame); defaults to driving forward until a goal is
                  reached.  Stops inside `goal_tolerance` of the goal.
    2. safety   : one lookahead-CBF half-plane per person (with a
                  moving-obstacle term from the track velocity and a
                  covariance-inflated radius), solved as a small QP that
                  projects the nominal command onto the safe set.

    Args:
        tracks:                Active person tracks (robot/laser frame).
        robot_x, robot_y:      Robot position in the odom frame (m).
        robot_theta:           Robot heading in the odom frame (rad).
        max_linear_speed:      Forward speed cap (m/s).
        max_angular_speed:     Rotation rate cap (rad/s).
        obstacle_radius_scale: Uncertainty inflation k for obstacle_radius().

    Returns:
        geometry_msgs/Twist with linear.x and angular.z set.
    """
    global _goal_odom, _auto_goal_done
    cmd = Twist()
    L = _CFG['lookahead']

    # ── 1. Nominal go-to-goal (pure pursuit), computed in the robot frame ──
    if not _auto_goal_done:
        # First call: plant a goal a few metres straight ahead (odom frame).
        _goal_odom = (
            robot_x + _AUTO_GOAL_AHEAD * math.cos(robot_theta),
            robot_y + _AUTO_GOAL_AHEAD * math.sin(robot_theta),
        )
        _auto_goal_done = True

    if _goal_odom is not None:
        dx = _goal_odom[0] - robot_x
        dy = _goal_odom[1] - robot_y
        ct, st = math.cos(robot_theta), math.sin(robot_theta)
        gx =  ct * dx + st * dy        # goal in robot frame (x forward)
        gy = -st * dx + ct * dy
        dist_goal = math.hypot(gx, gy)
        if dist_goal < _CFG['goal_tolerance']:
            return cmd                 # arrived → stop (zero Twist)
        heading_err = math.atan2(gy, gx)
        w_nom = _CFG['k_omega'] * heading_err
        # Slow down for large heading errors and near the goal.
        v_nom = max_linear_speed * max(0.0, math.cos(heading_err))
        v_nom = min(v_nom, dist_goal)
    else:                              # no goal → just drive forward
        v_nom, w_nom = max_linear_speed, 0.0

    v_nom = float(np.clip(v_nom, 0.0, max_linear_speed))
    w_nom = float(np.clip(w_nom, -max_angular_speed, max_angular_speed))

    # ── 2. CBF safety constraints, one row per person ─────────────────────
    rows: List[Tuple[float, float, float]] = []
    gamma = _CFG['gamma']
    base_r = _CFG['person_radius'] + _CFG['robot_radius']
    influence = _CFG['influence']
    t_pred = _CFG['t_pred'] if _CFG['use_prediction'] else 0.0

    nearest_dist = float('inf')
    left_block = right_block = 0.0       # crowdedness on each side (1/dist weight)
    for tr in tracks:
        if not getattr(tr, 'confirmed', True):
            continue                                   # ignore unconfirmed ghosts
        px, py = float(tr.m[0]), float(tr.m[1])        # person position (robot frame)
        vx, vy = float(tr.m[2]), float(tr.m[3])        # relative velocity (robot frame)
        # Advance the person by its predicted motion (first-order).
        px += vx * t_pred
        py += vy * t_pred
        dist = math.hypot(px, py)
        if influence is not None and dist > influence:
            continue                                   # too far to matter now
        # Track who is ahead, and the left/right crowdedness for escape steering.
        if px > -0.2:
            nearest_dist = min(nearest_dist, dist)
            w_blk = 1.0 / max(dist, 0.2)
            if py >= 0.0:
                left_block += w_blk
            else:
                right_block += w_blk

        r_i = base_r + obstacle_radius(tr, obstacle_radius_scale)

        # Robot at origin, heading +x ⇒ s = -px, q = -py, P = (L, 0).
        s = -px
        q = -py
        Ex = L - px
        Ey = -py
        h_i = Ex * Ex + Ey * Ey - (r_i + L) ** 2
        # Moving-obstacle term: ḣ gains -2 E·v_person.
        E_dot_v = Ex * vx + Ey * vy
        # CBF row:  -2(s+L) v - 2L q ω ≤ γ h_i - 2 E·v
        a = -2.0 * (s + L)
        b = -2.0 * L * q
        c = gamma * h_i - 2.0 * E_dot_v
        rows.append((a, b, c))

    if not rows:                                       # nobody to avoid
        cmd.linear.x = v_nom
        cmd.angular.z = w_nom
        return cmd

    v, w = _solve_qp_2d(
        v_nom, w_nom, _CFG['w_omega'], rows,
        v_min=0.0, v_max=max_linear_speed, w_max=max_angular_speed,
    )

    # Deadlock escape — the CBF can brake a head-on approach to v≈0 without a
    # steering preference (a person exactly ahead has q=0, so ω does not change
    # that person's barrier this step).  When blocked and still en route, turn
    # in place toward the more OPEN side so the geometry breaks and the filter
    # can then let us pass.
    blocked = (v < _CFG['escape_v_frac'] * max(v_nom, 1e-3)
               and nearest_dist < _CFG['escape_dist'])
    if blocked:
        # Turn toward the side with less crowding (default left if symmetric),
        # and STOP forward motion (v=0).  Turning in place keeps a disk robot
        # safe regardless of the CBF rows — whereas keeping the QP's forward v
        # with an overridden ω could violate a coupled CBF constraint.  This
        # preserves the safety guarantee while breaking the symmetric deadlock.
        turn_dir = 1.0 if left_block <= right_block else -1.0
        w = turn_dir * _CFG['escape_omega_frac'] * max_angular_speed
        v = 0.0

    cmd.linear.x = float(np.clip(v, 0.0, max_linear_speed))
    cmd.angular.z = float(np.clip(w, -max_angular_speed, max_angular_speed))
    return cmd
