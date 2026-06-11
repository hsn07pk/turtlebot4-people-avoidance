"""
controller.py — Stage 3 of the people-avoidance pipeline.

Input : List[Track], robot pose (x, y, theta), optional scan obstacle points
Output: geometry_msgs/Twist  published on /cmd_vel

Students implement:
  - obstacle_radius()    : derive a safety radius from track covariance
  - compute_velocity()   : avoidance policy → linear + angular velocity command

Approach — potential-field navigation + Control-Barrier-Function safety filter
------------------------------------------------------------------------------
Two layers, both from the UBISS 2026 control material:

  1. NOMINAL (smooth routing): a potential field steers the robot toward the
     goal while CURVING around obstacles — goal attraction + a vortex
     repulsion from every nearby obstacle (people AND walls).  The vortex
     (tangential) term routes the robot *around* an obstacle instead of
     stopping in front of it, and avoids the classic head-on local minimum.

  2. SAFETY (hard guarantee): the lookahead Control-Barrier-Function QP
     (Deka "Nonlinear controls"; exp_cbf_public.ipynb) projects the nominal
     command onto the safe set — one half-plane per obstacle:

         min ‖u − u_nom‖²   s.t.   ḣ_i(x,u) + γ h_i(x) ≥ 0   ∀ obstacle i
                                   v ∈ [0, v_max], |ω| ≤ ω_max

     with the Dubins lookahead barrier  h_i = ‖P − p_i‖² − (r_i + L)²,
     P = robot + L·heading.  People add a moving-obstacle term −2E·v_i.

Frame.  Tracks and scan points arrive in the **laser/robot frame** (robot at
the origin, heading +x), so the robot's Dubins state is (0, 0, 0) and a
track's stored velocity is already relative to the robot.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
from geometry_msgs.msg import Twist

from .tracking import Track


# ---------------------------------------------------------------------------
# Controller configuration (defaults; overridable via set_params)
# ---------------------------------------------------------------------------
_CFG = {
    'lookahead': 0.30,        # L — virtual probe distance ahead of the robot (m)
    'gamma': 2.0,             # CBF class-K gain γ
    'w_omega': 0.1,           # QP weight on ω  (<1 ⇒ "steer before brake")
    'person_radius': 0.30,    # physical half-width of a person (m)
    'wall_radius': 0.08,      # half-width assigned to a scan obstacle point (m)
    'robot_radius': 0.18,     # TurtleBot4 footprint radius (m)
    'goal_tolerance': 0.20,   # stop when within this distance of the goal (m)
    'k_omega': 1.5,           # heading-error gain for ω_nom
    'k_att': 1.0,             # goal attraction weight (potential field)
    'k_rep': 0.6,             # obstacle repulsion weight (potential field)
    'vortex': 1.6,            # tangential/radial ratio (route-around strength)
    'use_prediction': True,   # advance each person by v·t_pred before gating
    't_pred': 0.3,            # prediction lookahead for the barrier (s)
    'influence': 2.5,         # obstacles farther than this are ignored (m)
    'escape_v_frac': 0.25,    # "blocked" if filtered v < this × nominal v
    'escape_dist': 1.2,       # only escape-turn when an obstacle is within (m)
    'escape_omega_frac': 0.6, # escape turn rate as a fraction of ω_max
    'slow_radius': 1.0,       # start slowing this far outside an obstacle (m)
    # Follow-the-gap navigation (commits to an opening instead of oscillating)
    'gap_clear': 0.10,        # extra angular clearance added to each obstacle (m)
    'gap_hyst': 0.5,          # hysteresis: pull toward the last chosen heading
    'gap_min_deg': 14.0,      # ignore openings narrower than this (deg)
    # Back-off reflex: reverse out of the danger zone (e.g. a foot in front)
    'backoff_trigger': 0.55,  # reverse when nearest front obstacle is within (m)
    'backoff_clear': 0.85,    # stop reversing once it is beyond this (m, hysteresis)
    'backoff_speed': 0.10,    # reverse speed (m/s)
    'backoff_rear_min': 0.50, # only reverse if the space behind is clearer than (m)
                              # (≈ robot radius 0.18 + a reaction margin)
}

# Heading the robot last committed to (radians, robot frame).  The hysteresis
# in _gap_heading pulls toward this so the robot does not flip-flop left/right
# in front of an obstacle — it commits to going around one side.
_last_heading: float = 0.0

# True while the back-off reflex is reversing out of the danger zone.
_backing: bool = False

# Navigation goal in the ODOM frame.  None → drive straight forward.
_goal_odom: Optional[Tuple[float, float]] = None
_manual_cmd: Optional[Tuple[float, float]] = None   # (v, ω) manual override target


def set_goal(x: float, y: float) -> None:
    """Set the navigation goal (odom frame)."""
    global _goal_odom, _manual_cmd, _last_heading, _backing
    _goal_odom = (float(x), float(y))
    _manual_cmd = None
    _last_heading = 0.0          # forget the old commitment for a fresh goal
    _backing = False


def clear_goal() -> None:
    """Forget the goal → drive straight forward (still avoiding obstacles)."""
    global _goal_odom
    _goal_odom = None


def set_manual(v: float, omega: float) -> None:
    """Manual-drive intent (robot frame): the nominal becomes this (v, ω),
    still filtered by the CBF so manual driving cannot hit anything."""
    global _manual_cmd, _goal_odom
    _manual_cmd = (float(v), float(omega))
    _goal_odom = None


def clear_manual() -> None:
    global _manual_cmd
    _manual_cmd = None


def set_params(**kw) -> None:
    """Override controller config entries (e.g. gamma, lookahead)."""
    _CFG.update({k: v for k, v in kw.items() if k in _CFG})


def get_params() -> dict:
    return dict(_CFG)


def _angle_wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _gap_heading(obs: List[dict], goal_ang: float) -> Optional[float]:
    """
    Follow-the-Gap: choose the best free heading around the obstacles.

    Each obstacle blocks an angular wedge (its safety disk seen from the
    robot).  Among the free openings wide enough for the robot, pick the
    heading that best trades off proximity to the goal direction against a
    HYSTERESIS pull toward the last committed heading — so the robot commits
    to going around one side instead of oscillating, and will turn toward an
    opening that is far to the side or behind it.

    Returns the chosen heading (rad, robot frame), or None if fully boxed in.
    """
    global _last_heading
    N = 90
    binw = 2.0 * math.pi / N
    blocked = [False] * N
    for o in obs:
        d = math.hypot(o['px'], o['py'])
        if d < 1e-3:
            continue
        clr = o['r'] + _CFG['gap_clear']
        ratio = min(0.999, clr / max(d, clr))
        half = math.asin(ratio)                      # angular half-width blocked
        cb = int((math.atan2(o['py'], o['px']) + math.pi) / binw) % N
        steps = int(half / binw) + 1
        for k in range(-steps, steps + 1):
            blocked[(cb + k) % N] = True

    if all(blocked):
        return None

    min_bins = max(1, int(math.radians(_CFG['gap_min_deg']) / binw))
    best, best_cost = None, float('inf')
    # scan contiguous free runs (doubled array handles wrap-around)
    b2 = blocked + blocked
    j = 0
    while j < 2 * N:
        if b2[j]:
            j += 1
            continue
        k = j
        while k < 2 * N and not b2[k]:
            k += 1
        if (k - j) >= min_bins and j < N:
            for bi in range(j, k):
                a = -math.pi + (bi % N + 0.5) * binw
                cost = (abs(_angle_wrap(a - goal_ang))
                        + _CFG['gap_hyst'] * abs(_angle_wrap(a - _last_heading)))
                if cost < best_cost:
                    best_cost, best = cost, a
        j = k

    if best is None:                                 # only narrow slivers free
        for i in range(N):
            if blocked[i]:
                continue
            a = -math.pi + (i + 0.5) * binw
            cost = (abs(_angle_wrap(a - goal_ang))
                    + _CFG['gap_hyst'] * abs(_angle_wrap(a - _last_heading)))
            if cost < best_cost:
                best_cost, best = cost, a

    if best is not None:
        _last_heading = best                         # commit (hysteresis)
    return best


# ---------------------------------------------------------------------------
# Stage 3a — uncertainty-aware obstacle radius
# ---------------------------------------------------------------------------

def obstacle_radius(track: Track, sigma_scale: float) -> float:
    """
    Conservative obstacle radius from the track's positional uncertainty:

        radius = sigma_scale × √(λ_max(P[:2, :2]))

    Grows while the Kalman filter is uncertain, shrinks as it converges.
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
            s.t.  a·v + b·ω ≤ c   for every (a,b,c) in rows; box on v, ω.

    Exact 2-D method: whiten the cost (ω'=√w·ω) to a Euclidean projection,
    then take the nearest feasible point among the interior optimum, each
    single-constraint projection, and every constraint-pair intersection.
    """
    sw = math.sqrt(max(w_omega, 1e-9))
    all_rows = list(rows) + [
        (1.0, 0.0, v_max), (-1.0, 0.0, -v_min),
        (0.0, 1.0, w_max), (0.0, -1.0, w_max),
    ]
    W = [(a, b / sw, c) for (a, b, c) in all_rows]
    target = np.array([v_nom, sw * w_nom])

    def feasible(p, eps=1e-7):
        return all(a * p[0] + b * p[1] <= c + eps for (a, b, c) in W)

    candidates = [target]
    for (a, b, c) in W:
        n2 = a * a + b * b
        if n2 < 1e-12:
            continue
        t = (c - (a * target[0] + b * target[1])) / n2
        candidates.append(target + min(t, 0.0) * np.array([a, b]))
    for i in range(len(W)):
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
        return 0.0, 0.0
    return float(best[0]), float(best[1]) / sw


# ---------------------------------------------------------------------------
# Obstacle assembly: people (dynamic) + scan points (static)
# ---------------------------------------------------------------------------

def _gather_obstacles(
    tracks: List[Track],
    obstacle_points: Optional[Sequence[Tuple[float, float]]],
    obstacle_radius_scale: float,
) -> List[dict]:
    """Unified obstacle list in the robot frame.  Each: px,py,vx,vy,r,person."""
    t_pred = _CFG['t_pred'] if _CFG['use_prediction'] else 0.0
    base_person = _CFG['person_radius'] + _CFG['robot_radius']
    base_wall = _CFG['wall_radius'] + _CFG['robot_radius']
    influence = _CFG['influence']
    obs: List[dict] = []

    for tr in tracks:
        if not getattr(tr, 'confirmed', True):
            continue
        px, py = float(tr.m[0]), float(tr.m[1])
        vx, vy = float(tr.m[2]), float(tr.m[3])
        px += vx * t_pred
        py += vy * t_pred
        if math.hypot(px, py) > influence:
            continue
        r = base_person + obstacle_radius(tr, obstacle_radius_scale)
        obs.append({'px': px, 'py': py, 'vx': vx, 'vy': vy, 'r': r, 'person': True})

    if obstacle_points:
        for (ox, oy) in obstacle_points:
            ox = float(ox); oy = float(oy)
            if math.hypot(ox, oy) > influence:
                continue
            obs.append({'px': ox, 'py': oy, 'vx': 0.0, 'vy': 0.0,
                        'r': base_wall, 'person': False})
    return obs


# ---------------------------------------------------------------------------
# Stage 3b — avoidance policy
# ---------------------------------------------------------------------------

def compute_velocity(
    tracks: List[Track],
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    max_linear_speed: float = 0.2,
    max_angular_speed: float = 1.0,
    obstacle_radius_scale: float = 2.0,
    obstacle_points: Optional[Sequence[Tuple[float, float]]] = None,
) -> Twist:
    """
    Velocity command: navigate toward the goal while avoiding people AND
    static obstacles (walls/furniture from the scan), routing AROUND them.

    See module docstring.  `obstacle_points` is an optional list of (x, y)
    static obstacle points in the robot frame (downsampled scan returns);
    when omitted the controller avoids only the tracked people (the node's
    default behaviour).
    """
    cmd = Twist()
    L = _CFG['lookahead']
    obs = _gather_obstacles(tracks, obstacle_points, obstacle_radius_scale)

    # ── 0. Back-off reflex ────────────────────────────────────────────────
    # If an obstacle is in the danger zone right ahead (a foot stepping in
    # front) and there is room behind, REVERSE out until clear — then the
    # normal navigation below can find a way around.  Safe: we only back up
    # while the space behind stays clearer than backoff_rear_min.
    global _backing
    front = [math.hypot(o['px'], o['py']) for o in obs
             if abs(math.atan2(o['py'], o['px'])) < math.radians(75)]
    rear = [math.hypot(o['px'], o['py']) for o in obs
            if abs(math.atan2(o['py'], o['px'])) > math.radians(105)]
    near_front = min(front, default=float('inf'))
    near_rear = min(rear, default=float('inf'))
    rear_min = _CFG['backoff_rear_min']
    if _backing:
        if near_front > _CFG['backoff_clear'] or near_rear < rear_min:
            _backing = False
    elif near_front < _CFG['backoff_trigger'] and near_rear > rear_min:
        _backing = True
    if _backing:
        v = -min(_CFG['backoff_speed'], max_linear_speed)
        if near_rear < rear_min + 0.3:                 # ease off near rear limit
            v *= max(0.0, (near_rear - rear_min) / 0.3)
        cmd.linear.x = float(v)
        cmd.angular.z = 0.0                            # back straight; re-plan once clear
        return cmd

    # ── 1. NOMINAL command ────────────────────────────────────────────────
    if _manual_cmd is not None:
        # Manual drive: take the operator's (v, ω) as the nominal.
        v_nom = float(np.clip(_manual_cmd[0], 0.0, max_linear_speed))
        w_nom = float(np.clip(_manual_cmd[1], -max_angular_speed, max_angular_speed))
    else:
        # Goal in the robot frame (or straight ahead if no goal).
        if _goal_odom is not None:
            dx = _goal_odom[0] - robot_x
            dy = _goal_odom[1] - robot_y
            ct, st = math.cos(robot_theta), math.sin(robot_theta)
            gx =  ct * dx + st * dy
            gy = -st * dx + ct * dy
            dist_goal = math.hypot(gx, gy)
            if dist_goal < _CFG['goal_tolerance']:
                return cmd                         # arrived → stop
            gdir = (gx / dist_goal, gy / dist_goal)
        else:
            dist_goal = float('inf')
            gdir = (1.0, 0.0)                       # forward

        # Follow-the-Gap: steer toward the best free opening (commits to a
        # side instead of oscillating; finds openings far to the side/behind).
        goal_ang = math.atan2(gdir[1], gdir[0])
        gap = _gap_heading(obs, goal_ang)
        if gap is None:                            # fully boxed in → rotate out
            desired = _last_heading if _last_heading != 0.0 else math.pi / 2
        else:
            desired = gap
        heading_err = _angle_wrap(desired)
        w_nom = _CFG['k_omega'] * heading_err
        # forward speed: full when aimed at the opening, 0 while turning to it
        v_nom = max_linear_speed * max(0.0, math.cos(heading_err))
        nearest_edge = min((math.hypot(o['px'], o['py']) - o['r'] for o in obs),
                           default=float('inf'))
        if nearest_edge < _CFG['slow_radius']:
            v_nom *= max(0.0, nearest_edge / _CFG['slow_radius'])
        v_nom = min(v_nom, dist_goal)

    v_nom = float(np.clip(v_nom, 0.0, max_linear_speed))
    w_nom = float(np.clip(w_nom, -max_angular_speed, max_angular_speed))

    # ── 2. CBF safety constraints, one row per obstacle ───────────────────
    rows: List[Tuple[float, float, float]] = []
    gamma = _CFG['gamma']
    nearest_dist = float('inf')
    left_block = right_block = 0.0
    for o in obs:
        px, py, vx, vy, r_i = o['px'], o['py'], o['vx'], o['vy'], o['r']
        if px > -0.2:
            d = math.hypot(px, py)
            nearest_dist = min(nearest_dist, d)
            w_blk = 1.0 / max(d, 0.2)
            if py >= 0.0:
                left_block += w_blk
            else:
                right_block += w_blk
        s = -px
        q = -py
        Ex = L - px
        Ey = -py
        h_i = Ex * Ex + Ey * Ey - (r_i + L) ** 2
        E_dot_v = Ex * vx + Ey * vy
        a = -2.0 * (s + L)
        b = -2.0 * L * q
        c = gamma * h_i - 2.0 * E_dot_v
        rows.append((a, b, c))

    if not rows:
        cmd.linear.x = v_nom
        cmd.angular.z = w_nom
        return cmd

    v, w = _solve_qp_2d(
        v_nom, w_nom, _CFG['w_omega'], rows,
        v_min=0.0, v_max=max_linear_speed, w_max=max_angular_speed,
    )

    # ── 3. Deadlock escape (last resort) ──────────────────────────────────
    blocked = (v < _CFG['escape_v_frac'] * max(v_nom, 1e-3)
               and nearest_dist < _CFG['escape_dist'])
    if blocked:
        # Turn toward the COMMITTED gap heading (Follow-the-Gap), not the
        # instantaneous crowding — this keeps the robot rotating one way until
        # the opening is in front, instead of oscillating left/right.  Falls
        # back to the less-crowded side only if no heading is committed.
        if abs(_last_heading) > 1e-3:
            turn_dir = 1.0 if _last_heading > 0 else -1.0
        else:
            turn_dir = 1.0 if left_block <= right_block else -1.0
        w = turn_dir * _CFG['escape_omega_frac'] * max_angular_speed
        v = 0.0                                    # turn in place (always safe)

    cmd.linear.x = float(np.clip(v, 0.0, max_linear_speed))
    cmd.angular.z = float(np.clip(w, -max_angular_speed, max_angular_speed))
    return cmd
