"""
controller.py — Stage 3 of the people-avoidance pipeline: the avoidance
controller. A potential-field nominal command (goal attraction + vortex
repulsion routing around obstacles) is projected onto the safe set by a
lookahead CBF-QP safety filter.

Tracks and scan points arrive in the laser/robot frame (robot at the origin,
heading +x), so the robot's Dubins state is (0, 0, 0) and a track's velocity
is already relative to the robot.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
from geometry_msgs.msg import Twist

from .tracking import Track


# Controller configuration (defaults; overridable via set_params)
_CFG = {
    'lookahead': 0.15,        # L — probe distance ahead of robot (m); barrier inflates obstacles by (r+L)
    'gamma': 2.0,             # CBF class-K gain γ
    'w_omega': 0.1,           # QP weight on ω  (<1 ⇒ "steer before brake")
    'person_radius': 0.30,    # physical half-width of a person (m)
    'wall_radius': 0.03,      # margin added to each scan obstacle point (m)
    'robot_radius': 0.18,     # TurtleBot4 footprint radius (m)
    'goal_tolerance': 0.20,   # stop when within this distance of the goal (m)
    'k_omega': 1.5,           # heading-error gain for ω_nom
    'k_att': 1.0,             # goal attraction weight (potential field)
    'k_rep': 0.6,             # obstacle repulsion weight (potential field)
    'vortex': 1.6,            # tangential/radial ratio (route-around strength)
    'use_prediction': True,   # advance each person by v·t_pred before gating
    't_pred': 0.3,            # prediction lookahead for the barrier (s)
    'cors_cap': 0.12,         # cap on the covariance inflation term (m)
    'min_track_dist': 0.28,   # tracks closer than this are self-detections, not people (m)
    'slow_floor': 0.35,       # comfort slow-down never cuts below this fraction of speed
    'influence': 2.5,         # obstacles farther than this are ignored (m)
    'escape_v_frac': 0.25,    # "blocked" if filtered v < this × nominal v
    'escape_dist': 1.2,       # only escape-turn when an obstacle is within (m)
    'escape_omega_frac': 0.6, # escape turn rate as a fraction of ω_max
    'slow_radius': 1.0,       # start slowing this far outside an obstacle (m)
    'slew_v_rise': 0.05,      # max v increase per cycle (m/s); braking is never limited (safety)
    'slew_w': 0.45,           # max ω change per cycle (rad/s)
    'dock_dist': 0.8,         # within this of the goal, use precision docking: pure-pursuit + CBF only (m)
    # Follow-the-gap navigation
    'gap_clear': 0.04,        # extra angular clearance added to each obstacle (m)
    'gap_hyst': 0.5,          # hysteresis: pull toward the last chosen heading
    'gap_min_deg': 4.0,       # minimum gap width to consider (deg); ignores single-bin noise
    # Back-off reflex: reverse out of the danger zone (e.g. a foot in front)
    'backoff_trigger': 0.38,  # reverse when nearest front obstacle is within (m)
    'backoff_clear': 0.60,    # stop reversing once it is beyond this (m, hysteresis)
    'backoff_speed': 0.10,    # reverse speed (m/s)
    'backoff_rear_min': 0.50, # only reverse if the rear space is clearer than this (m)
}

# Last committed heading, stored in the WORLD (odom) frame so the commitment
# stays fixed while the robot rotates (in the body frame it would rotate away
# and cause left/right spin). _last_heading is the robot-frame projection.
_last_heading_world: Optional[float] = None
_spin_dir: Optional[float] = None     # committed turn-in-place direction


def _commit_spin(cmd, heading_err, w_max):
    """Latch one rotation direction while turning in place toward a far-off
    heading. At v=0 the CBF picks omega's sign from the momentarily-worst
    obstacle, which dithers ±ω forever when the goal is behind. Pure rotation
    of a round robot is collision-free, so the sign override is safe."""
    global _spin_dir
    if heading_err is None:
        _spin_dir = None
        return cmd
    if cmd.linear.x < 0.03 and abs(heading_err) > (0.9 if _spin_dir is None else 0.5):
        if _spin_dir is None:
            _spin_dir = 1.0 if heading_err > 0 else -1.0
        mag = max(0.6 * w_max, abs(cmd.angular.z))
        cmd.angular.z = float(np.clip(_spin_dir * mag, -w_max, w_max))
    else:
        _spin_dir = None
    return cmd
_last_heading: float = 0.0

# Back-off reflex state: _backing = currently reversing, _backoff_cycles =
# reversing duration (anti-latch), _backoff_cooldown = blocks re-triggering.
_backing: bool = False
_backoff_cycles: int = 0
_backoff_cooldown: int = 0
_BACKOFF_MAX_CYCLES = 16       # ~2 s at 8 Hz
_BACKOFF_COOLDOWN = 24         # ~3 s at 8 Hz

# Escape (turn-in-place) state: once turning one way, commit to that direction
# for _ESCAPE_HOLD cycles so the robot cannot vibrate left/right.
_escape_dir: float = 0.0
_escape_hold: int = 0
_ESCAPE_HOLD = 6               # ~0.75 s committed turn at 8 Hz

# Navigation goal in the ODOM frame.  None → drive straight forward.
_goal_odom: Optional[Tuple[float, float]] = None
_manual_cmd: Optional[Tuple[float, float]] = None   # (v, ω) manual override target


def set_goal(x: float, y: float) -> None:
    """Set the navigation goal (odom frame)."""
    global _goal_odom, _manual_cmd, _last_heading, _backing
    global _backoff_cycles, _backoff_cooldown
    global _last_heading_world, _spin_dir
    _spin_dir = None
    _goal_odom = (float(x), float(y))
    _manual_cmd = None
    _last_heading = 0.0          # forget the old commitment for a fresh goal
    _last_heading_world = None
    _backing = False
    _backoff_cycles = 0
    _backoff_cooldown = 0


def clear_goal() -> None:
    """Forget the goal → drive straight forward (still avoiding obstacles)."""
    global _goal_odom
    _goal_odom = None


def set_manual(v: float, omega: float) -> None:
    """Manual-drive intent (robot frame): nominal becomes this (v, ω), still CBF-filtered."""
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


_prev_cmd = [0.0, 0.0]


def _smooth(cmd: Twist) -> Twist:
    """Slew-rate limiter: smooth speed-ups and turn changes, instant braking."""
    v, w = float(cmd.linear.x), float(cmd.angular.z)
    pv, pw = _prev_cmd
    if v > pv:
        v = min(v, pv + _CFG['slew_v_rise'])
    dw = _CFG['slew_w']
    if abs(w - pw) > dw:
        w = pw + math.copysign(dw, w - pw)
    _prev_cmd[0], _prev_cmd[1] = v, w
    cmd.linear.x = v
    cmd.angular.z = w
    return cmd


def _angle_wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _gap_heading(obs: List[dict], goal_ang: float, robot_th: float = 0.0) -> Optional[float]:
    """Follow-the-Gap: pick the best free heading around the obstacles, trading
    goal-direction proximity against hysteresis toward the last committed heading.
    Returns the chosen heading (rad, robot frame), or None if fully boxed in."""
    global _last_heading, _last_heading_world
    # hysteresis reference in the ROBOT frame, derived from the world value
    if _last_heading_world is None:
        hyst_ref = None
    else:
        hyst_ref = _angle_wrap(_last_heading_world - robot_th)
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
                cost = abs(_angle_wrap(a - goal_ang))
                if hyst_ref is not None:
                    cost += _CFG['gap_hyst'] * abs(_angle_wrap(a - hyst_ref))
                if cost < best_cost:
                    best_cost, best = cost, a
        j = k

    if best is None:                                 # only narrow slivers free
        for i in range(N):
            if blocked[i]:
                continue
            a = -math.pi + (i + 0.5) * binw
            cost = abs(_angle_wrap(a - goal_ang))
            if hyst_ref is not None:
                cost += _CFG['gap_hyst'] * abs(_angle_wrap(a - hyst_ref))
            if cost < best_cost:
                best_cost, best = cost, a

    if best is not None:
        _last_heading = best                         # robot-frame projection
        _last_heading_world = _angle_wrap(best + robot_th)   # commit in WORLD
    return best


def obstacle_radius(track: Track, sigma_scale: float) -> float:
    """Conservative obstacle radius from the track's positional uncertainty:
    radius = sigma_scale × √(λ_max(P[:2, :2])). Grows with filter uncertainty."""
    pos_cov = np.asarray(track.P)[:2, :2]
    lambda_max = float(np.linalg.eigvalsh(pos_cov)[-1])
    return float(sigma_scale) * math.sqrt(max(lambda_max, 0.0))


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
        if getattr(tr, 'static', False):
            vx = vy = 0.0          # persistence says it does not move
        px += vx * t_pred
        py += vy * t_pred
        d0 = math.hypot(px, py)
        if d0 > influence or d0 < _CFG['min_track_dist']:
            continue            # too far to matter / robot self-detection
        # Clamp uncertainty inflation so a coasting track's bubble can't balloon mid-manoeuvre.
        cov_term = min(obstacle_radius(tr, obstacle_radius_scale),
                       _CFG['cors_cap'])
        r = base_person + cov_term
        obs.append({'px': px, 'py': py, 'vx': vx, 'vy': vy, 'r': r, 'person': True})

    if obstacle_points:
        for (ox, oy) in obstacle_points:
            ox = float(ox); oy = float(oy)
            if math.hypot(ox, oy) > influence:
                continue
            obs.append({'px': ox, 'py': oy, 'vx': 0.0, 'vy': 0.0,
                        'r': base_wall, 'person': False})
    return obs


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
    """Velocity command: navigate toward the goal while avoiding people and
    static obstacles, routing around them.

    `obstacle_points` is an optional list of (x, y) static obstacle points in
    the robot frame (downsampled scan returns); when omitted, only tracked
    people are avoided.
    """
    cmd = Twist()
    L = _CFG['lookahead']
    obs = _gather_obstacles(tracks, obstacle_points, obstacle_radius_scale)

    # 0. Back-off reflex: if an obstacle sits in the danger zone directly ahead
    # and there is room behind, reverse out until clear. A narrow ±35° cone
    # avoids triggering on gap-edge walls; an anti-latch gives up after
    # _BACKOFF_MAX_CYCLES with a cooldown so it does not immediately re-reverse.
    global _backing, _backoff_cycles, _backoff_cooldown
    front = [math.hypot(o['px'], o['py']) for o in obs
             if abs(math.atan2(o['py'], o['px'])) < math.radians(35)]
    rear = [math.hypot(o['px'], o['py']) for o in obs
            if abs(math.atan2(o['py'], o['px'])) > math.radians(105)]
    near_front = min(front, default=float('inf'))
    near_rear = min(rear, default=float('inf'))
    rear_min = _CFG['backoff_rear_min']
    if _backoff_cooldown > 0:
        _backoff_cooldown -= 1
    if _backing:
        _backoff_cycles += 1
        if near_front > _CFG['backoff_clear'] or near_rear < rear_min:
            _backing = False                           # cleared, or rear blocked
            _backoff_cycles = 0
        elif _backoff_cycles > _BACKOFF_MAX_CYCLES:
            _backing = False                           # latched — give up reversing
            _backoff_cycles = 0
            _backoff_cooldown = _BACKOFF_COOLDOWN       # let nav rotate instead
    elif (near_front < _CFG['backoff_trigger'] and near_rear > rear_min
          and _backoff_cooldown == 0):
        _backing = True
        _backoff_cycles = 0
    if _backing:
        v = -min(_CFG['backoff_speed'], max_linear_speed)
        if near_rear < rear_min + 0.3:                 # ease off near rear limit
            v *= max(0.0, (near_rear - rear_min) / 0.3)
        cmd.linear.x = float(v)
        cmd.angular.z = 0.0                            # back straight; re-plan once clear
        return _smooth(cmd)

    # 1. Nominal command
    heading_err = None
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

        # Precision docking near the goal: direct pure-pursuit (CBF still
        # guards), since gap/escape logic limit-cycles here.
        dock_mode = dist_goal < _CFG['dock_dist']
        # Follow-the-Gap: steer toward the best free opening.
        goal_ang = math.atan2(gdir[1], gdir[0])
        if dock_mode:
            gap = goal_ang
        else:
            gap = _gap_heading(obs, goal_ang, robot_theta)
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
            v_nom *= max(_CFG['slow_floor'],
                         min(1.0, nearest_edge / _CFG['slow_radius']))
        v_nom = min(v_nom, dist_goal)

    v_nom = float(np.clip(v_nom, 0.0, max_linear_speed))
    w_nom = float(np.clip(w_nom, -max_angular_speed, max_angular_speed))

    # 2. CBF safety constraints, one row per obstacle
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
        return _smooth(cmd)

    v, w = _solve_qp_2d(
        v_nom, w_nom, _CFG['w_omega'], rows,
        v_min=0.0, v_max=max_linear_speed, w_max=max_angular_speed,
    )

    # 3. Deadlock escape (last resort).
    # Bubble-exit: h<0 means we are inside an inflated safety bubble (not a
    # physical collision); the CBF can then freeze the QP at v=w=0. Creep out
    # along the away-direction at low speed — bounded and safe.
    L_ = _CFG['lookahead']
    worst_h, away = 0.0, None
    for o in obs:
        Ex, Ey = L_ - o['px'], -o['py']
        h_i = Ex*Ex + Ey*Ey - (o['r'] + L_)**2
        if h_i < worst_h:
            worst_h, away = h_i, math.atan2(-o['py'], -o['px'])
    if worst_h < 0 and v < 0.02 and away is not None:
        err = _angle_wrap(away)
        if abs(err) < 0.6:
            cmd.linear.x = 0.06
            cmd.angular.z = float(np.clip(1.5*err, -max_angular_speed, max_angular_speed))
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = float(np.clip(math.copysign(_CFG['escape_omega_frac']*max_angular_speed, err), -max_angular_speed, max_angular_speed))
        return _smooth(_commit_spin(cmd, heading_err, max_angular_speed))

    global _escape_dir, _escape_hold
    try:
        _dock = dock_mode
    except NameError:
        _dock = False
    blocked = (not _dock
               and v < _CFG['escape_v_frac'] * max(v_nom, 1e-3)
               and nearest_dist < _CFG['escape_dist'])
    if blocked:
        # Turn in place to get unblocked, committing to one direction for
        # _ESCAPE_HOLD cycles so the robot cannot vibrate left/right.
        if _escape_hold <= 0:
            if abs(_last_heading) > 1e-3:
                _escape_dir = 1.0 if _last_heading > 0 else -1.0
            else:
                _escape_dir = 1.0 if left_block <= right_block else -1.0
            _escape_hold = _ESCAPE_HOLD
        _escape_hold -= 1
        w = _escape_dir * _CFG['escape_omega_frac'] * max_angular_speed
        v = 0.0                                    # turn in place (always safe)
    else:
        _escape_hold = 0                           # cleared — re-decide next time

    cmd.linear.x = float(np.clip(v, 0.0, max_linear_speed))
    cmd.angular.z = float(np.clip(w, -max_angular_speed, max_angular_speed))
    return _smooth(_commit_spin(cmd, heading_err, max_angular_speed))
