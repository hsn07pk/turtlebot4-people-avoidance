#!/usr/bin/env python3
"""Live people-avoidance navigation dashboard (Stage 1+2+3).

Runs detection -> Kalman tracking -> CBF control on every /scan and serves an
interactive 3D/2D control panel with live sliders and manual drive on port 8080.
"""
import json
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import people_avoidance.leg_detection as legmod
from people_avoidance.leg_detection import (
    detect_legs, scan_to_cartesian, segment_scan, LegMeasurement)
from people_avoidance.tracking import KalmanTracker
from people_avoidance import controller as ctrl

TRAIL_LEN = 160

# Vendored Plotly served alongside this script, so the dashboard runs offline.
PLOTLY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'plotly.min.js')

# RPLidar A1 is mounted yaw +90° vs base_link, so rotate every laser point into base_link.
LASER_YAW = math.pi / 2.0
_CY, _SY = math.cos(LASER_YAW), math.sin(LASER_YAW)


def _to_base_xy(x, y):
    """Rotate a laser-frame (x, y) into the base_link frame."""
    return (_CY * x - _SY * y, _SY * x + _CY * y)


def _to_base_pts(pts):
    """Rotate an (N,2) array of laser points into base_link."""
    if pts.shape[0] == 0:
        return pts
    R = np.array([[_CY, -_SY], [_SY, _CY]])
    return pts @ R.T


def _to_base_meas(m):
    """Rotate a LegMeasurement (position + 2×2 covariance) into base_link."""
    x, y = _to_base_xy(m.x, m.y)
    R = np.array([[m.Rxx, m.Rxy], [m.Rxy, m.Ryy]])
    Rot = np.array([[_CY, -_SY], [_SY, _CY]])
    Rb = Rot @ R @ Rot.T
    return LegMeasurement(x=float(x), y=float(y),
                          Rxx=float(Rb[0, 0]), Rxy=float(Rb[0, 1]), Ryy=float(Rb[1, 1]))

# Live-tunable parameters (sliders write here; the scan callback reads).
params = {
    # Stage 1 detection
    'dt': 0.30, 'lr': 0.10, 'mlw': 0.41, 'circ': 0.80, 'minpts': 14, 'maxr': 1.30,
    # Stage 2 tracking
    'gate': 5.40, 'q': 1.0, 'horizon': 3.0, 'vmove': 0.3,
    # Stage 3 control
    'cmaxv': 0.35, 'cmaxw': 1.70, 'clook': 0.15, 'cgamma': 2.0, 'cwom': 0.10,
    'cpr': 0.10, 'cwr': 0.01, 'crr': 0.10, 'cinf': 0.60, 'cors': 2.0, 'ctpred': 0.30,
    'cgapclr': 0.04, 'cgaphyst': 0.5,
    'cbtrig': 0.40, 'cbclear': 0.60, 'cbspeed': 0.10, 'cbrear': 0.50,
    'cdrive': 0.0,
}

state = {
    'snapshot': {'points': [], 'legs': [], 'nseg': 0, 'ndet': 0, 'nscan': 0,
                 'dt_est': 0.0, 'cmd': {'v': 0.0, 'w': 0.0, 'drive': False},
                 'tracks': [], 'preds': {}, 'trails': {}, 'obstacles': [],
                 'goal': None, 'path': [], 'mode': 'auto'},
    'tracker': None, 'trails': {}, 'last_stamp': None, 'dt_est': 0.13,
    'pose': (0.0, 0.0, 0.0), 'have_pose': False,
    'cmd_pub': None, 'mode': 'auto',
    'latest_cmd': (0.0, 0.0),    # (v, ω) published at high rate by a timer
}

WIENER_Q = lambda q, dt: q * np.array(
    [[dt**3/3, 0, dt**2/2, 0], [0, dt**3/3, 0, dt**2/2],
     [dt**2/2, 0, dt, 0], [0, dt**2/2, 0, dt]])


def _yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def on_odom(msg):
    p = msg.pose.pose
    nx, ny, nth = p.position.x, p.position.y, _yaw(p.orientation)
    if state['have_pose']:
        ox, oy, oth = state['pose']
        dyaw = abs(math.atan2(math.sin(nth - oth), math.cos(nth - oth)))
        # Create3 odometry can teleport; a frame jump corrupts odom-frame tracks, so reset.
        if math.hypot(nx - ox, ny - oy) > 0.35 or dyaw > 0.5:
            state['tracker'] = None
            state['trails'] = {}
            print(f'[guard] odom teleport ({math.hypot(nx-ox,ny-oy):.2f} m, '
                  f'{dyaw:.2f} rad) — tracker reset', flush=True)
    state['pose'] = (nx, ny, nth)
    state['have_pose'] = True


def _ensure_tracker():
    tr = state['tracker']
    if tr is None:
        # max_misses=8 (~1 s): standing people briefly drop detections; lower respawns their tracks.
        tr = KalmanTracker(dt=state['dt_est'], process_noise_density=params['q'],
                           gate_chi2=params['gate'], max_misses=8)
        state['tracker'] = tr
    tr.gate_chi2 = params['gate']
    tr.Q = WIENER_Q(params['q'], tr.dt)
    return tr


def _extract_obstacles(pts, tracks, influence, person_r):
    """Nearest scan point per angular sector within influence, excluding tracked people."""
    if pts.shape[0] == 0:
        return []
    centers = [(float(t.m[0]), float(t.m[1])) for t in tracks if t.confirmed]
    nsec = 72
    best = {}
    for (x, y) in pts:
        d = math.hypot(x, y)
        if d > influence or d < 0.12:
            continue
        # Keep rear obstacles too: the back-off reflex checks if the space behind is clear.
        if any(math.hypot(x-cx, y-cy) < person_r for (cx, cy) in centers):
            continue
        sec = int((math.atan2(y, x) + math.pi) / (2*math.pi) * nsec)
        if sec not in best or d < best[sec][2]:
            best[sec] = (float(x), float(y), d)
    return [(v[0], v[1]) for v in best.values()]


def _rollout_path(v, w, dt=0.2, n=12):
    """Predict the robot's short-horizon path under the current command."""
    x = y = th = 0.0
    out = []
    for _ in range(n):
        x += v*math.cos(th)*dt; y += v*math.sin(th)*dt; th += w*dt
        out.append([round(x, 2), round(y, 2)])
    return out


def on_scan(scan):
    t = scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9
    if state['last_stamp'] is not None:
        d = t - state['last_stamp']
        if 0.01 < d < 1.0:
            state['dt_est'] = 0.9*state['dt_est'] + 0.1*d
    state['last_stamp'] = t

    legmod.CIRCULARITY_MIN = params['circ']
    legmod.MIN_CLUSTER_POINTS = int(params['minpts'])

    pts = _to_base_pts(scan_to_cartesian(scan))
    segs = segment_scan(pts, distance_threshold=params['dt'],
                        angle_increment=scan.angle_increment) if pts.shape[0] else []
    dets = [_to_base_meas(m) for m in
            detect_legs(scan, distance_threshold=params['dt'],
                        leg_radius=params['lr'], max_leg_width=params['mlw'])]
    # drop the robot's own mounting posts (self-detections at ~0.2 m)
    dets = [m for m in dets if math.hypot(m.x, m.y) > 0.30]

    # Track in the odom (world) frame so static people stay static as the robot moves.
    rx0, ry0, rth0 = state['pose']
    c0, s0 = math.cos(rth0), math.sin(rth0)
    odom_dets = []
    for m in dets:
        ox_ = rx0 + c0*m.x - s0*m.y
        oy_ = ry0 + s0*m.x + c0*m.y
        rxx = c0*c0*m.Rxx - 2*c0*s0*m.Rxy + s0*s0*m.Ryy
        ryy = s0*s0*m.Rxx + 2*c0*s0*m.Rxy + c0*c0*m.Ryy
        rxy = c0*s0*m.Rxx + (c0*c0 - s0*s0)*m.Rxy - c0*s0*m.Ryy
        odom_dets.append(LegMeasurement(x=ox_, y=oy_, Rxx=rxx, Rxy=rxy, Ryy=ryy))

    tr = _ensure_tracker()
    tr.update(odom_dets)
    tracks_odom = tr.get_tracks()

    class _RT:                       # robot-frame view of an odom track
        __slots__ = ('m', 'P', 'track_id', 'confirmed', 'static', 'hits', 'misses')
    tracks = []
    for t in tracks_odom:
        xo, yo, vxo, vyo = (float(v) for v in t.m)
        dx_, dy_ = xo - rx0, yo - ry0
        rt = _RT()
        rt.m = np.array([c0*dx_ + s0*dy_, -s0*dx_ + c0*dy_,
                         c0*vxo + s0*vyo, -s0*vxo + c0*vyo])
        rt.P = t.P                   # eigenvalues are rotation-invariant
        rt.track_id = t.track_id
        rt.confirmed = t.confirmed
        rt.static = bool(getattr(t, 'static', False))
        rt.hits, rt.misses = t.hits, t.misses
        tracks.append(rt)

    obstacles = _extract_obstacles(pts, tracks, params['cinf'], params['cpr'])

    ctrl.set_params(lookahead=params['clook'], gamma=params['cgamma'],
                    w_omega=params['cwom'], person_radius=params['cpr'],
                    wall_radius=params['cwr'], robot_radius=params['crr'],
                    influence=params['cinf'], t_pred=params['ctpred'],
                    gap_clear=params['cgapclr'], gap_hyst=params['cgaphyst'],
                    backoff_trigger=params['cbtrig'], backoff_clear=params['cbclear'],
                    backoff_speed=params['cbspeed'], backoff_rear_min=params['cbrear'])
    rx, ry, rth = state['pose']
    cmd = ctrl.compute_velocity(tracks, rx, ry, rth,
                                max_linear_speed=params['cmaxv'],
                                max_angular_speed=params['cmaxw'],
                                obstacle_radius_scale=params['cors'],
                                obstacle_points=obstacles)
    cv, cw = float(cmd.linear.x), float(cmd.angular.z)
    # The 50 Hz timer publishes; high rate so our commands beat TB4 teleop_twist_joy_node's zero-floods.
    state['latest_cmd'] = (cv, cw)

    # goal in robot frame (for drawing)
    goal_robot = None
    gp = ctrl._goal_odom
    if gp is not None and state['have_pose']:
        ct, st = math.cos(rth), math.sin(rth)
        dx, dy = gp[0]-rx, gp[1]-ry
        goal_robot = [round(ct*dx+st*dy, 2), round(-st*dx+ct*dy, 2)]

    trails = state['trails']; rows = []; live = {}
    def _to_rob(xo_, yo_):
        dx_, dy_ = xo_ - rx0, yo_ - ry0
        return (c0*dx_ + s0*dy_, -s0*dx_ + c0*dy_)
    for t, to in zip(tracks, tracks_odom):
        x, y, vx, vy = (float(v) for v in t.m)
        # same clamp the controller applies — display matches decisions
        cov_r = min(ctrl.obstacle_radius(t, params['cors']),
                    ctrl._CFG['cors_cap'])
        rows.append({'id': t.track_id, 'x': round(x, 3), 'y': round(y, 3),
                     'vx': round(vx, 3), 'vy': round(vy, 3),
                     'speed': round(math.hypot(float(to.m[2]), float(to.m[3])), 3),
                     'conf': t.confirmed, 'static': bool(getattr(t, 'static', False)),
                     'r': round(params['cpr']+params['crr']+cov_r, 2)})
        h = trails.get(t.track_id, [])
        h.append((round(float(to.m[0]), 2), round(float(to.m[1]), 2)))   # odom
        live[t.track_id] = h[-TRAIL_LEN:]
    state['trails'] = live
    preds_odom = tr.predict_ahead(params['horizon'])
    preds = {str(t.track_id): [[round(v_, 2) for v_ in _to_rob(float(r[0]), float(r[1]))]
                               for r in preds_odom.get(t.track_id, [])]
             for t in tracks_odom}

    point_rows, sid = [], 0
    for seg in segs:
        for q in seg:
            point_rows.append([round(float(q[0]), 3), round(float(q[1]), 3), sid])
        sid += 1

    state['snapshot'] = {
        'points': point_rows,
        'legs': [[round(m.x, 3), round(m.y, 3), round(math.hypot(m.x, m.y), 3)] for m in dets],
        'nseg': sid, 'ndet': len(dets), 'nscan': state['snapshot']['nscan']+1,
        'dt_est': round(state['dt_est'], 3),
        'cmd': {'v': round(cv, 3), 'w': round(cw, 3), 'drive': params['cdrive'] >= 0.5},
        'tracks': rows, 'preds': preds,
        'trails': {str(k): [[round(a_, 2) for a_ in _to_rob(px_, py_)]
                            for (px_, py_) in v[::2]] for k, v in live.items()},
        'obstacles': [[round(o[0], 2), round(o[1], 2)] for o in obstacles],
        'goal': goal_robot, 'path': _rollout_path(cv, cw),
        'influence': params['cinf'], 'lookahead': params['clook'],
        'robot_r': params['crr'], 'mode': state['mode'],
        'have_pose': state['have_pose'],
    }


class PipeNode(Node):
    def __init__(self):
        super().__init__('nav_dashboard')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/scan', on_scan, qos)
        self.create_subscription(Odometry, '/odom', on_odom, qos)
        state['cmd_pub'] = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.02, self._drive_timer)   # 50 Hz publish loop

    def _drive_timer(self):
        if params['cdrive'] < 0.5 or state['cmd_pub'] is None:
            return
        v, w = state['latest_cmd']
        tw = Twist(); tw.linear.x = float(v); tw.angular.z = float(w)
        state['cmd_pub'].publish(tw)


TIP = {
 'dt': 'Segmentation gap floor (m). Larger merges nearby points into one cluster; smaller splits them. Adaptive threshold r·Δθ+3σ is floored by this.',
 'lr': 'Expected single-leg radius (m). Sets the leg width gate (2·lr). Calf ≈0.06 m.',
 'mlw': 'Max separation (m) between two leg clusters to pair them into one person. Lecture d_max≈0.40 m.',
 'circ': 'Circularity floor for the wall-rejection filter. Higher rejects more wall-like (elongated) clusters.',
 'minpts': 'Minimum points per cluster. Higher drops noise/clutter but shortens detection range (Leigh/SPENCER use 3).',
 'maxr': 'Display range filter (m): hide detections/tracks beyond this. Visualization only.',
 'gate': 'Mahalanobis χ² gate for data association. 5.99=95%, 9.21=99%. Higher tolerates faster motion but admits more clutter.',
 'q': 'Process-noise spectral density of the constant-velocity Kalman model. Higher = tracks react faster but noisier velocity.',
 'horizon': 'Prediction horizon (s): how far ahead predict_ahead extrapolates each track (the ghost trail).',
 'vmove': 'Speed (m/s) above which a track is shown as "moving" (orange) vs static (blue).',
 'cmaxv': 'Max forward speed the controller may command (m/s).',
 'cmaxw': 'Max turn rate the controller may command (rad/s).',
 'clook': 'CBF lookahead distance L (m): the virtual probe ahead of the robot the barrier is evaluated at. Lets the QP steer (not just brake).',
 'cgamma': 'CBF class-K gain γ. Small = cautious/early avoidance; large = hugs the safety boundary.',
 'cwom': 'QP weight on ω vs v (<1 ⇒ "steer before brake"): the safety filter prefers turning over slowing.',
 'cpr': 'Person radius (m): half-width of a person added to the safety bubble.',
 'cwr': 'Wall/obstacle point radius (m): clearance kept from each scan obstacle point.',
 'crr': 'Robot footprint radius (m): added to every safety bubble.',
 'cinf': 'Influence radius (m): obstacles farther than this are ignored by the controller.',
 'cors': 'Obstacle-radius scale k: inflates a person’s bubble by k·σ of the track position uncertainty.',
 'ctpred': 'Barrier prediction t_pred (s): advance each person by velocity·t_pred before testing the CBF (anticipate motion).',
 'cgapclr': 'Follow-the-Gap extra clearance (m) added to each obstacle when searching for free openings. Higher = wider berth around obstacles.',
 'cgaphyst': 'Follow-the-Gap hysteresis: how strongly the robot commits to its last chosen direction. Higher = less left/right oscillation, commits harder to going around one side.',
 'cbtrig': 'Back-off trigger (m): reverse when the nearest front obstacle gets this close (a foot stepping in front).',
 'cbclear': 'Back-off clear (m): stop reversing once the obstacle is beyond this distance (hysteresis vs the trigger).',
 'cbspeed': 'Back-off reverse speed (m/s) when escaping the danger zone.',
 'cbrear': 'Back-off rear safety (m): only reverse if the space behind is clearer than this (≈ robot radius + margin).',
}

# (key, full display name, min, max, step)
SLIDERS = [
 ('STAGE 1 — LEG DETECTION', [
   ('dt', 'Segmentation distance threshold (m)', 0.02, 0.50, 0.005),
   ('lr', 'Leg radius (m)', 0.03, 0.35, 0.005),
   ('mlw', 'Max leg-pair width (m)', 0.10, 0.90, 0.01),
   ('circ', 'Circularity wall-reject', 0, 0.95, 0.01),
   ('minpts', 'Min points per cluster', 2, 25, 1),
   ('maxr', 'Max display range (m)', 0.5, 8, 0.1)]),
 ('STAGE 2 — KALMAN TRACKING', [
   ('gate', 'Mahalanobis gate (χ²)', 2, 15, 0.1),
   ('q', 'Process noise density q', 0.1, 5, 0.1),
   ('horizon', 'Prediction horizon (s)', 0.5, 4, 0.5),
   ('vmove', 'Moving-speed threshold (m/s)', 0.1, 1.0, 0.05)]),
 ('STAGE 3 — CONTROL (CBF + potential field)', [
   ('cmaxv', 'Max linear speed (m/s)', 0, 0.5, 0.01),
   ('cmaxw', 'Max angular speed (rad/s)', 0, 2.0, 0.05),
   ('clook', 'CBF lookahead distance L (m)', 0.1, 0.6, 0.01),
   ('cgamma', 'CBF class-K gain γ', 0.3, 8, 0.1),
   ('cwom', 'QP ω-weight (steer ↔ brake)', 0.02, 1.0, 0.02),
   ('cpr', 'Person radius (m)', 0.02, 0.6, 0.01),
   ('cwr', 'Wall / obstacle-point radius (m)', 0.0, 0.3, 0.01),
   ('crr', 'Robot footprint radius (m)', 0.05, 0.4, 0.01),
   ('cinf', 'Obstacle influence radius (m)', 0.3, 5.0, 0.05),
   ('cors', 'Obstacle uncertainty scale (σ)', 0.0, 4.0, 0.1),
   ('ctpred', 'Barrier prediction time t_pred (s)', 0.0, 1.0, 0.05),
   ('cgapclr', 'Follow-the-Gap clearance (m)', 0.0, 0.4, 0.01),
   ('cgaphyst', 'Follow-the-Gap commitment (hysteresis)', 0.0, 2.0, 0.05),
   ('cbtrig', 'Back-off trigger distance (m)', 0.2, 1.0, 0.05),
   ('cbclear', 'Back-off clear distance (m)', 0.4, 1.5, 0.05),
   ('cbspeed', 'Back-off reverse speed (m/s)', 0.0, 0.25, 0.01),
   ('cbrear', 'Back-off rear safety (m)', 0.3, 1.0, 0.05)]),
]


def _build_sliders_html():
    out = []
    for section, items in SLIDERS:
        out.append(f'<div class="sec">{section}</div>')
        for (k, name, lo, hi, st) in items:
            tip = TIP.get(k, '').replace('"', '&quot;')
            out.append(
              f'<div class="row" title="{tip}"><label>{name}'
              f'<span class="val" id="v{k}"></span></label>'
              f'<input type=range id={k} min={lo} max={hi} step={st} value={params[k]}></div>')
    return '\n'.join(out)


HTML_HEAD = """<!doctype html><html><head><meta charset="utf-8"><title>People-Avoidance Navigation</title>
<script src="/plotly.min.js"></script>
<style>
body{margin:0;font-family:sans-serif;background:#0e0e0e;color:#eee;display:flex}
#plot{flex:1;height:100vh;background:#fff}
#panel{width:340px;padding:12px;background:#1a1a1a;overflow:auto;box-sizing:border-box}
h3{margin:0 0 4px} .row{margin:8px 0}
label{display:block;font-size:12px;margin-bottom:3px}
input[type=range]{width:100%} .val{color:#4cf;float:right}
#status{margin-top:10px;font-size:12px;line-height:1.7;border-top:1px solid #333;padding-top:8px}
.red{color:#ff5252}.org{color:#ffa726}.blu{color:#64b5f6}.grn{color:#69f0ae}
button{margin-top:6px;padding:6px;background:#333;color:#eee;border:1px solid #555;cursor:pointer}
small{color:#888} .sec{margin-top:10px;border-top:1px solid #333;padding-top:6px;font-size:12px;color:#9ad}
.modes{display:flex;gap:4px;margin:6px 0}.modes button{flex:1;font-size:12px}
.modes button.on{background:#2962ff;border-color:#2962ff}
#mini{background:#101820;border:1px solid #333;cursor:crosshair;display:block;margin:4px 0}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:3px;margin:4px 0}
.pad button{padding:8px 0}
.drive{background:#3a1010;padding:6px;border-radius:4px;margin-top:6px}
.row[title]{cursor:help}
</style></head><body>
<div id="plot"></div>
<div id="panel">
<h3>People-Avoidance Navigation</h3><small>Stage 1 + 2 + 3 · live on the TurtleBot4</small>
<div class="sec">NAVIGATION</div>
<div class="modes">
 <button id="ma" onclick="setMode('auto')">AUTO (click map)</button>
 <button id="mm" onclick="setMode('manual')">MANUAL</button>
 <button id="ms" onclick="stopAll()">STOP</button>
</div>
<canvas id="mini" width="312" height="312"></canvas>
<small id="navhint">AUTO: click the map to set a goal. MANUAL: arrow keys / WASD or buttons.</small>
<div class="pad" id="manpad" style="display:none">
 <button onclick="man(0,1)">↰</button><button onclick="man(1,0)">↑</button><button onclick="man(0,-1)">↱</button>
 <button onclick="man(-1,0)">↓</button><button onclick="man(0,0)">■</button><button onclick="man(1,0)">↑</button>
</div>
<div class="drive">
 <label title="OFF = preview only (computes commands, robot does not move). ON = publishes /cmd_vel and the robot drives, CBF-filtered.">
 <input type=checkbox id=cdrive onchange="vals()"> <b>Drive robot</b> (publish /cmd_vel)</label></div>
"""

HTML_TAIL = """
<button onclick="fetch('/reset')" style="width:100%">Reset tracker</button>
<div class="row" title="default = lab-tuned; test-1-research = literature + on-robot measured values">
<label>Config preset</label>
<select id="preset" onchange="applyPreset()" style="width:100%;padding:6px;background:#222;color:#eee;border:1px solid #555">
 <option value="research">test-1-research</option><option value="default">default (lab-tuned)</option>
</select></div>
<div id="status"></div>
</div>
<script>
var ids=__IDS__;
var PRESETS={
 'research':{dt:0.13,lr:0.06,mlw:0.65,circ:0.20,minpts:3,maxr:4.0,gate:9.21,q:1.0,horizon:2.0,vmove:0.30,cmaxv:0.20,cmaxw:1.00,clook:0.15,cgamma:2.0,cwom:0.10,cpr:0.30,cwr:0.03,crr:0.18,cinf:1.5,cors:1.0,ctpred:0.30,cgapclr:0.04,cgaphyst:0.5,cbtrig:0.38,cbclear:0.60,cbspeed:0.10,cbrear:0.50},
 'default':{dt:0.30,lr:0.10,mlw:0.41,circ:0.80,minpts:14,maxr:1.30,gate:5.40,q:1.0,horizon:3.0,vmove:0.30,cmaxv:0.35,cmaxw:1.70,clook:0.15,cgamma:2.0,cwom:0.10,cpr:0.10,cwr:0.01,crr:0.10,cinf:0.60,cors:2.0,ctpred:0.30,cgapclr:0.04,cbtrig:0.40,cgaphyst:0.5,cbclear:0.60,cbspeed:0.10,cbrear:0.50}};
function applyPreset(){var p=PRESETS[document.getElementById('preset').value];if(!p)return;
 ids.forEach(function(i){if(p[i]!==undefined)document.getElementById(i).value=p[i];});vals();}
function vals(){var o={};ids.forEach(function(i){o[i]=document.getElementById(i).value;
 var e=document.getElementById('v'+i);if(e)e.textContent=parseFloat(o[i]).toFixed(i=='minpts'?0:2);});
 o['cdrive']=document.getElementById('cdrive').checked?1:0;return o;}
ids.forEach(function(i){document.getElementById(i).addEventListener('input',vals);});

var mode='auto';
function setMode(m){mode=m;
 document.getElementById('ma').className=(m=='auto')?'on':'';
 document.getElementById('mm').className=(m=='manual')?'on':'';
 document.getElementById('manpad').style.display=(m=='manual')?'grid':'none';
 fetch('/mode?m='+m);}
function stopAll(){fetch('/manual?v=0&w=0');fetch('/clear_goal');setMode('manual');}
function man(vd,wd){var v=vd*parseFloat(document.getElementById('cmaxv').value);
 var w=wd*parseFloat(document.getElementById('cmaxw').value);
 setMode('manual');fetch('/manual?v='+v.toFixed(3)+'&w='+w.toFixed(3));}
document.addEventListener('keydown',function(e){
 if(['INPUT','SELECT'].indexOf(e.target.tagName)>=0)return;
 var v=parseFloat(document.getElementById('cmaxv').value),w=parseFloat(document.getElementById('cmaxw').value);
 var k=e.key.toLowerCase(),vd=0,wd=0;
 if(k=='arrowup'||k=='w')vd=1;else if(k=='arrowdown'||k=='s')vd=-1;
 else if(k=='arrowleft'||k=='a')wd=1;else if(k=='arrowright'||k=='d')wd=-1;
 else if(k==' ')vd=wd=0;else return;
 e.preventDefault();setMode('manual');fetch('/manual?v='+(vd*v).toFixed(3)+'&w='+(wd*w).toFixed(3));});

// minimap: top-down, robot at center facing up (+x up). click -> goal (robot frame).
var mini=document.getElementById('mini'),mctx=mini.getContext('2d');
var MSCALE=312/8;  // 8 m across
function w2p(x,y){return [156 - y*MSCALE, 156 - x*MSCALE];}  // x fwd=up, y left=left
mini.addEventListener('click',function(ev){
 var rc=mini.getBoundingClientRect();var px=ev.clientX-rc.left,py=ev.clientY-rc.top;
 var gx=(156-py)/MSCALE, gy=(156-px)/MSCALE;   // robot-frame goal
 setMode('auto');fetch('/goal?gx='+gx.toFixed(2)+'&gy='+gy.toFixed(2));});
function drawMini(d){
 mctx.fillStyle='#101820';mctx.fillRect(0,0,312,312);
 mctx.strokeStyle='#243';for(var r=1;r<=4;r++){mctx.beginPath();mctx.arc(156,156,r*MSCALE,0,7);mctx.stroke();}
 // obstacles
 mctx.fillStyle='#888';d.obstacles.forEach(function(o){var p=w2p(o[0],o[1]);mctx.fillRect(p[0]-1,p[1]-1,2,2);});
 // tracks
 d.tracks.forEach(function(t){if(Math.hypot(t.x,t.y)>4)return;var p=w2p(t.x,t.y);
  mctx.fillStyle=t.speed>0.3?'#ffa726':'#64b5f6';mctx.beginPath();mctx.arc(p[0],p[1],4,0,7);mctx.fill();
  mctx.strokeStyle='#ffffff33';mctx.beginPath();mctx.arc(p[0],p[1],t.r*MSCALE,0,7);mctx.stroke();});
 // path
 mctx.strokeStyle='#00e676';mctx.beginPath();var o0=w2p(0,0);mctx.moveTo(o0[0],o0[1]);
 d.path.forEach(function(pp){var p=w2p(pp[0],pp[1]);mctx.lineTo(p[0],p[1]);});mctx.stroke();
 // goal
 if(d.goal){var g=w2p(d.goal[0],d.goal[1]);mctx.fillStyle='#e53935';mctx.beginPath();mctx.arc(g[0],g[1],5,0,7);mctx.fill();
  mctx.strokeStyle='#e53935';mctx.beginPath();mctx.moveTo(156,156);mctx.lineTo(g[0],g[1]);mctx.stroke();}
 // robot
 mctx.fillStyle='#33ff88';mctx.beginPath();mctx.arc(156,156,5,0,7);mctx.fill();
 mctx.strokeStyle='#33ff88';mctx.beginPath();mctx.moveTo(156,156);mctx.lineTo(156,156-14);mctx.stroke();
}

var PAL=['#e53935','#1e88e5','#ef9a9a','#64b5f6','#b71c1c','#f06292'];
function tcol(id){return PAL[((id%6)+6)%6];}
var layout={paper_bgcolor:'#fff',uirevision:'k',scene:{uirevision:'k',
 xaxis:{range:[-6,6],showticklabels:false,title:{text:''},gridcolor:'#dcdcdc',zeroline:false},
 yaxis:{range:[-6,6],showticklabels:false,title:{text:''},gridcolor:'#dcdcdc',zeroline:false},
 zaxis:{range:[0,2.3],showticklabels:false,title:{text:''},showgrid:false,zeroline:false,backgroundcolor:'#fafafa',showbackground:true},
 aspectmode:'manual',aspectratio:{x:1,y:1,z:0.3},camera:{eye:{x:1.4,y:1.4,z:0.9}}},
 margin:{l:0,r:0,t:0,b:0},showlegend:false};
var inited=false;
function circleXY(cx,cy,r,z){var x=[],y=[],zz=[];for(var i=0;i<=28;i++){var a=2*Math.PI*i/28;x.push(cx+r*Math.cos(a));y.push(cy+r*Math.sin(a));zz.push(z);}return{x:x,y:y,z:zz};}
function build(d,maxr,vmove){
 var data=[];
 data.push({type:'scatter3d',mode:'markers',x:d.points.map(p=>p[0]),y:d.points.map(p=>p[1]),z:d.points.map(_=>0.01),marker:{size:1.8,color:'#3a3a3a',opacity:0.5},hoverinfo:'skip'});
 data.push({type:'scatter3d',mode:'markers',x:d.obstacles.map(o=>o[0]),y:d.obstacles.map(o=>o[1]),z:d.obstacles.map(_=>0.03),marker:{size:3,color:'#ff9800'},hoverinfo:'skip'});
 var lg=d.legs.filter(l=>l[2]<=maxr);
 data.push({type:'scatter3d',mode:'markers',x:lg.map(l=>l[0]),y:lg.map(l=>l[1]),z:lg.map(_=>0.05),marker:{size:4,color:'#ff2222'},hoverinfo:'skip'});
 var tx=[],ty=[],tz=[],tc=[],lx=[],ly=[],lt=[],lc=[],rx=[],ry=[],rz=[],trx=[],trY=[],trz=[],trc=[];
 d.tracks.forEach(function(t){if(Math.hypot(t.x,t.y)>maxr)return;var col=tcol(t.id);
  tx.push(t.x,t.x,null);ty.push(t.y,t.y,null);tz.push(0,1.7,null);tc.push(col,col,col);
  lx.push(t.x);ly.push(t.y);lt.push('#'+t.id+(t.static?' (static)':(t.speed>vmove?' '+t.speed.toFixed(1):'')));lc.push(col);
  var c=circleXY(t.x,t.y,t.r,0.02);for(var i=0;i<c.x.length;i++){rx.push(c.x[i]);ry.push(c.y[i]);rz.push(c.z[i]);}rx.push(null);ry.push(null);rz.push(null);
  var tr=d.trails[String(t.id)]||[];tr.forEach(function(p){trx.push(p[0]);trY.push(p[1]);trz.push(0.02);trc.push(col);});trx.push(null);trY.push(null);trz.push(null);trc.push(col);});
 data.push({type:'scatter3d',mode:'lines',x:trx,y:trY,z:trz,line:{color:trc,width:5},hoverinfo:'skip'});
 data.push({type:'scatter3d',mode:'lines',x:tx,y:ty,z:tz,line:{color:tc,width:7},hoverinfo:'skip'});
 data.push({type:'scatter3d',mode:'lines',x:rx,y:ry,z:rz,line:{color:'#e53935',width:3},hoverinfo:'skip'});  // safety radii
 data.push({type:'scatter3d',mode:'text',x:lx,y:ly,z:lx.map(_=>1.85),text:lt,textfont:{color:lc,size:15},hoverinfo:'skip'});
 // influence ring
 var ic=circleXY(0,0,d.influence,0.01);data.push({type:'scatter3d',mode:'lines',x:ic.x,y:ic.y,z:ic.z,line:{color:'#90caf9',width:2,dash:'dot'},hoverinfo:'skip'});
 // lookahead point
 data.push({type:'scatter3d',mode:'markers',x:[d.lookahead],y:[0],z:[0.1],marker:{size:5,color:'#7e57c2'},hoverinfo:'skip'});
 // predicted path
 data.push({type:'scatter3d',mode:'lines',x:[0].concat(d.path.map(p=>p[0])),y:[0].concat(d.path.map(p=>p[1])),z:d.path.map(_=>0.06).concat([0.06]),line:{color:'#00e676',width:6},hoverinfo:'skip'});
 // goal
 if(d.goal){data.push({type:'scatter3d',mode:'markers+text',x:[d.goal[0]],y:[d.goal[1]],z:[0.1],text:['GOAL'],textfont:{color:'#e53935',size:13},marker:{size:8,color:'#e53935',symbol:'diamond'},hoverinfo:'skip'});}
 // robot + command arrow
 data.push({type:'scatter3d',mode:'markers+text',x:[0],y:[0],z:[0.1],text:['ROBOT'],textfont:{color:'#1565c0',size:11},textposition:'bottom center',marker:{size:9,color:'#1565c0',symbol:'square'},hoverinfo:'skip'});
 var ang=d.cmd.w*0.6,len=0.35+d.cmd.v*2.0,hx=len*Math.cos(ang),hy=len*Math.sin(ang);
 data.push({type:'scatter3d',mode:'lines',x:[0,hx],y:[0,hy],z:[0.14,0.14],line:{color:d.cmd.drive?'#00e676':'#9e9e9e',width:11},hoverinfo:'skip'});
 return data;
}
function tick(){var v=vals();var qs=ids.map(i=>i+'='+v[i]).join('&')+'&cdrive='+v.cdrive;
 fetch('/data?'+qs).then(r=>r.json()).then(function(d){
  var maxr=parseFloat(v.maxr),vmove=parseFloat(v.vmove);
  var data=build(d,maxr,vmove);
  if(!inited){Plotly.newPlot('plot',data,layout,{responsive:true});inited=true;}else{Plotly.react('plot',data,layout);}
  drawMini(d);
  var moving=d.tracks.filter(t=>t.conf&&t.speed>vmove).length;
  var h='scans '+d.nscan+' (dt≈'+d.dt_est+'s) · pose '+(d.have_pose?'ok':'<span class=red>NO /odom</span>')+'<br>'+
   'clusters '+d.nseg+' · <span class=red>dets '+d.ndet+'</span> · obstacles '+d.obstacles.length+'<br>'+
   'tracks '+d.tracks.length+' (<span class=org>moving '+moving+'</span>) · mode '+d.mode+'<br>'+
   'CMD ['+(d.cmd.drive?'<span style="color:#00e676">DRIVING</span>':'preview')+']: v='+d.cmd.v.toFixed(2)+' ω='+d.cmd.w.toFixed(2)+
   (d.goal?'<br>goal: '+Math.hypot(d.goal[0],d.goal[1]).toFixed(2)+' m away':'');
  document.getElementById('status').innerHTML=h;
 }).catch(_=>{});}
vals();setMode('auto');setInterval(tick,200);
</script></body></html>"""


def page():
    body = HTML_HEAD + _build_sliders_html() + HTML_TAIL
    return body.replace('__IDS__', json.dumps([k for _, items in SLIDERS for (k, _n, _l, _h, _s) in items]))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype='text/html'):
        self.send_response(200); self.send_header('Content-Type', ctype); self.end_headers()
        self.wfile.write(body if isinstance(body, bytes) else body.encode())

    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query)
        if u.path == '/data':
            for k in params:
                if k in q:
                    try: params[k] = float(q[k][0])
                    except ValueError: pass
            self._send(json.dumps(state['snapshot']).encode(), 'application/json')
        elif u.path == '/goal':
            gx = float(q.get('gx', [0])[0]); gy = float(q.get('gy', [0])[0])
            rx, ry, rth = state['pose']
            ct, st = math.cos(rth), math.sin(rth)
            ctrl.set_goal(rx + ct*gx - st*gy, ry + st*gx + ct*gy)   # robot→odom
            state['mode'] = 'auto'; self._send(b'ok')
        elif u.path == '/manual':
            ctrl.set_manual(float(q.get('v', [0])[0]), float(q.get('w', [0])[0]))
            state['mode'] = 'manual'; self._send(b'ok')
        elif u.path == '/mode':
            m = q.get('m', ['auto'])[0]; state['mode'] = m
            if m == 'auto': ctrl.clear_manual()
            self._send(b'ok')
        elif u.path == '/clear_goal':
            ctrl.clear_goal(); self._send(b'ok')
        elif u.path == '/plotly.min.js':
            try:
                with open(PLOTLY_PATH, 'rb') as f:
                    self._send(f.read(), 'application/javascript')
            except OSError:
                self.send_error(404)
        elif u.path == '/params':
            self._send(json.dumps(params).encode(), 'application/json')
        elif u.path == '/reset':
            state['tracker'] = None; state['trails'] = {}; self._send(b'ok')
        else:
            self._send(page())


def main():
    rclpy.init()
    node = PipeNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    ThreadingHTTPServer(('0.0.0.0', 8080), Handler).serve_forever()


if __name__ == '__main__':
    main()
