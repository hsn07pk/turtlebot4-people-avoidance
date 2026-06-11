#!/usr/bin/env python3
"""
Live people-tracking 3D visualizer + tuner (Stage 1 + both Stage 2 trackers),
SPENCER-style rendering (Linder & Arras, "Real-Time Multi-Modal People
Tracking for Mobile Robots in Crowded Environments").

Runs the full pipeline on every /scan with TWO trackers in parallel on the
same detections:

    Tracking 1 — tracking.py    : Hungarian assignment + lifecycle
    Tracking 2 — tracking_2.py  : greedy NN from the course notebook

SPENCER-style scene per tracker: white world with a floor grid, a 3D
humanoid figure per track coloured by ID (faded while coasting), gray
wireframe person boxes, ground rings, big ID labels, and each track's
PREVIOUS PATH drawn on the floor.  Detections are orange dots, the raw
scan is dark points.  Velocity arrows + Kalman prediction ghosts kept.

Views: Tracking 1 / Tracking 2 / side by side. Sliders retune live.
"""
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist

import people_avoidance.leg_detection as legmod
from people_avoidance.leg_detection import detect_legs, scan_to_cartesian, segment_scan
from people_avoidance.tracking import KalmanTracker
from people_avoidance.tracking_2 import GreedyNNTracker
from people_avoidance import controller as ctrl

TRAIL_LEN = 160          # ~20 s of path history per track at ~8 Hz

# Live-tunable parameters (sliders write here; the scan callback reads).
# Defaults = values tuned live in the lab (2026-06-10 session).
params = {
    # --- Stage 1 detection ---
    'dt': 0.30,      # segmentation gap floor (m)
    'lr': 0.20,      # leg radius (m)
    'mlw': 0.60,     # max leg-pair width (m)
    'circ': 0.80,    # circularity wall-reject
    'minpts': 12,    # min points per cluster
    'maxr': 2.0,     # display range filter (m)
    # --- Stage 2 tracking ---
    'gate': 5.40,    # Mahalanobis gate chi2 (tracking 1 only; T2 fixed 5.99)
    'q': 1.0,        # process noise spectral density (both trackers)
    'horizon': 3.0,  # prediction horizon (s)
    'vmove': 0.3,    # speed above which a track counts as "moving" (m/s)
    # --- Stage 3 controller (CBF-QP), run live on tracking-1 ---
    'cmaxv': 0.20,   # max_linear_speed (m/s)
    'cmaxw': 1.00,   # max_angular_speed (rad/s)
    'clook': 0.30,   # CBF lookahead L (m)
    'cgamma': 2.0,   # CBF class-K gain gamma
    'cwom': 0.10,    # QP weight on omega (steer-before-brake)
    'cpr': 0.30,     # person radius (m)
    'crr': 0.18,     # robot radius (m)
    'cinf': 3.0,     # influence radius — ignore people beyond (m)
    'cors': 2.0,     # obstacle_radius_scale (uncertainty inflation)
    'ctpred': 0.30,  # controller barrier prediction lookahead (s)
    'cdrive': 0.0,   # 1 = publish /cmd_vel to the robot, 0 = preview only
}

state = {
    'snapshot': {'points': [], 'legs': [], 'nseg': 0, 'ndet': 0, 'nscan': 0,
                 'dt_est': 0.0, 'cmd': {'v': 0.0, 'w': 0.0, 'drive': False},
                 't1': {'tracks': [], 'preds': {}, 'trails': {}},
                 't2': {'tracks': [], 'preds': {}, 'trails': {}}},
    'tracker1': None,
    'tracker2': None,
    'trails': {'t1': {}, 't2': {}},
    'last_stamp': None,
    'dt_est': 0.13,
    'cmd_pub': None,        # /cmd_vel publisher (set in ScanNode)
}

WIENER_Q = lambda q, dt: q * np.array(
    [[dt**3/3, 0, dt**2/2, 0],
     [0, dt**3/3, 0, dt**2/2],
     [dt**2/2, 0, dt, 0],
     [0, dt**2/2, 0, dt]])


def _stamp_seconds(scan):
    return scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9


def _ensure_trackers():
    """(Re)create both trackers when missing; live-retune gate/Q otherwise."""
    t1, t2 = state['tracker1'], state['tracker2']
    if t1 is None:
        t1 = KalmanTracker(dt=state['dt_est'],
                           process_noise_density=params['q'],
                           gate_chi2=params['gate'])
        state['tracker1'] = t1
    if t2 is None:
        t2 = GreedyNNTracker(dt=state['dt_est'],
                             process_noise_density=params['q'])
        state['tracker2'] = t2
    t1.gate_chi2 = params['gate']                 # T2 keeps its 95 % gate
    t1.Q = WIENER_Q(params['q'], t1.dt)
    t2.Q = WIENER_Q(params['q'], t2.dt)
    return t1, t2


def _tracker_snapshot(key, tracker, horizon):
    """Track rows + predictions + per-track path history (SPENCER trails)."""
    trails = state['trails'][key]
    rows, live = [], {}
    for trk in tracker.get_tracks():
        x, y, vx, vy = (float(v) for v in trk.m)
        rows.append({
            'id': trk.track_id, 'x': round(x, 3), 'y': round(y, 3),
            'vx': round(vx, 3), 'vy': round(vy, 3),
            'speed': round(math.hypot(vx, vy), 3),
            'conf': trk.confirmed, 'hits': trk.hits, 'misses': trk.misses,
            'static': bool(getattr(trk, 'static', False)),  # Module-4 persistence
        })
        hist = trails.get(trk.track_id, [])
        hist.append((round(x, 2), round(y, 2)))
        live[trk.track_id] = hist[-TRAIL_LEN:]
    state['trails'][key] = live                  # paths of dead tracks vanish
    preds = {str(tid): [[round(float(r[0]), 3), round(float(r[1]), 3),
                         round(float(r[2]), 3)] for r in arr]
             for tid, arr in tracker.predict_ahead(horizon).items()}
    trails_out = {str(tid): hist[::2] for tid, hist in live.items()}
    return {'tracks': rows, 'preds': preds, 'trails': trails_out}


def on_scan(scan):
    # dt estimate from scan stamps (EMA) — feeds new tracker instances.
    t = _stamp_seconds(scan)
    if state['last_stamp'] is not None:
        d = t - state['last_stamp']
        if 0.01 < d < 1.0:
            state['dt_est'] = 0.9 * state['dt_est'] + 0.1 * d
    state['last_stamp'] = t

    legmod.CIRCULARITY_MIN = params['circ']
    legmod.MIN_CLUSTER_POINTS = int(params['minpts'])

    pts = scan_to_cartesian(scan)
    segs = segment_scan(pts, distance_threshold=params['dt'],
                        angle_increment=scan.angle_increment) if pts.shape[0] else []
    dets = detect_legs(scan, distance_threshold=params['dt'],
                       leg_radius=params['lr'], max_leg_width=params['mlw'])

    t1, t2 = _ensure_trackers()
    t1.update(dets)
    t2.update(dets)

    point_rows, sid = [], 0
    for seg in segs:
        for q in seg:
            point_rows.append([round(float(q[0]), 3), round(float(q[1]), 3), sid])
        sid += 1
    leg_rows = [[round(m.x, 3), round(m.y, 3),
                 round(math.hypot(m.x, m.y), 3)] for m in dets]

    snap1 = _tracker_snapshot('t1', t1, params['horizon'])
    snap2 = _tracker_snapshot('t2', t2, params['horizon'])

    # ── Stage 3 controller, live on tracking-1's tracks ───────────────────
    # The robot is at the origin of the laser frame, so robot pose = (0,0,0)
    # and tracks are already robot-relative.  Drive-forward nominal (no goal)
    # → the CBF deflects around the detected people.
    ctrl.set_params(lookahead=params['clook'], gamma=params['cgamma'],
                    w_omega=params['cwom'], person_radius=params['cpr'],
                    robot_radius=params['crr'], influence=params['cinf'],
                    t_pred=params['ctpred'])
    ctrl.clear_goal()
    cmd = ctrl.compute_velocity(
        t1.get_tracks(), 0.0, 0.0, 0.0,
        max_linear_speed=params['cmaxv'], max_angular_speed=params['cmaxw'],
        obstacle_radius_scale=params['cors'])
    cmd_v, cmd_w = float(cmd.linear.x), float(cmd.angular.z)
    if params['cdrive'] >= 0.5 and state['cmd_pub'] is not None:
        state['cmd_pub'].publish(cmd)        # actually drive the robot

    nscan = state['snapshot']['nscan'] + 1
    state['snapshot'] = {
        'points': point_rows, 'legs': leg_rows, 'nseg': sid, 'ndet': len(dets),
        'nscan': nscan, 'dt_est': round(state['dt_est'], 3),
        't1': snap1, 't2': snap2,
        'cmd': {'v': round(cmd_v, 3), 'w': round(cmd_w, 3),
                'drive': params['cdrive'] >= 0.5},
    }


class ScanNode(Node):
    def __init__(self):
        super().__init__('leg_track_viz')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/scan', on_scan, qos)
        state['cmd_pub'] = self.create_publisher(Twist, '/cmd_vel', 10)


HTML = """<!doctype html><html><head><meta charset="utf-8"><title>People Tracking 3D — SPENCER style</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
body{margin:0;font-family:sans-serif;background:#0e0e0e;color:#eee;display:flex}
#plots{flex:1;display:flex;height:100vh;background:#fff}
.wrap{position:relative;flex:1;min-width:0;border-right:1px solid #ccc}
.wrap .plab{position:absolute;top:8px;left:12px;z-index:5;font-size:13px;color:#222;
  background:#ffffffcc;padding:3px 10px;border-radius:4px;pointer-events:none;border:1px solid #ddd}
.plotdiv{width:100%;height:100vh}
#panel{width:330px;padding:14px;background:#1a1a1a;overflow:auto;box-sizing:border-box}
h3{margin:0 0 4px} .row{margin:9px 0}
label{display:block;font-size:12px;margin-bottom:3px}
input[type=range]{width:100%} .val{color:#4cf;float:right}
#status{margin-top:12px;font-size:13px;line-height:1.7;border-top:1px solid #333;padding-top:8px}
.red{color:#ff5252}.org{color:#ffa726}.blu{color:#64b5f6}.grn{color:#69f0ae}
button{margin-top:10px;width:100%;padding:6px;background:#333;color:#eee;border:1px solid #555;cursor:pointer}
small{color:#888} .sec{margin-top:10px;border-top:1px solid #333;padding-top:6px;font-size:12px;color:#aaa}
.modes{display:flex;gap:4px;margin:8px 0}
.modes button{margin:0;flex:1;font-size:11px;padding:6px 2px}
.modes button.on{background:#2962ff;border-color:#2962ff}
</style></head><body>
<div id="plots">
 <div class="wrap" id="wrap1"><div class="plab">Tracking 1 — Hungarian + lifecycle</div><div id="plot1" class="plotdiv"></div></div>
 <div class="wrap" id="wrap2"><div class="plab">Tracking 2 — Greedy NN (notebook)</div><div id="plot2" class="plotdiv"></div></div>
</div>
<div id="panel">
<h3>People Tracking — live</h3><small>SPENCER-style · Stage 1 + two Stage 2 trackers</small>
<div class="sec">VIEW</div>
<div class="modes">
 <button id="m1" onclick="setMode(1)">Tracking 1</button>
 <button id="m2" onclick="setMode(2)">Tracking 2</button>
 <button id="m3" onclick="setMode(3)">Side by side</button>
</div>
<div class="sec">DETECTION</div>
<div class="row"><label>distance_threshold<span class="val" id="vdt"></span></label><input type=range id=dt min=0.02 max=0.50 step=0.005 value=0.30></div>
<div class="row"><label>leg_radius<span class="val" id="vlr"></span></label><input type=range id=lr min=0.03 max=0.35 step=0.005 value=0.20></div>
<div class="row"><label>max_leg_width<span class="val" id="vmlw"></span></label><input type=range id=mlw min=0.10 max=0.90 step=0.01 value=0.60></div>
<div class="row"><label>circularity_min<span class="val" id="vcirc"></span></label><input type=range id=circ min=0 max=0.95 step=0.01 value=0.80></div>
<div class="row"><label>min_points<span class="val" id="vminpts"></span></label><input type=range id=minpts min=2 max=25 step=1 value=12></div>
<div class="sec">TRACKING</div>
<div class="row"><label>gate χ² — Tracking 1 only<span class="val" id="vgate"></span></label><input type=range id=gate min=2 max=15 step=0.1 value=5.4></div>
<div class="row"><small>Tracking 2 gate is fixed at χ²₂(95%) = 5.99 (notebook)</small></div>
<div class="row"><label>process noise q (both)<span class="val" id="vq"></span></label><input type=range id=q min=0.1 max=5 step=0.1 value=1.0></div>
<div class="row"><label>prediction horizon (s)<span class="val" id="vhorizon"></span></label><input type=range id=horizon min=0.5 max=4 step=0.5 value=3></div>
<div class="row"><label>moving if speed > (m/s)<span class="val" id="vvmove"></span></label><input type=range id=vmove min=0.1 max=1.0 step=0.05 value=0.3></div>
<div class="sec">CONTROL — Stage 3 (CBF-QP, on Tracking 1)</div>
<div class="row"><label>max_linear_speed (m/s)<span class="val" id="vcmaxv"></span></label><input type=range id=cmaxv min=0.0 max=0.5 step=0.01 value=0.20></div>
<div class="row"><label>max_angular_speed (rad/s)<span class="val" id="vcmaxw"></span></label><input type=range id=cmaxw min=0.0 max=2.0 step=0.05 value=1.00></div>
<div class="row"><label>CBF lookahead L (m)<span class="val" id="vclook"></span></label><input type=range id=clook min=0.1 max=0.6 step=0.01 value=0.30></div>
<div class="row"><label>CBF gamma (γ)<span class="val" id="vcgamma"></span></label><input type=range id=cgamma min=0.3 max=8 step=0.1 value=2.0></div>
<div class="row"><label>QP ω-weight (steer↔brake)<span class="val" id="vcwom"></span></label><input type=range id=cwom min=0.02 max=1.0 step=0.02 value=0.10></div>
<div class="row"><label>person_radius (m)<span class="val" id="vcpr"></span></label><input type=range id=cpr min=0.1 max=0.6 step=0.02 value=0.30></div>
<div class="row"><label>robot_radius (m)<span class="val" id="vcrr"></span></label><input type=range id=crr min=0.1 max=0.4 step=0.01 value=0.18></div>
<div class="row"><label>influence radius (m)<span class="val" id="vcinf"></span></label><input type=range id=cinf min=1.0 max=5.0 step=0.1 value=3.0></div>
<div class="row"><label>obstacle_radius_scale (σ infl.)<span class="val" id="vcors"></span></label><input type=range id=cors min=0.0 max=4.0 step=0.1 value=2.0></div>
<div class="row"><label>barrier prediction t_pred (s)<span class="val" id="vctpred"></span></label><input type=range id=ctpred min=0.0 max=1.0 step=0.05 value=0.30></div>
<div class="row" style="background:#3a1010;padding:6px;border-radius:4px">
 <label><input type=checkbox id=cdrive onchange="vals()"> <b>Drive robot</b> (publish /cmd_vel)</label>
 <small>OFF = preview only. ON = the robot actually moves to avoid people.</small></div>
<div class="sec">DISPLAY</div>
<div class="row"><label>max_range view (m)<span class="val" id="vmaxr"></span></label><input type=range id=maxr min=0.5 max=8 step=0.1 value=2></div>
<button onclick="fetch('/reset')">Reset both trackers</button>
<div class="row"><label>Config preset</label>
<select id="preset" onchange="applyPreset()" style="width:100%;padding:6px;background:#222;color:#eee;border:1px solid #555">
 <option value="default">default (lab-tuned)</option>
 <option value="research">test-1-research</option>
</select></div>
<div id="status"></div>
</div>
<script>
var ids=['dt','lr','mlw','circ','minpts','maxr','gate','q','horizon','vmove',
         'cmaxv','cmaxw','clook','cgamma','cwom','cpr','crr','cinf','cors','ctpred'];
// Config presets. "default" = values tuned live in the lab. "research" =
// literature + on-robot measured values (RPLidar A1 @0.5deg/7.7Hz, noise
// 1-40mm by range; leg_tracker/Arras/SPENCER detector + KF tuning; SPENCER
// 99% gate; SPENCER 0.2-0.4 m/s moving cutoff). See PRESETS.md.
var CTRL={cmaxv:0.20,cmaxw:1.00,clook:0.30,cgamma:2.0,cwom:0.10,cpr:0.30,crr:0.18,cinf:3.0,cors:2.0,ctpred:0.30};
function withCtrl(o){var r={};for(var k in o)r[k]=o[k];for(var k in CTRL)r[k]=CTRL[k];return r;}
var PRESETS={
 'default':  withCtrl({dt:0.30, lr:0.20, mlw:0.60, circ:0.80, minpts:12, maxr:2.0, gate:5.40, q:1.0, horizon:3.0, vmove:0.30}),
 'research': withCtrl({dt:0.13, lr:0.06, mlw:0.45, circ:0.20, minpts:4,  maxr:4.0, gate:9.21, q:0.5, horizon:2.0, vmove:0.30})
};
function applyPreset(){
 var p=PRESETS[document.getElementById('preset').value]; if(!p) return;
 ids.forEach(function(i){ if(p[i]!==undefined){ document.getElementById(i).value=p[i]; }});
 vals();
}
function vals(){var o={};ids.forEach(function(i){o[i]=document.getElementById(i).value;document.getElementById('v'+i).textContent=parseFloat(o[i]).toFixed(i=='minpts'?0:2);});
 o['cdrive']=document.getElementById('cdrive').checked?1:0;return o;}
ids.forEach(function(i){document.getElementById(i).addEventListener('input',vals);});
var mode=3;
function setMode(m){
 mode=m;
 document.getElementById('wrap1').style.display=(m==1||m==3)?'block':'none';
 document.getElementById('wrap2').style.display=(m==2||m==3)?'block':'none';
 ['m1','m2','m3'].forEach(function(b,i){document.getElementById(b).className=(i+1==m)?'on':'';});
 setTimeout(function(){
   if(m==1||m==3) Plotly.Plots.resize('plot1');
   if(m==2||m==3) Plotly.Plots.resize('plot2');
 },60);
}
// SPENCER track palette: red / blue / salmon / light blue / dark red / pink
var PAL=['#e53935','#1e88e5','#ef9a9a','#64b5f6','#b71c1c','#f06292'];
function tcolor(id){return PAL[((id%PAL.length)+PAL.length)%PAL.length];}

// --- low-poly humanoid (lathed silhouette), built once -----------------
var RINGS=[[0.00,0.05],[0.06,0.085],[0.50,0.095],[0.95,0.135],[1.15,0.175],
           [1.30,0.19],[1.45,0.20],[1.52,0.085],[1.60,0.11],[1.72,0.105],[1.80,0.02]];
var SEG=10, UX=[],UY=[],UZ=[],UI=[],UJ=[],UK=[];
RINGS.forEach(function(rz){
  for(var s=0;s<SEG;s++){var a=2*Math.PI*s/SEG;
    UX.push(rz[1]*Math.cos(a));UY.push(rz[1]*Math.sin(a));UZ.push(rz[0]);}
});
for(var k=0;k<RINGS.length-1;k++){
  for(var s=0;s<SEG;s++){
    var a=k*SEG+s,b=k*SEG+(s+1)%SEG,c=(k+1)*SEG+s,d2=(k+1)*SEG+(s+1)%SEG;
    UI.push(a,b);UJ.push(b,d2);UK.push(c,c);
  }
}
function person(t,col,op){
  return {type:'mesh3d',
    x:UX.map(function(v){return v+t.x;}),
    y:UY.map(function(v){return v+t.y;}),
    z:UZ.slice(),
    i:UI,j:UJ,k:UK,color:col,opacity:op,flatshading:false,
    lighting:{ambient:0.6,diffuse:0.7,specular:0.15,roughness:0.6},
    lightposition:{x:50,y:80,z:200},hoverinfo:'skip',showlegend:false};
}
function pushBox(o,t){ // gray wireframe person box 0.6x0.6x1.9
  var w=0.3,h=1.9,x=t.x,y=t.y;
  var c=[[x-w,y-w],[x+w,y-w],[x+w,y+w],[x-w,y+w]];
  for(var i2=0;i2<4;i2++){var j2=(i2+1)%4;
    o.x.push(c[i2][0],c[j2][0],null);o.y.push(c[i2][1],c[j2][1],null);o.z.push(0,0,null);
    o.x.push(c[i2][0],c[j2][0],null);o.y.push(c[i2][1],c[j2][1],null);o.z.push(h,h,null);
    o.x.push(c[i2][0],c[i2][0],null);o.y.push(c[i2][1],c[i2][1],null);o.z.push(0,h,null);
  }
}
function pushRing(o,t,col){ // ground circle at the feet
  var R=0.32,N=22;
  for(var s=0;s<=N;s++){var a=2*Math.PI*s/N;
    o.x.push(t.x+R*Math.cos(a));o.y.push(t.y+R*Math.sin(a));o.z.push(0.02);o.c.push(col);}
  o.x.push(null);o.y.push(null);o.z.push(null);o.c.push(col);
}

var mkLayout=function(){return {paper_bgcolor:'#ffffff',uirevision:'keep',
 scene:{uirevision:'keep',
  xaxis:{range:[-6,6],showticklabels:false,title:{text:''},gridcolor:'#dcdcdc',zeroline:false,showbackground:false},
  yaxis:{range:[-6,6],showticklabels:false,title:{text:''},gridcolor:'#dcdcdc',zeroline:false,showbackground:false},
  zaxis:{range:[0,2.3],showticklabels:false,title:{text:''},showgrid:false,zeroline:false,showbackground:true,backgroundcolor:'#fafafa'},
  aspectmode:'manual',aspectratio:{x:1,y:1,z:0.30},bgcolor:'#ffffff',
  camera:{eye:{x:1.45,y:1.45,z:0.9}}},
 margin:{l:0,r:0,t:0,b:0},showlegend:false};};
var layouts={1:mkLayout(),2:mkLayout()};
var inited={1:false,2:false};

function sceneData(d, td, maxr, vmove, cmd){
  var data=[],stats={n:0,conf:0,move:0};
  // raw scan: dark structure points (SPENCER style)
  data.push({type:'scatter3d',mode:'markers',
    x:d.points.map(function(p){return p[0];}),y:d.points.map(function(p){return p[1];}),
    z:d.points.map(function(){return 0.01;}),
    marker:{size:1.8,color:'#3a3a3a',opacity:0.55},hoverinfo:'skip',showlegend:false});
  // detections: orange laser dots
  var legs=d.legs.filter(function(l){return l[2]<=maxr;});
  data.push({type:'scatter3d',mode:'markers',
    x:legs.map(function(l){return l[0];}),y:legs.map(function(l){return l[1];}),
    z:legs.map(function(){return 0.04;}),
    marker:{size:4,color:'#ff6d00'},hoverinfo:'skip',showlegend:false});

  var stems={x:[],y:[],z:[],c:[]};
  var boxes={x:[],y:[],z:[]};
  var rings={x:[],y:[],z:[],c:[]};
  var trail={x:[],y:[],z:[],c:[]};
  var labx=[],laby=[],labt=[],labc=[];
  var ax=[],ay=[],az=[];var px=[],py=[],pz=[];

  td.tracks.forEach(function(t){
    var r=Math.hypot(t.x,t.y); if(r>maxr) return;
    stats.n++;
    var moving=t.speed>vmove;
    if(t.conf)stats.conf++; if(t.conf&&moving)stats.move++;
    var col=tcolor(t.id);
    var op=(t.misses>0)?0.40:0.95;            // faded while coasting
    var scol=(t.misses>0)?'#c5c5c5':col;
    stems.x.push(t.x,t.x,null);stems.y.push(t.y,t.y,null);stems.z.push(0,1.8,null);
    stems.c.push(scol,scol,scol);
    pushBox(boxes,t);
    pushRing(rings,t,col);
    labx.push(t.x);laby.push(t.y);labt.push(String(t.id));labc.push(col);
    var tr=td.trails[String(t.id)]||[];        // previous path on the floor
    tr.forEach(function(p){trail.x.push(p[0]);trail.y.push(p[1]);trail.z.push(0.02);trail.c.push(col);});
    trail.x.push(null);trail.y.push(null);trail.z.push(null);trail.c.push(col);
    if(t.conf&&moving){
      ax.push(t.x,t.x+t.vx,null);ay.push(t.y,t.y+t.vy,null);az.push(0.95,0.95,null);
      var pr=td.preds[String(t.id)]||[];
      pr.forEach(function(p){px.push(p[0]);py.push(p[1]);pz.push(0.95);});
      px.push(null);py.push(null);pz.push(null);
    }
  });
  data.push({type:'scatter3d',mode:'lines',x:stems.x,y:stems.y,z:stems.z,
    line:{color:stems.c,width:8},hoverinfo:'skip',showlegend:false});
  data.push({type:'scatter3d',mode:'lines',x:trail.x,y:trail.y,z:trail.z,
    line:{color:trail.c,width:6},hoverinfo:'skip',showlegend:false});
  data.push({type:'scatter3d',mode:'lines',x:boxes.x,y:boxes.y,z:boxes.z,
    line:{color:'#9e9e9e',width:2},hoverinfo:'skip',showlegend:false});
  data.push({type:'scatter3d',mode:'lines',x:rings.x,y:rings.y,z:rings.z,
    line:{color:rings.c,width:5},hoverinfo:'skip',showlegend:false});
  data.push({type:'scatter3d',mode:'text',x:labx,y:laby,z:labx.map(function(){return 2.08;}),
    text:labt,textfont:{color:labc,size:17},hoverinfo:'skip',showlegend:false});
  data.push({type:'scatter3d',mode:'lines',x:ax,y:ay,z:az,
    line:{color:'#fbc02d',width:5},hoverinfo:'skip',showlegend:false});
  data.push({type:'scatter3d',mode:'markers',x:px,y:py,z:pz,
    marker:{size:2.6,color:'#8d6e63',opacity:0.5},hoverinfo:'skip',showlegend:false});
  // the robot at the origin
  data.push({type:'scatter3d',mode:'markers+text',x:[0],y:[0],z:[0.1],
    text:['ROBOT'],textfont:{color:'#1565c0',size:11},textposition:'bottom center',
    marker:{size:9,color:'#1565c0',symbol:'square'},hoverinfo:'skip',showlegend:false});
  // Stage-3 controller command arrow (only when cmd given — i.e. Tracking 1).
  if(cmd){
    var ang=cmd.w*0.6;                       // turn intent (rad)
    var len=0.35+cmd.v*2.0;                   // arrow length scales with v
    var hx=len*Math.cos(ang), hy=len*Math.sin(ang);
    var col=cmd.drive?'#00e676':'#9e9e9e';    // bright green = driving, gray = preview
    data.push({type:'scatter3d',mode:'lines',x:[0,hx],y:[0,hy],z:[0.14,0.14],
      line:{color:col,width:11},hoverinfo:'skip',showlegend:false});
    data.push({type:'scatter3d',mode:'markers',x:[hx],y:[hy],z:[0.14],
      marker:{size:7,color:col,symbol:'diamond'},hoverinfo:'skip',showlegend:false});
  }
  return {data:data,stats:stats};
}
function render(divId, n, d, td, maxr, vmove, cmd){
  var s=sceneData(d, td, maxr, vmove, cmd);
  if(!inited[n]){Plotly.newPlot(divId,s.data,layouts[n],{responsive:true});inited[n]=true;}
  else{Plotly.react(divId,s.data,layouts[n]);}
  return s.stats;
}
function tick(){
 var v=vals();var qs=ids.map(function(i){return i+'='+v[i];}).join('&')+'&cdrive='+v.cdrive;
 fetch('/data?'+qs).then(function(r){return r.json();}).then(function(d){
  var maxr=parseFloat(v.maxr), vmove=parseFloat(v.vmove);
  var s1=null,s2=null;
  if(mode==1||mode==3) s1=render('plot1',1,d,d.t1,maxr,vmove,d.cmd);  // cmd only on T1
  if(mode==2||mode==3) s2=render('plot2',2,d,d.t2,maxr,vmove,null);
  var h='scans: '+d.nscan+'  (dt≈'+d.dt_est+'s)<br>clusters: '+d.nseg+
        '  <span class="red">detections: '+d.ndet+'</span><br>';
  h+='<span class="grn">T1</span> tracks: '+d.t1.tracks.length;
  if(s1) h+=' — <span class="blu">conf '+s1.conf+'</span>, <span class="org">moving '+s1.move+'</span>';
  h+='<br><span class="grn">T2</span> tracks: '+d.t2.tracks.length;
  if(s2) h+=' — <span class="blu">in view '+s2.conf+'</span>, <span class="org">moving '+s2.move+'</span>';
  if(d.cmd){
    var dc=d.cmd.drive?'<span style="color:#00e676">DRIVING</span>':'<span style="color:#9e9e9e">preview</span>';
    h+='<br>CONTROL ['+dc+']: v='+d.cmd.v.toFixed(2)+' m/s  ω='+d.cmd.w.toFixed(2)+' rad/s';
  }
  document.getElementById('status').innerHTML=h;
 }).catch(function(){});
}
vals();setMode(3);setInterval(tick,250);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == '/data':
            q = parse_qs(u.query)
            for k in params:
                if k in q:
                    try:
                        params[k] = float(q[k][0])
                    except ValueError:
                        pass
            body = json.dumps(state['snapshot']).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(body)
        elif u.path == '/reset':
            state['tracker1'] = None
            state['tracker2'] = None
            state['trails'] = {'t1': {}, 't2': {}}
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'ok')
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML.encode())


def main():
    rclpy.init()
    node = ScanNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', 8080), Handler)
    print('Serving SPENCER-style dual-tracker viewer on http://0.0.0.0:8080')
    srv.serve_forever()


if __name__ == '__main__':
    main()
