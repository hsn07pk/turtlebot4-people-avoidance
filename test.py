#!/usr/bin/env python3
"""
Live people-tracking 3D visualizer + tuner (Stage 1 + Stage 2).

A tiny web server (built-in libs only) serves a 3D room map; open it in a
browser on the same network. The full pipeline runs on every /scan:

    detect_legs()  ->  KalmanTracker.update()  ->  predict_ahead()

Display: scan points coloured by cluster, leg detections in red, Kalman
tracks as labelled columns (orange = moving person, blue = static), velocity
arrows, and a dashed "ghost" trail showing each moving track's predicted
future positions (Kalman prediction iterated over the chosen horizon).
Sliders retune detection and tracking parameters live.
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

import people_avoidance.leg_detection as legmod
from people_avoidance.leg_detection import detect_legs, scan_to_cartesian, segment_scan
from people_avoidance.tracking import KalmanTracker

# Live-tunable parameters (sliders write here; the scan callback reads).
# Defaults = values tuned live in the lab (2026-06-10 session).
params = {
    'dt': 0.30,      # segmentation gap floor (m)
    'lr': 0.20,      # leg radius (m)
    'mlw': 0.60,     # max leg-pair width (m)
    'circ': 0.80,    # circularity wall-reject
    'minpts': 12,    # min points per cluster
    'maxr': 2.0,     # display range filter (m)
    'gate': 5.40,    # Mahalanobis gate chi2 (tracker)
    'q': 1.0,        # process noise spectral density (tracker)
    'horizon': 3.0,  # prediction horizon (s)
    'vmove': 0.3,    # speed above which a track counts as "moving" (m/s)
}

state = {
    'snapshot': {'points': [], 'legs': [], 'tracks': [], 'preds': {},
                 'nseg': 0, 'ndet': 0, 'nscan': 0, 'dt_est': 0.0},
    'tracker': None,
    'last_stamp': None,
    'dt_est': 0.13,
}


def _stamp_seconds(scan):
    return scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9


def _ensure_tracker():
    """(Re)create the tracker when missing; live-retune gate/Q otherwise."""
    tr = state['tracker']
    if tr is None:
        tr = KalmanTracker(dt=state['dt_est'],
                           process_noise_density=params['q'],
                           gate_chi2=params['gate'])
        state['tracker'] = tr
        return tr
    tr.gate_chi2 = params['gate']
    dt, qd = tr.dt, params['q']
    tr.Q = qd * np.array(
        [[dt**3/3, 0, dt**2/2, 0],
         [0, dt**3/3, 0, dt**2/2],
         [dt**2/2, 0, dt, 0],
         [0, dt**2/2, 0, dt]])
    return tr


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

    tracker = _ensure_tracker()
    tracker.update(dets)
    preds = tracker.predict_ahead(params['horizon'])

    point_rows, sid = [], 0
    for seg in segs:
        for q in seg:
            point_rows.append([round(float(q[0]), 3), round(float(q[1]), 3), sid])
        sid += 1

    leg_rows = [[round(m.x, 3), round(m.y, 3),
                 round(math.hypot(m.x, m.y), 3)] for m in dets]

    track_rows = []
    for trk in tracker.get_tracks():
        x, y, vx, vy = (float(v) for v in trk.m)
        track_rows.append({
            'id': trk.track_id, 'x': round(x, 3), 'y': round(y, 3),
            'vx': round(vx, 3), 'vy': round(vy, 3),
            'speed': round(math.hypot(vx, vy), 3),
            'conf': trk.confirmed, 'hits': trk.hits, 'misses': trk.misses,
        })

    pred_rows = {str(tid): [[round(float(r[0]), 3), round(float(r[1]), 3),
                             round(float(r[2]), 3)] for r in arr]
                 for tid, arr in preds.items()}

    nscan = state['snapshot']['nscan'] + 1
    state['snapshot'] = {
        'points': point_rows, 'legs': leg_rows, 'tracks': track_rows,
        'preds': pred_rows, 'nseg': sid, 'ndet': len(dets),
        'nscan': nscan, 'dt_est': round(state['dt_est'], 3),
    }


class ScanNode(Node):
    def __init__(self):
        super().__init__('leg_track_viz')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/scan', on_scan, qos)


HTML = """<!doctype html><html><head><meta charset="utf-8"><title>People Tracking 3D</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
body{margin:0;font-family:sans-serif;background:#0e0e0e;color:#eee;display:flex}
#plot{flex:1;height:100vh}
#panel{width:330px;padding:14px;background:#1a1a1a;overflow:auto;box-sizing:border-box}
h3{margin:0 0 4px} .row{margin:9px 0}
label{display:block;font-size:12px;margin-bottom:3px}
input[type=range]{width:100%} .val{color:#4cf;float:right}
#status{margin-top:12px;font-size:13px;line-height:1.7;border-top:1px solid #333;padding-top:8px}
.red{color:#ff5252}.org{color:#ffa726}.blu{color:#64b5f6}
button{margin-top:10px;width:100%;padding:6px;background:#333;color:#eee;border:1px solid #555;cursor:pointer}
small{color:#888} .sec{margin-top:10px;border-top:1px solid #333;padding-top:6px;font-size:12px;color:#aaa}
</style></head><body>
<div id="plot"></div>
<div id="panel">
<h3>People Tracking — live</h3><small>Stage 1 detection + Stage 2 Kalman tracks</small>
<div class="sec">DETECTION</div>
<div class="row"><label>distance_threshold<span class="val" id="vdt"></span></label><input type=range id=dt min=0.02 max=0.50 step=0.005 value=0.30></div>
<div class="row"><label>leg_radius<span class="val" id="vlr"></span></label><input type=range id=lr min=0.03 max=0.35 step=0.005 value=0.20></div>
<div class="row"><label>max_leg_width<span class="val" id="vmlw"></span></label><input type=range id=mlw min=0.10 max=0.90 step=0.01 value=0.60></div>
<div class="row"><label>circularity_min<span class="val" id="vcirc"></span></label><input type=range id=circ min=0 max=0.95 step=0.01 value=0.80></div>
<div class="row"><label>min_points<span class="val" id="vminpts"></span></label><input type=range id=minpts min=2 max=25 step=1 value=12></div>
<div class="sec">TRACKING (Kalman)</div>
<div class="row"><label>gate χ² (Mahalanobis)<span class="val" id="vgate"></span></label><input type=range id=gate min=2 max=15 step=0.1 value=5.4></div>
<div class="row"><label>process noise q<span class="val" id="vq"></span></label><input type=range id=q min=0.1 max=5 step=0.1 value=1.0></div>
<div class="row"><label>prediction horizon (s)<span class="val" id="vhorizon"></span></label><input type=range id=horizon min=0.5 max=4 step=0.5 value=3></div>
<div class="row"><label>moving if speed > (m/s)<span class="val" id="vvmove"></span></label><input type=range id=vmove min=0.1 max=1.0 step=0.05 value=0.3></div>
<div class="sec">DISPLAY</div>
<div class="row"><label>max_range view (m)<span class="val" id="vmaxr"></span></label><input type=range id=maxr min=0.5 max=8 step=0.1 value=2></div>
<button onclick="fetch('/reset')">Reset tracker</button>
<div id="status"></div>
</div>
<script>
var ids=['dt','lr','mlw','circ','minpts','maxr','gate','q','horizon','vmove'];
function vals(){var o={};ids.forEach(function(i){o[i]=document.getElementById(i).value;document.getElementById('v'+i).textContent=parseFloat(o[i]).toFixed(i=='minpts'?0:2);});return o;}
ids.forEach(function(i){document.getElementById(i).addEventListener('input',vals);});
var layout={paper_bgcolor:'#0e0e0e',scene:{xaxis:{title:'x fwd (m)',color:'#888',range:[-6,6]},yaxis:{title:'y left (m)',color:'#888',range:[-6,6]},zaxis:{title:'z (m)',color:'#888',range:[0,2]},aspectmode:'manual',aspectratio:{x:1,y:1,z:0.35},bgcolor:'#0e0e0e'},margin:{l:0,r:0,t:0,b:0},showlegend:true,legend:{font:{color:'#eee'},x:0,y:1}};
var inited=false;
function tick(){
 var v=vals();var qs=ids.map(function(i){return i+'='+v[i];}).join('&');
 fetch('/data?'+qs).then(function(r){return r.json();}).then(function(d){
  var maxr=parseFloat(v.maxr), vmove=parseFloat(v.vmove);
  var pts={type:'scatter3d',mode:'markers',name:'objects ('+d.nseg+')',
    x:d.points.map(function(p){return p[0];}),y:d.points.map(function(p){return p[1];}),z:d.points.map(function(){return 0;}),
    marker:{size:2.2,color:d.points.map(function(p){return p[2];}),colorscale:'Rainbow',opacity:0.75}};
  var legs=d.legs.filter(function(l){return l[2]<=maxr;});
  var legT={type:'scatter3d',mode:'markers',name:'detections (red)',
    x:legs.map(function(l){return l[0];}),y:legs.map(function(l){return l[1];}),z:legs.map(function(){return 0.05;}),
    marker:{size:4.5,color:'#ff2222'}};
  var tx=[],ty=[],tz=[],tcol=[],labx=[],laby=[],labt=[],labc=[];
  var ax=[],ay=[],az=[];var px=[],py=[],pz=[];
  var nconf=0,nmove=0;
  d.tracks.forEach(function(t){
    var r=Math.hypot(t.x,t.y); if(r>maxr) return;
    var moving=t.speed>vmove;
    if(t.conf)nconf++; if(t.conf&&moving)nmove++;
    var col=t.conf?(moving?'#ffa726':'#64b5f6'):'#777777';
    tx.push(t.x,t.x,null);ty.push(t.y,t.y,null);tz.push(0,1.7,null);tcol.push(col,col,col);
    labx.push(t.x);laby.push(t.y);labt.push('#'+t.id+(moving?' '+t.speed.toFixed(1)+'m/s':''));labc.push(col);
    if(t.conf&&moving){
      ax.push(t.x,t.x+t.vx,null);ay.push(t.y,t.y+t.vy,null);az.push(0.9,0.9,null);
      var pr=d.preds[String(t.id)]||[];
      pr.forEach(function(p){px.push(p[0]);py.push(p[1]);pz.push(0.9);});
      px.push(null);py.push(null);pz.push(null);
    }
  });
  var stems={type:'scatter3d',mode:'lines',name:'tracks',x:tx,y:ty,z:tz,line:{color:tcol,width:7},hoverinfo:'skip'};
  var heads={type:'scatter3d',mode:'markers+text',showlegend:false,x:labx,y:laby,z:labx.map(function(){return 1.85;}),
    marker:{size:5,color:labc},text:labt,textfont:{color:'#eee',size:11},textposition:'top center'};
  var arrows={type:'scatter3d',mode:'lines',name:'velocity (1s)',x:ax,y:ay,z:az,line:{color:'#ffee58',width:5},hoverinfo:'skip'};
  var ghost={type:'scatter3d',mode:'markers',name:'prediction ghosts',x:px,y:py,z:pz,
    marker:{size:3,color:'#ff8a65',opacity:0.55},hoverinfo:'skip'};
  var robot={type:'scatter3d',mode:'markers',name:'robot',x:[0],y:[0],z:[0],marker:{size:7,color:'#33ff88',symbol:'diamond'}};
  var data=[pts,legT,stems,heads,arrows,ghost,robot];
  if(!inited){Plotly.newPlot('plot',data,layout,{responsive:true});inited=true;}else{Plotly.react('plot',data,layout);}
  document.getElementById('status').innerHTML=
    'scans: '+d.nscan+'  (dt≈'+d.dt_est+'s)<br>clusters: '+d.nseg+'<br>'+
    '<span class="red">detections: '+d.ndet+'</span><br>'+
    'tracks: '+d.tracks.length+' — <span class="blu">confirmed: '+nconf+'</span>, '+
    '<span class="org">moving: '+nmove+'</span>';
 }).catch(function(){});
}
vals();setInterval(tick,250);
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
            state['tracker'] = None
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
    print('Serving people-tracking 3D viewer on http://0.0.0.0:8080')
    srv.serve_forever()


if __name__ == '__main__':
    main()
