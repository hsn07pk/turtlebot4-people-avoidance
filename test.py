#!/usr/bin/env python3
"""
Real-time leg-detection 3D visualizer + threshold tuner.
A tiny web server (built-in libs only) serves a 3D room map; open it in your
Mac browser. Sliders re-run detect_legs() live. Legs = red columns; other
objects = coloured by cluster.
"""
import json, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import people_avoidance.leg_detection as legmod
from people_avoidance.leg_detection import detect_legs, scan_to_cartesian, segment_scan

latest = {'scan': None}

class ScanNode(Node):
    def __init__(self):
        super().__init__('leg_viz')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/scan', lambda s: latest.__setitem__('scan', s), qos)

def compute(p):
    scan = latest['scan']
    if scan is None:
        return {'points': [], 'legs': [], 'nseg': 0, 'ntotal': 0, 'nkept': 0}
    legmod.CIRCULARITY_MIN = p['circ']
    legmod.MIN_CLUSTER_POINTS = int(p['minpts'])
    pts = scan_to_cartesian(scan)
    segs = segment_scan(pts, distance_threshold=p['dt'], angle_increment=scan.angle_increment) if pts.shape[0] else []
    point_rows, sid = [], 0
    for seg in segs:
        for q in seg:
            point_rows.append([round(float(q[0]), 3), round(float(q[1]), 3), sid])
        sid += 1
    legs = detect_legs(scan, distance_threshold=p['dt'], leg_radius=p['lr'], max_leg_width=p['mlw'])
    leg_rows = []
    for m in legs:
        r = (m.x**2 + m.y**2) ** 0.5
        if r <= p['maxr']:
            leg_rows.append([round(m.x, 3), round(m.y, 3), round(r, 3)])
    return {'points': point_rows, 'legs': leg_rows, 'nseg': sid,
            'ntotal': len(legs), 'nkept': len(leg_rows)}

HTML = """<!doctype html><html><head><meta charset="utf-8"><title>Leg Detection 3D Tuner</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
body{margin:0;font-family:sans-serif;background:#0e0e0e;color:#eee;display:flex}
#plot{flex:1;height:100vh}
#panel{width:330px;padding:16px;background:#1a1a1a;overflow:auto;box-sizing:border-box}
h3{margin:0 0 6px} .row{margin:12px 0}
label{display:block;font-size:13px;margin-bottom:4px}
input[type=range]{width:100%} .val{color:#4cf;float:right}
#status{margin-top:16px;font-size:13px;line-height:1.7;border-top:1px solid #333;padding-top:10px}
.red{color:#ff5252;font-weight:bold} small{color:#888}
</style></head><body>
<div id="plot"></div>
<div id="panel">
<h3>Leg Detection — live tuner</h3><small>drag to rotate the 3D map</small>
<div class="row"><label>distance_threshold (seg gap)<span class="val" id="vdt"></span></label><input type=range id=dt min=0.02 max=0.30 step=0.005 value=0.10></div>
<div class="row"><label>leg_radius<span class="val" id="vlr"></span></label><input type=range id=lr min=0.03 max=0.20 step=0.005 value=0.10></div>
<div class="row"><label>max_leg_width (pair dist)<span class="val" id="vmlw"></span></label><input type=range id=mlw min=0.10 max=0.60 step=0.01 value=0.25></div>
<div class="row"><label>circularity_min (wall reject)<span class="val" id="vcirc"></span></label><input type=range id=circ min=0 max=0.80 step=0.01 value=0.15></div>
<div class="row"><label>min_points<span class="val" id="vminpts"></span></label><input type=range id=minpts min=2 max=12 step=1 value=2></div>
<div class="row"><label>max_range (view filter)<span class="val" id="vmaxr"></span></label><input type=range id=maxr min=0.5 max=8 step=0.1 value=8></div>
<div id="status"></div>
</div>
<script>
var ids=['dt','lr','mlw','circ','minpts','maxr'];
function vals(){var o={};ids.forEach(function(i){o[i]=document.getElementById(i).value;document.getElementById('v'+i).textContent=parseFloat(o[i]).toFixed(i=='minpts'?0:3);});return o;}
ids.forEach(function(i){document.getElementById(i).addEventListener('input',vals);});
var layout={paper_bgcolor:'#0e0e0e',scene:{xaxis:{title:'x fwd (m)',color:'#888',range:[-6,6]},yaxis:{title:'y left (m)',color:'#888',range:[-6,6]},zaxis:{title:'height (m)',color:'#888',range:[0,2]},aspectmode:'manual',aspectratio:{x:1,y:1,z:0.35},bgcolor:'#0e0e0e'},margin:{l:0,r:0,t:0,b:0},showlegend:true,legend:{font:{color:'#eee'},x:0,y:1}};
var inited=false;
function tick(){
 var v=vals();var qs=ids.map(function(i){return i+'='+v[i];}).join('&');
 fetch('/data?'+qs).then(function(r){return r.json();}).then(function(d){
  var px=d.points.map(function(p){return p[0];}),py=d.points.map(function(p){return p[1];}),pz=d.points.map(function(){return 0;}),pc=d.points.map(function(p){return p[2];});
  var pts={type:'scatter3d',mode:'markers',name:'objects ('+d.nseg+')',x:px,y:py,z:pz,marker:{size:2.5,color:pc,colorscale:'Rainbow',opacity:0.85}};
  var lx=[],ly=[],lz=[];d.legs.forEach(function(l){lx.push(l[0],l[0],null);ly.push(l[1],l[1],null);lz.push(0,1.6,null);});
  var stems={type:'scatter3d',mode:'lines',x:lx,y:ly,z:lz,line:{color:'#ff2222',width:6},showlegend:false,hoverinfo:'skip'};
  var heads={type:'scatter3d',mode:'markers',name:'LEGS (red)',x:d.legs.map(function(l){return l[0];}),y:d.legs.map(function(l){return l[1];}),z:d.legs.map(function(){return 1.6;}),marker:{size:7,color:'#ff2222'},text:d.legs.map(function(l){return 'r='+l[2]+'m';}),hoverinfo:'text'};
  var robot={type:'scatter3d',mode:'markers',name:'robot',x:[0],y:[0],z:[0],marker:{size:7,color:'#33ff88',symbol:'diamond'}};
  var data=[pts,stems,heads,robot];
  if(!inited){Plotly.newPlot('plot',data,layout,{responsive:true});inited=true;}else{Plotly.react('plot',data,layout);}
  document.getElementById('status').innerHTML='objects (clusters): '+d.nseg+'<br>scan points: '+d.points.length+'<br>total leg detections: '+d.ntotal+'<br><span class="red">shown (≤ max_range): '+d.nkept+'</span>';
 }).catch(function(){});
}
vals();setInterval(tick,250);
</script></body></html>"""

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        u = urlparse(self.path)
        if u.path == '/data':
            q = parse_qs(u.query)
            g = lambda k, d: float(q.get(k, [d])[0])
            p = dict(dt=g('dt',0.1), lr=g('lr',0.10), mlw=g('mlw',0.25),
                     circ=g('circ',0.15), minpts=g('minpts',2), maxr=g('maxr',8.0))
            body = json.dumps(compute(p)).encode()
            self.send_response(200); self.send_header('Content-Type','application/json'); self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(200); self.send_header('Content-Type','text/html'); self.end_headers()
            self.wfile.write(HTML.encode())

def main():
    rclpy.init()
    node = ScanNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', 8080), Handler)
    print("Serving leg-detection 3D tuner on http://0.0.0.0:8080")
    srv.serve_forever()

if __name__ == '__main__':
    main()
