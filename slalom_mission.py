"""Slalom v4: waypoint chain through the corridor axis + docking mode.
W1=entrance -> W2=exit (pass between), W3=back-entrance -> W4=start (return)."""
import math, time, urllib.request
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import people_avoidance.leg_detection as legmod
from people_avoidance.leg_detection import detect_legs, LegMeasurement
from people_avoidance.tracking import KalmanTracker
from people_avoidance import controller as ctrl

LY = math.pi/2; CY, SY = math.cos(LY), math.sin(LY)
def rot(x, y): return (CY*x - SY*y, SY*x + CY*y)
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

legmod.MIN_CLUSTER_POINTS = 3
legmod.CIRCULARITY_MIN = 0.20
DET = dict(distance_threshold=0.13, leg_radius=0.06, max_leg_width=0.65)
VMAX, WMAX = 0.18, 0.7
L = 0.04; COVCAP = 0.02; RR = 0.17; MARGIN = 0.01; PR_FLOOR = 0.08
WALL_RANGE = 1.0

rclpy.init(); node = Node('mission')
qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT
st = {'scan': None, 'odom': None}
node.create_subscription(LaserScan, '/scan', lambda m: st.update(scan=m), qos)
node.create_subscription(Odometry, '/odom', lambda m: st.update(odom=m), qos)
pub = node.create_publisher(Twist, '/cmd_vel', 10)
def spin(dt=0.005): rclpy.spin_once(node, timeout_sec=dt)
def pose():
    o = st['odom'].pose.pose
    return o.position.x, o.position.y, yaw_of(o.orientation)

t0 = time.time()
while (st['scan'] is None or st['odom'] is None) and time.time()-t0 < 12: spin(0.1)
if st['scan'] is None or st['odom'] is None:
    print("ABORT: no scan/odom"); raise SystemExit
tracker = KalmanTracker(dt=0.13, process_noise_density=1.0, gate_chi2=9.21, max_misses=8)

def pipeline():
    s = st['scan']; st['scan'] = None
    rx, ry, rth = pose()
    c, s_ = math.cos(rth), math.sin(rth)
    odets = []
    for m in detect_legs(s, **DET):
        bx, by = rot(m.x, m.y)
        if math.hypot(bx, by) <= 0.30: continue
        ox = rx + c*bx - s_*by; oy = ry + s_*bx + c*by
        rxx = c*c*m.Rxx - 2*c*s_*m.Rxy + s_*s_*m.Ryy
        ryy = s_*s_*m.Rxx + 2*c*s_*m.Rxy + c*c*m.Ryy
        rxy = c*s_*m.Rxx + (c*c-s_*s_)*m.Rxy - c*s_*m.Ryy
        odets.append(LegMeasurement(x=ox, y=oy, Rxx=rxx, Rxy=rxy, Ryy=ryy))
    tracker.update(odets)
    tow = [t for t in tracker.get_tracks() if t.confirmed]
    class RT: __slots__=('m','P','track_id','confirmed','static','hits','misses')
    rts = []
    for t in tow:
        xo, yo, vxo, vyo = (float(v) for v in t.m)
        dx, dy = xo-rx, yo-ry
        r = RT(); r.m = np.array([c*dx+s_*dy, -s_*dx+c*dy, c*vxo+s_*vyo, -s_*vxo+c*vyo])
        r.P = t.P; r.track_id = t.track_id; r.confirmed = True
        r.static = bool(getattr(t, 'static', False)); r.hits=t.hits; r.misses=t.misses
        rts.append(r)
    ang = np.linspace(s.angle_min, s.angle_max, len(s.ranges))
    rng = np.asarray(s.ranges, float)
    ok = np.isfinite(rng) & (rng > 0.12) & (rng < WALL_RANGE)
    centers = [(float(r.m[0]), float(r.m[1])) for r in rts]
    best = {}
    for rr_, aa in zip(rng[ok], ang[ok]):
        x, y = rot(rr_*math.cos(aa), rr_*math.sin(aa))
        if any(math.hypot(x-cx, y-cy) < 0.45 for cx, cy in centers): continue
        sec = int((math.atan2(y, x)+math.pi)/(2*math.pi)*72)
        if sec not in best or rr_ < best[sec][2]: best[sec] = (x, y, rr_)
    return tow, rts, [(v[0], v[1]) for v in best.values()]

print("=== PHASE 0: scene lock (4 s) ===")
t0 = time.time()
while time.time()-t0 < 4:
    spin()
    if st['scan'] is not None: pipeline()
while st['scan'] is None: spin(0.05)
tow, rts, walls = pipeline()
rx, ry, rth = pose()
front = sorted([t for t, r in zip(tow, rts) if r.m[0] > 0.15 and math.hypot(r.m[0], r.m[1]) < 2.3],
               key=lambda t: math.hypot(float(t.m[0])-rx, float(t.m[1])-ry))
if len(front) < 2:
    print(f"ABORT: need 2 front people, found {len(front)}"); raise SystemExit
A, B = front[0], front[1]
AW = (float(A.m[0]), float(A.m[1])); BW = (float(B.m[0]), float(B.m[1]))
gap = math.hypot(AW[0]-BW[0], AW[1]-BW[1])
MW = ((AW[0]+BW[0])/2, (AW[1]+BW[1])/2)
print(f"A=({AW[0]:+.2f},{AW[1]:+.2f})s={getattr(A,'static',0)} B=({BW[0]:+.2f},{BW[1]:+.2f})s={getattr(B,'static',0)} GAP={gap:.2f}")
pr = gap/2 - RR - COVCAP - L - MARGIN
if pr < PR_FLOOR:
    print(f"ABORT: gap {gap:.2f} < passable {2*(PR_FLOOR+RR+COVCAP+L+MARGIN):.2f}"); raise SystemExit
pr = min(pr, 0.25)
ux, uy = -(BW[1]-AW[1]), (BW[0]-AW[0])
un = math.hypot(ux, uy); ux, uy = ux/un, uy/un
if ux*(MW[0]-rx) + uy*(MW[1]-ry) < 0: ux, uy = -ux, -uy
W1 = (MW[0]-0.55*ux, MW[1]-0.55*uy)     # front entrance
W2 = (MW[0]+0.85*ux, MW[1]+0.85*uy)     # exit (beyond the people)
W3 = (MW[0]+0.55*ux, MW[1]+0.55*uy)     # back entrance (return)
START = (rx, ry)
print(f"person_radius={pr:.2f}  W1=({W1[0]:+.2f},{W1[1]:+.2f}) W2=({W2[0]:+.2f},{W2[1]:+.2f}) start=({rx:+.2f},{ry:+.2f})")
ctrl.set_params(lookahead=L, gamma=3.0, w_omega=0.10, person_radius=pr,
                wall_radius=0.02, robot_radius=RR, influence=WALL_RANGE, t_pred=0.2,
                cors_cap=COVCAP, gap_clear=0.02, gap_hyst=0.6,
                backoff_trigger=0.20, backoff_clear=0.40, backoff_speed=0.08,
                backoff_rear_min=0.45, escape_dist=0.55, escape_v_frac=0.12,
                goal_tolerance=0.25, slow_floor=0.5, dock_dist=0.8)

def show_goal(g):
    try:
        x, y, th = pose(); c, s_ = math.cos(th), math.sin(th)
        dx, dy = g[0]-x, g[1]-y
        urllib.request.urlopen(f"http://localhost:8080/goal?gx={c*dx+s_*dy:.2f}&gy={-s_*dx+c*dy:.2f}", timeout=1)
    except Exception: pass

def goto(name, goal, tol, timeout):
    ctrl.set_goal(*goal); ctrl.set_params(goal_tolerance=tol); show_goal(goal)
    t0 = time.time(); last_pub = last_log = 0.0
    cmd = Twist(); minA = minB = 1e9
    while time.time()-t0 < timeout:
        spin()
        if st['scan'] is not None:
            tow, rts, walls = pipeline()
            x, y, th = pose()
            dA = math.hypot(AW[0]-x, AW[1]-y); dB = math.hypot(BW[0]-x, BW[1]-y)
            minA, minB = min(minA, dA), min(minB, dB)
            if dA < 0.26 or dB < 0.26:
                print(f"  !! SAFETY STOP (dA={dA:.2f} dB={dB:.2f})")
                for _ in range(20): pub.publish(Twist()); time.sleep(0.02)
                return False, minA, minB
            cmd = ctrl.compute_velocity(rts, x, y, th, VMAX, WMAX, 1.0, obstacle_points=walls)
        now = time.time()
        if now-last_pub >= 0.02: pub.publish(cmd); last_pub = now
        if now-last_log >= 1.5:
            x, y, th = pose()
            print(f"  [{name} {now-t0:4.1f}s] ({x:+.2f},{y:+.2f},{math.degrees(th):+4.0f}) goal={math.hypot(goal[0]-x,goal[1]-y):.2f} cmd=({cmd.linear.x:+.2f},{cmd.angular.z:+.2f})")
            last_log = now
        x, y, th = pose()
        if math.hypot(goal[0]-x, goal[1]-y) < tol:
            for _ in range(5): pub.publish(Twist()); time.sleep(0.02)
            return True, minA, minB
    for _ in range(15): pub.publish(Twist()); time.sleep(0.02)
    return False, minA, minB

mins = []
print("=== PASS: W1 (entrance) ===");  ok1, a, b = goto("W1", W1, 0.30, 60); mins += [a, b]
print(f"  W1 {'ok' if ok1 else 'TIMEOUT'}")
ok2 = False
if ok1:
    print("=== PASS: W2 (through the gap) ===")
    ok2, a, b = goto("W2", W2, 0.30, 60); mins += [a, b]
    print(f"  W2 {'ok — PASSED BETWEEN ✅' if ok2 else 'TIMEOUT'}")
print("=== RETURN: W3 (back entrance) ===")
ok3, a, b = goto("W3", W3, 0.35, 50); mins += [a, b]
print("=== RETURN: start ===")
ok4, a, b = goto("HOME", START, 0.25, 80); mins += [a, b]
x, y, th = pose()
print(f"=== RESULT: pass={'YES' if ok2 else 'no'} return={'YES' if ok4 else 'no'} "
      f"final_err={math.hypot(START[0]-x,START[1]-y):.2f} m  closest_person={min(mins):.2f} m ===")
node.destroy_node(); rclpy.shutdown()
