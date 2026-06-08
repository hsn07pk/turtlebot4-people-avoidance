# People Avoidance — ROS 2 Teaching Package

A ROS 2 Jazzy scaffold for a LiDAR-based people-avoidance pipeline.
You implement three algorithm stages; your code runs on a physical **TurtleBot4**
during lab sessions.

---

## What you build

```
/scan  (LaserScan · RPLidar A1 · ~5 Hz)
   │
   ▼  leg_detection.py     Stage 1 — segment scan, detect leg pairs
   │  List[LegMeasurement] { x, y, Rxx, Rxy, Ryy }
   │
   ▼  tracking.py          Stage 2 — Kalman filter + data association
   │  List[Track]          { m=[x,y,vx,vy], P=(4×4) }
   │
   ▼  controller.py        Stage 3 — avoidance policy
   │  Twist                { linear.x, angular.z }
   │
/cmd_vel  →  TurtleBot4 moves
```

Three files to implement, in order:

| Stage | File | Function |
|-------|------|----------|
| 1 | `src/people_avoidance/people_avoidance/leg_detection.py` | Detect people from LiDAR |
| 2 | `src/people_avoidance/people_avoidance/tracking.py` | Track them over time |
| 3 | `src/people_avoidance/people_avoidance/controller.py` | Avoid them |

`people_avoidance_node.py` wires the stages — **do not edit it**.

---

## Documentation

| Document | Read when |
|----------|-----------|
| [SETUP.md](SETUP.md) | First session — install ROS 2, clone and build the workspace |
| [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md) | Implementing the algorithm — data contract, equations, hints |
| [REAL_ROBOT.md](REAL_ROBOT.md) | Lab session — connect to the TB4, run your node, tune parameters |
| [SIMULATION.md](SIMULATION.md) | Optional — test your algorithm offline when no robot is available |
| [DOCKER.md](DOCKER.md) | Optional — run everything inside Docker (any OS) |

---

## Quick reference

**Lab session (real robot):**
```bash
source ~/ros2_ws/install/setup.zsh
ros2 launch people_avoidance people_avoidance.launch.py
```

**Offline testing (simulation):**
```bash
ros2 launch pedestrian_sim simulation.launch.py          # terminal 1
ros2 launch people_avoidance people_avoidance.launch.py  # terminal 2
```

---

## Repository layout

```
src/
├── people_avoidance/           ← implement here
│   └── people_avoidance/
│       ├── leg_detection.py    Stage 1  TODO
│       ├── tracking.py         Stage 2  TODO
│       ├── controller.py       Stage 3  TODO
│       └── people_avoidance_node.py     (do not edit)
└── pedestrian_sim/             simulation tool (do not edit)
    ├── launch/simulation.launch.py
    └── worlds/simple_room.sdf
```
