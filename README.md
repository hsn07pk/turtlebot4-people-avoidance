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
| [SETUP.md](SETUP.md) | **Step 1** — install ROS 2, clone and build the workspace |
| [TURTLEBOT4_GUIDE.md](TURTLEBOT4_GUIDE.md) | **Step 2** — platform intro: drive the robot, SLAM, Nav2, rosbag |
| [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md) | **Step 3** — implement the algorithm: data contract, equations, hints |
| [REAL_ROBOT.md](REAL_ROBOT.md) | **Step 4** — run your node on the TB4, tune parameters, safety |
| [SIMULATION.md](SIMULATION.md) | Optional — test offline when no robot is available |
| [DOCKER.md](DOCKER.md) | Optional — run everything inside Docker (any OS) |

---

## Quick reference

**Lab session (real robot):**
```bash
source ~/ros2_ws/install/setup.zsh    # zsh  (Ubuntu 24.04 / Jazzy)
# source ~/ros2_ws/install/setup.bash # bash (Ubuntu 22.04 / Humble)
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
