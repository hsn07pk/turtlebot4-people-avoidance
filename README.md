# People Avoidance — Teaching Package

A ROS 2 Jazzy scaffold for a LiDAR-based people-avoidance pipeline on a
TurtleBot4.  Students implement four algorithm stages; the node wires them
together and runs safely (zero velocity) from the first `colcon build`.

---

## Get started

```bash
git clone <repo-url> ros2_ws
cd ros2_ws
colcon build --symlink-install
source install/setup.zsh
```

Then follow one of the guides below.

---

## Documentation

| Guide | Contents |
|-------|----------|
| [SIMULATION_SETUP.md](SIMULATION_SETUP.md) | Install ROS 2 Jazzy, build the workspace, launch the Gazebo simulation and pedestrian simulator — **native Ubuntu** and **Docker** |
| [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md) | Pipeline overview, data contract, per-stage implementation instructions, parameters, test commands |
| [REAL_ROBOT.md](REAL_ROBOT.md) | Deploy on a physical TurtleBot4: network setup, topic verification, parameter tuning, safety guidelines |

---

## Pipeline at a glance

```
/scan  (LaserScan · RPLidar A1 · ~5 Hz)
   │
   ▼  leg_detection.py          Stage 1 — segment scan, detect leg pairs
   │  List[LegMeasurement]      { x, y, Rxx, Rxy, Ryy }
   │
   ▼  tracking.py               Stage 2 — Kalman filter with data association
   │  List[Track]               { m=[x,y,vx,vy], P=(4×4) }
   │
   ▼  controller.py             Stage 3 — avoidance policy
   │  Twist                     { linear.x, angular.z }
   │
/cmd_vel
```

### Files to implement

| File | Function | What to fill in |
|------|----------|-----------------|
| `src/people_avoidance/people_avoidance/leg_detection.py` | Stage 1 | `segment_scan()` and `detect_legs()` |
| `src/people_avoidance/people_avoidance/tracking.py` | Stage 2 | `KalmanTracker` predict / associate / update |
| `src/people_avoidance/people_avoidance/controller.py` | Stage 3 | `obstacle_radius()` and `compute_velocity()` |

`people_avoidance_node.py` wires the stages — **do not edit it**.

---

## Two-terminal workflow

```bash
# Terminal 1 — simulation (Gazebo + 2 walking people)
ros2 launch pedestrian_sim simulation.launch.py

# Terminal 2 — your algorithm node
ros2 launch people_avoidance people_avoidance.launch.py
```

---

## Repository layout

```
.
├── README.md                    this file
├── SIMULATION_SETUP.md          native + Docker setup guide
├── DEVELOPMENT_GUIDE.md         algorithm implementation guide
├── REAL_ROBOT.md                deployment on physical TB4
│
├── docker/
│   ├── Dockerfile               complete environment (osrf/ros:jazzy-desktop base)
│   ├── docker-compose.yml       profiles: sim | avoidance | all | sim-gpu
│   └── entrypoint.sh
│
└── src/
    ├── people_avoidance/        ← students implement here
    │   ├── people_avoidance/
    │   │   ├── leg_detection.py       Stage 1  TODO
    │   │   ├── tracking.py            Stage 2  TODO
    │   │   ├── controller.py          Stage 3  TODO
    │   │   └── people_avoidance_node.py        (do not edit)
    │   └── launch/people_avoidance.launch.py
    │
    └── pedestrian_sim/          ← simulation environment (do not edit)
        ├── pedestrian_sim/pedestrian_sim_node.py
        ├── launch/
        │   ├── simulation.launch.py   full environment launch
        │   └── pedestrian_sim.launch.py
        └── worlds/
            └── simple_room.sdf        10×10 m room, no internet deps
```

---

## Docker quick start

```bash
# Allow host display access (Linux)
xhost +local:docker

# Build image once (~10 min first time)
docker build -f docker/Dockerfile -t people_avoidance:jazzy .

# Start simulator
docker compose -f docker/docker-compose.yml --profile sim up

# Start student node (separate terminal — auto-rebuilds on source changes)
docker compose -f docker/docker-compose.yml --profile avoidance up
```

---

## Notes for instructors

- **Simulation world**: `simple_room.sdf` is a 10×10 m dependency-free room.
  Use `world:=warehouse` for the TB4 warehouse environment (requires pre-cached
  Fuel models; the Docker image caches them at build time).

- **Pedestrian behaviour**: tunable at launch:
  ```bash
  ros2 launch pedestrian_sim simulation.launch.py num_people:=3 ped_speed:=0.3
  ```

- **Student parameters**: all thresholds and speed limits are ROS 2 declared
  parameters — override at launch without touching code.

- **Safe default**: all stubs return `[]` or zero Twist, so the robot stops
  safely before any stage is implemented.
