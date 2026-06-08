# Simulation — Optional Offline Testing Tool

The Gazebo simulation lets you test your algorithm **without access to the robot**.
Use it to verify your code is working before the lab session, or to experiment
with parameters at home.

It is not a replacement for the real robot — real sensor noise, leg geometry,
and robot dynamics will differ.

---

## What it provides

A 10 × 10 m room with the TurtleBot4 and two (or more) simulated "people"
— cylinders that move with a random walk, producing realistic two-blob LiDAR
patterns for your leg detector to find.

```
Simulated person = two 0.10 m radius cylinders, 0.25 m apart
→ produces the same leg-pair pattern as real legs in /scan
```

---

## Additional packages required

The simulation requires the TB4 Gazebo stack, which is not needed for real-robot
work.  Install once:

```bash
sudo apt install -y ros-jazzy-turtlebot4-simulator
```

---

## Launch

```bash
# Terminal 1 — Gazebo + TurtleBot4 + pedestrians
conda deactivate
ros2 launch pedestrian_sim simulation.launch.py

# Terminal 2 — your algorithm (exactly the same command as on the real robot)
conda deactivate
ros2 launch people_avoidance people_avoidance.launch.py
```

Wait ~25 seconds after launch for the robot and pedestrians to appear.

---

## Options

```bash
# More pedestrians
ros2 launch pedestrian_sim simulation.launch.py num_people:=3

# Slower (easier to track while you debug)
ros2 launch pedestrian_sim simulation.launch.py ped_speed:=0.3

# TB4 warehouse environment instead of the simple room
ros2 launch pedestrian_sim simulation.launch.py world:=warehouse
```

---

## Verify it is working

```bash
# Should show varied ranges, not all 0.164
ros2 topic echo --once /scan | grep -A5 "ranges:"

# Should publish at ~5 Hz
ros2 topic hz /scan
```

---

## Docker (any operating system)

If you are not on Ubuntu 22.04 or 24.04, use Docker:

```bash
# Build image once (~10 min)
docker build -f docker/Dockerfile -t people_avoidance:jazzy .

# Allow display forwarding (Linux/macOS with XQuartz)
xhost +local:docker

# Start simulator
docker compose -f docker/docker-compose.yml --profile sim up

# Start student node (separate terminal — auto-rebuilds on source changes)
docker compose -f docker/docker-compose.yml --profile avoidance up
```

---

## Known differences vs real robot

| Aspect | Simulation | Real robot |
|--------|-----------|------------|
| `/scan` frame_id | `turtlebot4/rplidar_link/rplidar` | `rplidar_link` |
| Leg geometry | Perfect cylinders | Irregular, clothing-dependent |
| Scan noise | Minimal | ~5–10 cm range noise |
| Walking pattern | Random walk in circle | Unpredictable human motion |
| Robot dynamics | Ideal diff-drive | Wheel slip, floor friction |
| Reflex stops | None | REFLEX_BUMP, REFLEX_CLIFF active |

Parameters that work well in simulation may need adjustment on the real robot.
See `REAL_ROBOT.md` for tuning guidance.
