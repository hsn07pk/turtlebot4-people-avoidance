# Running on a Real TurtleBot4

This guide assumes you have completed the simulation exercises and now want to
deploy the `people_avoidance` node on a physical TurtleBot4 (standard or lite).

> **Safety first** — the avoidance controller publishes to `/cmd_vel`.  
> Before running on a real robot, make sure:
> - The test area is clear and at least 3 m wide.
> - A person is within reach of the robot's physical power button at all times.
> - Start with `max_linear_speed: 0.1` (half the default) until you are
>   confident the controller behaves correctly.

---

## 1 — Architecture

In simulation, Gazebo and all ROS 2 nodes run on one machine.  With a real
robot the system splits across two machines:

```
┌─────────────────────────────────┐     WiFi      ┌───────────────────────────┐
│  Your laptop                    │◄─────────────►│  TurtleBot4               │
│                                 │               │                           │
│  people_avoidance_node          │               │  RPi 4 running ROS 2:     │
│    subscribes to /scan          │               │    /scan   (RPLidar A1)   │
│    publishes to /cmd_vel        │               │    /odom   (Create3)      │
│                                 │               │    /cmd_vel               │
└─────────────────────────────────┘               └───────────────────────────┘
```

The robot's RPi runs the sensor drivers and motor control.  Your laptop runs
your algorithm.  ROS 2 DDS connects them over WiFi automatically, as long as
both are on the same network and configured with the same domain ID.

---

## 2 — Network setup

### Connect to the TB4 network

The TurtleBot4 creates its own WiFi access point on first boot, or can be
configured to join an existing network.  See the official TB4 networking guide:
<https://turtlebot.github.io/turtlebot4-user-manual/setup/networking.html>

The simplest setup for a lab:

1. Connect the robot to the lab WiFi.
2. Connect your laptop to the same WiFi.
3. Find the robot's IP: `ros2 run turtlebot4_navigation find_turtlebot4` or
   check the display on the robot (TB4 standard) for its IP address.

### ROS 2 domain ID

By default both robot and laptop use `ROS_DOMAIN_ID=0`.  If several groups
work simultaneously, assign each group a unique domain ID (0–101):

```bash
# On your laptop — match whatever the robot's domain ID is set to
export ROS_DOMAIN_ID=0      # default; change to match your robot

# Add to ~/.zshrc to persist
echo "export ROS_DOMAIN_ID=0" >> ~/.zshrc
```

Check that the robot is visible:
```bash
ros2 topic list | grep scan     # should show /scan
ros2 node list                  # should show TB4 nodes
```

---

## 3 — Verify topics before running

Confirm the exact same topics are available on the real robot as in simulation:

```bash
# LiDAR — should publish at ~5 Hz
ros2 topic hz /scan

# One scan message — ranges should vary (not all 0.164)
ros2 topic echo --once /scan | grep ranges | head -3

# Odometry
ros2 topic echo --once /odom | grep -A3 position

# Check cmd_vel is being subscribed to by the robot
ros2 topic info /cmd_vel
```

### Frame ID differences: simulation vs real robot

| | Simulation | Real robot |
|--|--|--|
| LiDAR frame_id | `turtlebot4/rplidar_link/rplidar` | `rplidar_link` |
| Odom frame | `odom` | `odom` |
| Base frame | `base_link` | `base_link` |

The `laser_frame` ROS 2 parameter in `people_avoidance_node` is **reference only**
(used for documentation/TF lookups, not for the core algorithm which works in the
laser frame directly).  No change is needed to run on the real robot.

---

## 4 — Run the node

```bash
# Source your workspace
source /opt/ros/jazzy/setup.zsh
source ~/ros2_ws/install/setup.zsh

# Launch with conservative speed limits for first run
ros2 launch people_avoidance people_avoidance.launch.py \
    max_linear_speed:=0.1 \
    max_angular_speed:=0.5 \
    obstacle_radius_scale:=3.0
```

You do **not** run `simulation.launch.py` or `pedestrian_sim` — those are only
for the simulated environment.  Real people walking near the robot are the input.

---

## 5 — Real-world tuning vs simulation

The simulation uses idealised cylinders as people.  Real legs produce messier
scans.  Expect to tune these parameters:

| Parameter | Sim default | Real-world starting point | Why |
|-----------|-------------|--------------------------|-----|
| `distance_threshold` | 0.10 m | 0.08–0.12 m | Real scans have more noise |
| `leg_radius` | 0.10 m | 0.08–0.12 m | Clothing changes effective radius |
| `max_leg_width` | 0.25 m | 0.20–0.35 m | Stance width varies |
| `dt` | 0.1 s | Match actual scan rate | Check `ros2 topic hz /scan` |
| `max_linear_speed` | 0.2 m/s | Start at 0.1 m/s | Increase only when tracking is reliable |
| `obstacle_radius_scale` | 2.0 | 3.0–4.0 | More margin for real uncertainty |

Override without editing code:
```bash
ros2 launch people_avoidance people_avoidance.launch.py \
    distance_threshold:=0.09 \
    max_leg_width:=0.30 \
    max_linear_speed:=0.15
```

---

## 6 — Checking the robot's safety systems

The TurtleBot4's Create 3 base has built-in reflexes that override `/cmd_vel`:

| Reflex | Trigger | Effect |
|--------|---------|--------|
| `REFLEX_BUMP` | Front bumper contact | Backs up, stops |
| `REFLEX_CLIFF` | Cliff sensor detects drop | Stops immediately |
| `REFLEX_WHEEL_DROP` | Wheel lifted off ground | Stops |

These are independent of your algorithm and **cannot** be disabled.  If the robot
stops unexpectedly, check `/hazard_detection`:
```bash
ros2 topic echo /hazard_detection
```

---

## 7 — Stopping the robot safely

Publish a zero Twist at any time:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

Or press the physical power button on the Create 3 base to cut motors.

---

## 8 — Typical test procedure

1. Place the robot in an open area (≥ 3 m × 3 m).
2. Start your node with conservative limits (Step 4 above).
3. Stand 2 m in front of the robot and walk slowly toward it — it should stop
   or turn away before reaching your legs.
4. Test edge cases: approaching from the side, two people simultaneously,
   one person moving away while another approaches.
5. Once behaviour is reliable, increase `max_linear_speed` toward the default
   0.2 m/s and reduce `obstacle_radius_scale` toward 2.0.
