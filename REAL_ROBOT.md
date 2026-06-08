# Running on the TurtleBot4

This is the primary lab guide.  Your algorithm runs on your laptop and
communicates with the physical TurtleBot4 over WiFi.

```
Your laptop                           TurtleBot4 (RPi 4)
─────────────────────────────         ─────────────────────────────
people_avoidance_node                 /scan   RPLidar A1 driver
  subscribes /scan          ◄──WiFi── /odom   Create 3 odometry
  publishes  /cmd_vel       ──WiFi──► /cmd_vel Create 3 motor control
```

---

## Before the lab

1. Your code compiles (`colcon build` succeeds with no errors).
2. You have tested at least Stage 1 in simulation — the node does not crash
   when it receives a scan.
3. You know your group's `ROS_DOMAIN_ID` (given by the instructor).

---

## 1 — Connect to the robot network

Connect your laptop to the **lab WiFi** specified by your instructor.
The TurtleBot4 is on the same network.

Check connectivity — the robot's topics should appear:
```bash
source ~/.zshrc    # zsh — loads ROS_DOMAIN_ID
# source ~/.bashrc # bash
ros2 topic list | grep -E "^/scan$|^/cmd_vel$|^/odom$"
```

If those three topics are not listed, wait 10 seconds and retry.  If still
missing, check with your instructor — the robot may need to be restarted.

---

## 2 — Verify the LiDAR

```bash
# Should print ~5 Hz
ros2 topic hz /scan

# Should show varied range values (not all the same number)
ros2 topic echo --once /scan | grep -A5 "ranges:"
```

Walk slowly in front of the robot — you should see values in the `ranges` array
decrease as you approach.

---

## 3 — Run your node

```bash
conda deactivate
source ~/ros2_ws/install/setup.zsh    # zsh
# source ~/ros2_ws/install/setup.bash # bash
ros2 launch people_avoidance people_avoidance.launch.py
```

The node will print:
```
[people_avoidance] PeopleAvoidanceNode ready — listening on '/scan', publishing to '/cmd_vel'
```

**First run:** keep `max_linear_speed` low until you are confident the
controller behaves correctly:

```bash
ros2 launch people_avoidance people_avoidance.launch.py \
    max_linear_speed:=0.1 \
    max_angular_speed:=0.5 \
    obstacle_radius_scale:=3.0
```

---

## 4 — Verify your node is working

Open a second terminal:

```bash
source ~/ros2_ws/install/setup.zsh

# Watch what your node outputs — should update at ~5 Hz
ros2 topic echo /cmd_vel

# See live detections (add this print to your detect_legs() while debugging)
ros2 run people_avoidance people_avoidance_node --ros-args --log-level debug
```

Expected progression:

| Stage complete | What you observe |
|----------------|-----------------|
| Stubs only | `/cmd_vel` publishes zeros; robot sits still |
| Stage 1 done | Debug log shows `LegMeasurement` entries when people are nearby |
| Stage 2 done | Debug log shows persistent `Track` objects that survive multiple scans |
| Stage 3 done | Robot moves away from approaching people |

---

## 5 — Stop the robot safely

Any of these works at any time:
```bash
# Publish a zero command once
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# Press Ctrl-C in the terminal running your node
# Press the physical power button on the Create 3 base (cuts motors)
```

---

## 6 — Tuning for real-world conditions

Simulation uses idealised cylinders.  Real legs are noisier.  Start with these
adjustments and refine during the session:

| Parameter | Sim default | Real-world start | Why |
|-----------|-------------|-----------------|-----|
| `distance_threshold` | 0.10 m | 0.08–0.12 m | More scan noise |
| `leg_radius` | 0.10 m | 0.08–0.12 m | Clothing varies |
| `max_leg_width` | 0.25 m | 0.20–0.35 m | Stance width varies |
| `dt` | 0.1 s | match `ros2 topic hz /scan` | Sync KF step to real rate |
| `max_linear_speed` | 0.2 m/s | start at 0.1 m/s | Increase once confident |
| `obstacle_radius_scale` | 2.0 | 3.0–4.0 | More safety margin |

Override without editing code:
```bash
ros2 launch people_avoidance people_avoidance.launch.py \
    distance_threshold:=0.09 max_leg_width:=0.30 max_linear_speed:=0.15
```

---

## 7 — Robot safety systems

The Create 3 base has built-in reflexes that override `/cmd_vel` regardless of
what your node publishes:

| Reflex | Trigger | Effect |
|--------|---------|--------|
| `REFLEX_BUMP` | Front bumper contact | Backs up, stops |
| `REFLEX_CLIFF` | Cliff sensor detects drop | Stops immediately |
| `REFLEX_WHEEL_DROP` | Wheel lifted | Stops |

These protect the robot and cannot be disabled.  If the robot stops
unexpectedly:
```bash
ros2 topic echo /hazard_detection   # shows what triggered the reflex
```

---

## 8 — Frame IDs: real robot vs simulation

The `/scan` message `frame_id` differs slightly between environments.
Your algorithm works in the laser frame directly — no changes needed.

| | Real robot | Simulation |
|--|--|--|
| `/scan` frame_id | `rplidar_link` | `turtlebot4/rplidar_link/rplidar` |
| `/odom` frame | `odom` | `odom` |
| `/cmd_vel` | same | same |

---

## 9 — Test procedure for the lab session

1. Connect to lab WiFi, verify topics appear (Step 1–2).
2. Run your node with conservative limits (Step 3).
3. One person walks slowly toward the front of the robot from 3 m — the robot
   should stop or steer away before contact.
4. Two people approach simultaneously from different angles.
5. One person stands still while another walks past — robot should track the
   moving one.
6. Once the behaviour is reliable, increase `max_linear_speed` to the default
   0.2 m/s.
