# TurtleBot4 Platform Guide

An introduction to the TurtleBot4 hardware and the ROS 2 tools you will use
throughout the lab sessions.  Work through the sections in order during your
first session before running the people-avoidance code.

> **Resources**
> - ROS 2 documentation: <https://docs.ros.org/en/jazzy/>
> - ROS 2 tutorials: <https://docs.ros.org/en/jazzy/Tutorials.html>
> - TurtleBot4 user manual: <https://turtlebot.github.io/turtlebot4-user-manual/>
> - TurtleBot4 repository: <https://github.com/turtlebot>

---

## General tips

- `Ctrl+Shift+T` — new tab in the same terminal window.
- `Ctrl+Shift+N` — new terminal window.
- `Ctrl+C` — stop the currently running node or command.
- `Tab` — autocomplete ROS 2 package, node, and topic names.
- Always source your ROS 2 environment before running any ROS 2 command
  (see [SETUP.md](SETUP.md)).

Real-world robotics involves sensor noise, timing issues, and occasional
failures.  Unexpected behaviour is normal — focus on understanding what
happened and how to recover, not on getting perfect results every time.

---

## 1 — Power on and connect

### Power on the robot

1. Place the TurtleBot4 on its dock.  The green LED on the dock lights up
   briefly, confirming power is flowing.
2. Wait for the boot sequence to complete (~60 s).  The robot's display will
   show an IP address and battery level when ready.
3. If the dock LED turns red, the battery is low — wait for a full charge
   before proceeding.

### Connect your laptop to the lab network

Connect to the same WiFi network as the robot (your instructor will provide
the SSID and password).  A wired connection between your laptop and the lab
router gives more reliable ROS 2 communication than WiFi-only.

### Configure ROS 2 on your laptop

Add the following to your shell configuration file (see [SETUP.md](SETUP.md)):

**zsh** (`~/.zshrc`):
```bash
source /opt/ros/jazzy/setup.zsh     # Ubuntu 24.04
# source /opt/ros/humble/setup.zsh  # Ubuntu 22.04
source ~/ros2_ws/install/setup.zsh 2>/dev/null || true
export ROS_DOMAIN_ID=0                     # must match the robot — ask your instructor
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp # use the same middleware as the TB4
```

**bash** (`~/.bashrc`):
```bash
source /opt/ros/jazzy/setup.bash    # Ubuntu 24.04
# source /opt/ros/humble/setup.bash # Ubuntu 22.04
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Reload: `source ~/.zshrc` or `source ~/.bashrc`

### Verify the connection

```bash
ros2 topic echo /ip
```

You should see the robot's Raspberry Pi IP address printed periodically:
```
data: 192.168.1.101
```

If nothing appears after 10 seconds, check that `ROS_DOMAIN_ID` matches the
robot and both devices are on the same network.

### SSH into the robot (optional)

You can inspect and restart processes on the robot's Raspberry Pi directly:

```bash
ssh ubuntu@<robot-ip>    # password: turtlebot4
```

> **Do not modify firmware or system software on the Raspberry Pi.**

---

## 2 — Drive the robot (teleoperation)

The simplest way to verify the robot is working is to drive it manually with
the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

A text interface appears in the terminal.  Use the keys shown to move the
robot forward, backward, and turn.  Press `Ctrl+C` to stop teleoperation.

While driving, watch for the robot's safety reflexes:
- **Bump** — the Create 3 base backs away from an obstacle on contact.
- **Cliff** — the robot stops at the edge of a table or step.
- **Wheel drop** — the robot stops if a wheel is lifted.

---

## 3 — Visualise sensor data with RViz2

RViz2 lets you see what the robot's sensors are detecting in real time:
laser scans, the TF frame tree, camera images, and maps.

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

A window opens showing the robot model with a live laser scan overlay.
Drive the robot and watch the scan update as it detects walls and obstacles.

### TF2 frame tree

ROS 2 uses a tree of coordinate frames (TF) to express where every sensor
and link is relative to every other.  To inspect the tree:

```bash
ros2 run tf2_tools view_frames
```

This generates a `frames.pdf` file in the current directory — open it to see
how `odom`, `base_link`, `rplidar_link`, and other frames are connected.

---

## 4 — Build a map with SLAM

SLAM (Simultaneous Localisation and Mapping) lets the robot build a 2D map
of the environment while localising itself within it.

```bash
# Terminal 1 — run SLAM
ros2 launch turtlebot4_navigation slam.launch.py

# Terminal 2 — visualise
ros2 launch turtlebot4_viz view_robot.launch.py
```

Drive the robot (teleop, Terminal 3) through the area you want to map.
Watch the map grow in RViz2.  The SLAM system produces more accurate maps
when the robot revisits areas and closes loops.

### Save the map

When you are satisfied with the map:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "name: {data: 'my_map'}"
```

This creates `my_map.pgm` (image) and `my_map.yaml` (metadata) in the
current directory.  You will use these files for navigation in Section 6.

---

## 5 — Record and replay data with ROS 2 bag

ROS 2 bag records all messages on specified topics to a file.  You can replay
the file later to reproduce exactly the same sensor data — useful for
developing and testing algorithms without the robot present.

### Record

```bash
# List available topics to decide what to record
ros2 topic list

# Record the LiDAR scan and odometry
ros2 bag record -o my_recording /scan /odom
```

Drive the robot while recording, then press `Ctrl+C` to stop.

### Inspect the recording

```bash
ros2 bag info my_recording
```

This shows the duration, message counts, and file size.

### Replay

```bash
ros2 bag play my_recording
```

Topics are published exactly as they were during the recording.  You can
run your algorithm node against the recorded data without the robot:

```bash
# Terminal 1
ros2 bag play my_recording

# Terminal 2 — your node processes the replayed scan
ros2 launch people_avoidance people_avoidance.launch.py
```

---

## 6 — Autonomous navigation with Nav2

Nav2 is the ROS 2 navigation stack.  It combines a global path planner and
a local motion controller to drive the robot autonomously to a goal pose.

### Launch localization

Localisation places the robot on the previously saved map:

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=my_map.yaml
```

Replace `my_map.yaml` with the path to your map file.

### Launch Nav2

```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

### Set the initial pose in RViz2

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

1. In RViz2, click **2D Pose Estimate** in the toolbar.
2. Click and drag on the map to set the robot's approximate starting position
   and heading.  The laser scan should align with the map walls after a
   moment.

### Send a navigation goal

1. Click **Nav2 Goal** in the RViz2 toolbar.
2. Click and drag on the map to set the destination pose.
3. The robot plans a path and drives autonomously to the goal.

### Visualise the ROS 2 node graph

To see all active nodes and the topics connecting them:

```bash
ros2 run rqt_graph rqt_graph
```

This is useful for understanding how the navigation stack is wired together
and for debugging communication issues.

---

## 7 — Useful diagnostic commands

```bash
# List all active topics
ros2 topic list

# Check a topic is publishing (Ctrl+C to stop)
ros2 topic hz /scan
ros2 topic hz /odom

# Print one message from a topic
ros2 topic echo --once /scan
ros2 topic echo --once /odom

# List all active nodes
ros2 node list

# Show node publishers, subscribers, services
ros2 node info /people_avoidance_node

# Check the parameter values of your node
ros2 param list /people_avoidance_node
ros2 param get  /people_avoidance_node max_linear_speed
```

---

## Next step

Once you are comfortable driving the robot and seeing sensor data, move on to
implementing the people-avoidance algorithm:

→ [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md)

When you are ready to run your implementation on the robot:

→ [REAL_ROBOT.md](REAL_ROBOT.md)
