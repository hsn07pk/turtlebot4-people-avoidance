# Simulation Setup Guide

Two ways to run the environment: **native Ubuntu** (recommended for the full
experience) or **Docker** (reproducible, works on any machine).

---

## Option A — Native Ubuntu install

### Requirements

| | Requirement |
|--|--|
| OS | Ubuntu 24.04 LTS (Noble), 64-bit |
| RAM | 8 GB min, 16 GB recommended |
| GPU | Any; integrated graphics work |
| Disk | 12 GB free |
| Internet | Required for first run (~1.5 GB apt + Fuel model cache) |

---

### A1 — Add the ROS 2 apt repository

Skip if `/etc/apt/sources.list.d/ros2.sources` already exists.

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.sources > /dev/null
sudo apt update
```

---

### A2 — Install packages

```bash
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-turtlebot4-simulator \
    python3-colcon-common-extensions \
    python3-rosdep
```

---

### A3 — Shell configuration

Add to `~/.zshrc` (or `~/.bashrc` — replace `setup.zsh` with `setup.bash`):

```bash
# ROS 2 + workspace
source /opt/ros/jazzy/setup.zsh
source ~/ros2_ws/install/setup.zsh 2>/dev/null || true
export TURTLEBOT4_MODEL=standard

# Prevents Gazebo GUI hanging on "requesting list of world names"
export GZ_IP=127.0.0.1
```

Then reload: `source ~/.zshrc`

> **Conda users**: conda intercepts Python and breaks ROS 2 tools.  
> Run `conda deactivate` at the start of every terminal used for ROS work.

---

### A4 — Get the workspace packages

```bash
mkdir -p ~/ros2_ws/src
# Place the two packages so you have:
#   ~/ros2_ws/src/people_avoidance/
#   ~/ros2_ws/src/pedestrian_sim/
```

---

### A5 — Build

```bash
conda deactivate
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.zsh
```

`--symlink-install` links Python files in-place: edit code, re-run immediately
without rebuilding.

**Verify:**
```bash
ros2 pkg list | grep -E "people_avoidance|pedestrian_sim"
```

---

### A6 — Pre-cache Gazebo world assets (first run only)

The simple_room world (default) has **no** external dependencies.  
If you want to use the TB4 warehouse world instead, pre-cache its models:

```bash
source /opt/ros/jazzy/setup.zsh
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse"
gz fuel download -u "https://fuel.gazebosim.org/1.0/MovAi/models/pallet_box_mobile"
gz fuel download -u "https://fuel.gazebosim.org/1.0/plateau/models/Casual female"
# (shelf, shelf_big, Chair, CoffeeTable, etc. are bundled already)
```

---

### A7 — Launch the simulation

```bash
conda deactivate
ros2 launch pedestrian_sim simulation.launch.py
```

| Time | Event |
|------|-------|
| 0–5 s | Gazebo window opens, grey room visible |
| 5–20 s | TurtleBot4 robot appears in the room centre |
| ~25 s | Two blue cylinder "people" appear and start walking |

Optional arguments:
```bash
ros2 launch pedestrian_sim simulation.launch.py world:=warehouse   # TB4 warehouse world
ros2 launch pedestrian_sim simulation.launch.py num_people:=3
ros2 launch pedestrian_sim simulation.launch.py ped_speed:=0.3     # slower = easier to track
```

---

### A8 — Verify

In a second terminal:
```bash
ros2 topic list | grep -E "^/scan$|^/cmd_vel$|^/odom$"   # all three must appear
ros2 topic hz /scan                                        # ~5 Hz
ros2 topic echo --once /scan | grep ranges | head -3       # varied values, not all 0.164
```

---

### A9 — Run the skeleton node

In a third terminal:
```bash
conda deactivate
ros2 launch people_avoidance people_avoidance.launch.py
```

Expected:
```
[people_avoidance] PeopleAvoidanceNode ready — listening on '/scan', publishing to '/cmd_vel'
```

---

### Troubleshooting

| Symptom | Fix |
|---------|-----|
| `Package 'X' not found` | `source ~/ros2_ws/install/setup.zsh` |
| `ModuleNotFoundError: yaml / rclpy` | `conda deactivate` |
| Gazebo stuck on *"requesting world names"* | Make sure `GZ_IP=127.0.0.1` is exported |
| `diffdrive_controller` spawner error on first launch | Timing race; relaunch with `pkill -9 -f "gz sim"; pkill -9 -f ros2; sleep 3` |
| `/scan` all `0.164` (range_min) | Gazebo render not initialised; relaunch cleanly |
| Gazebo crashes (SIGSEGV in ogre2) | Caused by `<light>` or `<Sensors plugin>` in world SDF — use `simple_room` world |

---

## Option B — Docker

Docker gives every student an identical environment regardless of host OS.
Gazebo runs inside the container with X11 forwarded to the host display.

### B1 — Prerequisites

```bash
# Install Docker Engine (skip if already installed)
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER   # log out and back in after this

# NVIDIA Container Toolkit (only if you have an NVIDIA GPU)
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

# Allow Docker containers to use the host X11 display
xhost +local:docker
```

### B2 — Build the image

```bash
cd ~/ros2_ws
docker build -f docker/Dockerfile -t people_avoidance:jazzy .
```

This takes ~10 minutes on first build (downloads ROS packages and Fuel models).
Subsequent builds use the layer cache and are fast.

### B3 — Run with Docker Compose

```bash
cd ~/ros2_ws

# Terminal 1 — simulation (Gazebo + pedestrians)
docker compose -f docker/docker-compose.yml --profile sim up

# Terminal 2 — student node (auto-rebuilds on source changes)
docker compose -f docker/docker-compose.yml --profile avoidance up
```

Or start everything at once:
```bash
docker compose -f docker/docker-compose.yml --profile all up
```

Stop with `Ctrl-C`, then:
```bash
docker compose -f docker/docker-compose.yml down
```

### B4 — Edit code inside Docker

The `people_avoidance` source directory is bind-mounted into the container
(`src/people_avoidance → /ros2_ws/src/people_avoidance`).  
Edit files on your host — the `avoidance` service rebuilds and relaunches
automatically on each `docker compose up`.

### B5 — NVIDIA GPU in Docker

Use the `sim-gpu` profile instead of `sim`:
```bash
docker compose -f docker/docker-compose.yml --profile sim-gpu up
```

### B6 — No display / headless server

Set `LIBGL_ALWAYS_SOFTWARE=1` in the environment or in `docker-compose.yml` to
use software rendering (no GPU, no display required for the ROS topics — Gazebo
window still needs a display unless you add a virtual framebuffer):

```bash
# Minimal headless test (topics only, no Gazebo window)
docker run --rm --network host \
    -e GZ_IP=127.0.0.1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    people_avoidance:jazzy \
    ros2 launch pedestrian_sim simulation.launch.py
```

---

## Terminal layout at a glance

| Terminal | Command |
|----------|---------|
| **T1** Simulation | `ros2 launch pedestrian_sim simulation.launch.py` |
| **T2** Your node | `ros2 launch people_avoidance people_avoidance.launch.py` |
| **T3** Inspect | `ros2 topic echo /cmd_vel` |
| **T4** RQT tools | `conda deactivate && ros2 run rqt_gui rqt_gui` |
