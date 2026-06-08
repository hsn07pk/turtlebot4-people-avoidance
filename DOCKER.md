# Docker Guide

Use Docker if you are not on Ubuntu 24.04, or if you want a guaranteed
identical environment across all machines in the lab.

---

## Prerequisites

### Install Docker Engine

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
# Log out and back in so the group membership takes effect
docker run --rm hello-world   # verify
```

### Install Docker Compose plugin

```bash
sudo apt install -y docker-compose-plugin
docker compose version        # verify — should print v2.x
```

### NVIDIA GPU (optional — skip if you have no NVIDIA card)

```bash
# Install NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## Build the image

From the repository root (takes ~10 minutes the first time; subsequent builds
use the cache and are fast):

```bash
cd ~/ros2_ws
docker build -f docker/Dockerfile -t people_avoidance:jazzy .
```

The build:
- Installs ROS 2 Jazzy desktop + TurtleBot4 simulator packages
- Builds both ROS 2 packages (`people_avoidance`, `pedestrian_sim`)
- Pre-downloads all Gazebo Fuel models so no internet is needed at runtime

---

## Allow display access (required for Gazebo)

```bash
xhost +local:docker
```

Run this once per login session before starting any container with a GUI.
On macOS, install [XQuartz](https://www.xquartz.org/) first and run
`xhost +localhost`.

---

## Run with Docker Compose

### Simulation only

```bash
docker compose -f docker/docker-compose.yml --profile sim up
```

Starts Gazebo with the simple room world and two walking pedestrians.
The Gazebo window opens on your screen via X11 forwarding.

### Your algorithm node only

```bash
docker compose -f docker/docker-compose.yml --profile avoidance up
```

Starts only the `people_avoidance` node.  The `src/people_avoidance`
directory is **bind-mounted** from your host, so edits on your laptop take
effect immediately — the container rebuilds the package on each `up`.

### Everything together

```bash
docker compose -f docker/docker-compose.yml --profile all up
```

### With NVIDIA GPU

```bash
docker compose -f docker/docker-compose.yml --profile sim-gpu up
```

### Stop

```bash
docker compose -f docker/docker-compose.yml down
```

---

## Edit code inside Docker

The `people_avoidance` source is bind-mounted:

```
Host:       ~/ros2_ws/src/people_avoidance/
Container:  /ros2_ws/src/people_avoidance/   (same files)
```

Edit on your host with your normal editor.  The `avoidance` service runs
`colcon build` automatically when you `docker compose up`, so changes are
always compiled before the node starts.

---

## Run a single command inside the container

```bash
# Open an interactive shell
docker run --rm -it \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/ros2_ws/src/people_avoidance:/ros2_ws/src/people_avoidance \
    people_avoidance:jazzy \
    bash

# Check topics (while sim is running in another container)
docker run --rm \
    --network host \
    people_avoidance:jazzy \
    ros2 topic list
```

---

## Connecting to the real robot from inside Docker

The container uses `--network host`, so it shares the host's network interface.
Set `ROS_DOMAIN_ID` to match the robot before starting the container:

```bash
export ROS_DOMAIN_ID=0    # match your robot's domain ID

docker run --rm -it \
    --network host \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -v ~/ros2_ws/src/people_avoidance:/ros2_ws/src/people_avoidance \
    people_avoidance:jazzy \
    ros2 launch people_avoidance people_avoidance.launch.py
```

Or add `ROS_DOMAIN_ID` to the environment section in `docker-compose.yml`.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `Cannot connect to display` | Run `xhost +local:docker` on the host |
| `Error response from daemon: could not select device driver "nvidia"` | NVIDIA Container Toolkit not installed or Docker not restarted after `nvidia-ctk` |
| Topics not visible from container | Check `--network host` is set and `ROS_DOMAIN_ID` matches |
| Gazebo crashes on startup | Try the non-GPU `sim` profile; GPU rendering may need the `sim-gpu` profile |
| `colcon build` fails in container | Check the bind-mounted source has no syntax errors |
