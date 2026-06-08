# Docker Guide

Use Docker if you are not on Ubuntu 24.04, or if you want a guaranteed
identical environment regardless of your host OS.

Jump to your platform:
- [Linux (Ubuntu)](#linux-ubuntu)
- [macOS](#macos)
- [Windows](#windows)

---

## Linux (Ubuntu)

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
docker compose version        # should print v2.x
```

### NVIDIA GPU (optional)

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Allow display access (once per session)

```bash
xhost +local:docker
```

### Build and run

```bash
cd ~/ros2_ws
docker build -f docker/Dockerfile -t people_avoidance:jazzy .

# Simulation
docker compose -f docker/docker-compose.yml --profile sim up

# Your algorithm node (separate terminal)
docker compose -f docker/docker-compose.yml --profile avoidance up
```

---

## macOS

> **Apple Silicon (M1/M2/M3)**: the image is built for `linux/amd64`.
> It runs via Rosetta 2 emulation — Gazebo will be slow.
> The algorithm node (`avoidance` profile) runs at full speed and is the
> main use case on Apple Silicon.

### 1 — Install Docker Desktop

Download and install **Docker Desktop for Mac** from
[docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop/).

Open Docker Desktop and wait for the engine to start (whale icon in the menu bar
turns steady).

### 2 — Install XQuartz (required for Gazebo window)

XQuartz provides the X11 display server that Gazebo uses to render its GUI.

```bash
brew install --cask xquartz
```

Or download the installer from [xquartz.org](https://www.xquartz.org/).

After installing, **log out and log back in** (XQuartz installs a launch agent
that activates on login).

### 3 — Enable network connections in XQuartz

Open XQuartz, then:
**XQuartz → Preferences → Security** → tick **"Allow connections from network clients"**

Quit and reopen XQuartz for the setting to take effect.

### 4 — Allow display access (once per session)

```bash
xhost +localhost
```

Run this every time you open a fresh terminal before starting a container.

### 5 — Clone the repository

```bash
git clone https://version.aalto.fi/kucnert1/ubiss_2026.git ros2_ws
cd ros2_ws
```

### 6 — Build the image

```bash
# Intel Mac
docker build -f docker/Dockerfile -t people_avoidance:jazzy .

# Apple Silicon — explicitly request the amd64 platform
docker build --platform linux/amd64 -f docker/Dockerfile -t people_avoidance:jazzy .
```

First build takes ~15 minutes.

### 7 — Set the display variable and run

macOS Docker containers cannot use `--network host` (Docker Desktop on macOS uses
a VM).  Use `host.docker.internal` instead:

```bash
# Algorithm node only — connects to a real robot over the lab WiFi
export ROS_DOMAIN_ID=0   # set to your group's domain ID
docker run --rm -it \
    -e DISPLAY=host.docker.internal:0 \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -p 7400-7500:7400-7500/udp \
    -v "$(pwd)/src/people_avoidance:/ros2_ws/src/people_avoidance" \
    people_avoidance:jazzy \
    ros2 launch people_avoidance people_avoidance.launch.py

# Simulation (Gazebo window appears via XQuartz)
docker run --rm -it \
    -e DISPLAY=host.docker.internal:0 \
    -e GZ_IP=127.0.0.1 \
    -e TURTLEBOT4_MODEL=standard \
    -p 7400-7500:7400-7500/udp \
    people_avoidance:jazzy \
    ros2 launch pedestrian_sim simulation.launch.py
```

> **Note on `--network host`**: macOS Docker Desktop does not support
> `--network host`.  The `-p 7400-7500:7400-7500/udp` flag exposes the DDS
> discovery ports used by ROS 2.  For real-robot work this is usually
> sufficient; if topics are not visible, check that the robot and your Mac are
> on the same WiFi subnet.

### macOS troubleshooting

| Symptom | Fix |
|---------|-----|
| `Error: Can't open display` | XQuartz not running, or `xhost +localhost` not run |
| Gazebo window does not appear | Check XQuartz Security → "Allow connections from network clients" is ticked |
| Robot topics not visible | Docker Desktop on macOS cannot use `--network host`; ensure UDP ports are forwarded and `ROS_DOMAIN_ID` matches |
| Very slow Gazebo on M1/M2/M3 | Expected — amd64 image runs under emulation; use the `avoidance` profile only for real-robot work |

---

## Windows

Docker on Windows runs inside **WSL 2** (Windows Subsystem for Linux).
All commands below are run inside a **WSL 2 terminal**, not PowerShell or CMD.

### 1 — Enable WSL 2

Open PowerShell **as Administrator** and run:

```powershell
wsl --install
```

Restart when prompted.  After restart, open **Ubuntu** from the Start menu and
complete the Linux user setup.

### 2 — Install Docker Desktop

Download **Docker Desktop for Windows** from
[docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop/).

During installation, ensure **"Use WSL 2 based engine"** is checked.

After installing, open Docker Desktop → **Settings → Resources → WSL Integration**
→ enable integration for your Ubuntu distro.

Verify inside WSL 2:
```bash
docker run --rm hello-world
docker compose version
```

### 3 — Display setup for Gazebo

#### Windows 11 — WSLg (built-in, recommended)

Windows 11 includes **WSLg**, which provides a built-in X11/Wayland server.
No extra software is needed.

Check it works:
```bash
echo $DISPLAY    # should print something like :0 or :1
```

If `$DISPLAY` is empty, update WSL:
```powershell
# In PowerShell (Admin)
wsl --update
wsl --shutdown
```

#### Windows 10 — VcXsrv

1. Download and install **VcXsrv** from
   [sourceforge.net/projects/vcxsrv](https://sourceforge.net/projects/vcxsrv/).
2. Launch **XLaunch** (Start menu), choose:
   - Display settings: **Multiple windows**
   - Start no client
   - Extra settings: tick **"Disable access control"**
3. Set the display variable in WSL 2:

```bash
export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0.0
# Add to ~/.bashrc to persist:
echo "export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0.0" >> ~/.bashrc
```

### 4 — Clone the repository (inside WSL 2)

```bash
git clone https://version.aalto.fi/kucnert1/ubiss_2026.git ros2_ws
cd ros2_ws
```

### 5 — Build the image (inside WSL 2)

```bash
docker build -f docker/Dockerfile -t people_avoidance:jazzy .
```

### 6 — Allow display access and run (inside WSL 2)

```bash
# Windows 11 with WSLg — display is already available
export ROS_DOMAIN_ID=0

# Simulation
docker run --rm -it \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e GZ_IP=127.0.0.1 \
    -e TURTLEBOT4_MODEL=standard \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/src/people_avoidance:/ros2_ws/src/people_avoidance" \
    people_avoidance:jazzy \
    ros2 launch pedestrian_sim simulation.launch.py

# Algorithm node (real robot)
docker run --rm -it \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -v "$(pwd)/src/people_avoidance:/ros2_ws/src/people_avoidance" \
    people_avoidance:jazzy \
    ros2 launch people_avoidance people_avoidance.launch.py
```

Or use Docker Compose exactly as on Linux:
```bash
xhost +local:docker   # WSLg — may not be needed but harmless
docker compose -f docker/docker-compose.yml --profile sim up
docker compose -f docker/docker-compose.yml --profile avoidance up
```

### Windows troubleshooting

| Symptom | Fix |
|---------|-----|
| `docker: command not found` in WSL 2 | Docker Desktop WSL integration not enabled for your distro |
| `Cannot connect to display` on Win 11 | Run `wsl --update` and restart WSL |
| `Cannot connect to display` on Win 10 | VcXsrv not running, or `DISPLAY` not set correctly in WSL 2 |
| Gazebo window flickers or is very slow | Normal on Windows; try reducing the Gazebo render rate |
| Robot topics not visible | Ensure laptop and robot are on the same WiFi subnet; `ROS_DOMAIN_ID` must match |
| `docker compose` not found | Install Docker Compose plugin: `sudo apt install docker-compose-plugin` inside WSL 2 |

---

## Common operations (all platforms)

### Edit code — changes take effect immediately

The `src/people_avoidance` directory is bind-mounted into the container.
Edit files with any editor on your host; the container sees the changes live.
The `avoidance` service rebuilds the package on every `docker compose up`.

### Open an interactive shell inside the container

```bash
docker run --rm -it \
    -e DISPLAY=$DISPLAY \
    -v "$(pwd)/src/people_avoidance:/ros2_ws/src/people_avoidance" \
    people_avoidance:jazzy bash
```

### Check ROS 2 topics from inside the container

```bash
docker run --rm --network host people_avoidance:jazzy ros2 topic list
```

### Rebuild the image after Dockerfile changes

```bash
docker build --no-cache -f docker/Dockerfile -t people_avoidance:jazzy .
```

### Connect to real robot from inside Docker

Set `ROS_DOMAIN_ID` to match the robot (given by the instructor) and ensure
your machine is on the lab WiFi.  On Linux the `--network host` flag gives the
container direct access to the host network.  On macOS/Windows, Docker uses a
VM network — use the UDP port-forwarding approach shown in the macOS section.
