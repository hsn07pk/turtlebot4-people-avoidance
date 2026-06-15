# Setup Guide

One-time setup on your laptop.  Takes about 20 minutes on a fresh Ubuntu install.

---

## Supported configurations

| Ubuntu | ROS 2 distro | Status |
|--------|-------------|--------|
| 24.04 LTS (Noble) | **Jazzy Jalisco** | Recommended — LTS, supported until 2029 |
| 22.04 LTS (Jammy) | **Humble Hawksbill** | Supported — LTS, supported until 2027 |

Choose the row that matches your Ubuntu version.  All commands below show
both variants; substitute the one that applies to you.

> **Conda users** — conda intercepts Python and breaks ROS 2 tools.  
> Run `conda deactivate` at the start of every terminal used for ROS work.

---

## 1 — Add the ROS 2 apt repository

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

## 2 — Install ROS 2

**Ubuntu 24.04 — Jazzy:**
```bash
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
```

**Ubuntu 22.04 — Humble:**
```bash
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
```

---

## 3 — Configure your shell

Add the following lines to your shell configuration file and then reload it.

**zsh** (`~/.zshrc`):
```bash
# ROS 2 — pick the line matching your Ubuntu version
source /opt/ros/jazzy/setup.zsh     # Ubuntu 24.04
# source /opt/ros/humble/setup.zsh  # Ubuntu 22.04

source ~/ros2_ws/install/setup.zsh 2>/dev/null || true
export ROS_DOMAIN_ID=0              # must match the robot — your instructor will tell you
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

```bash
source ~/.zshrc
```

**bash** (`~/.bashrc`):
```bash
# ROS 2 — pick the line matching your Ubuntu version
source /opt/ros/jazzy/setup.bash    # Ubuntu 24.04
# source /opt/ros/humble/setup.bash # Ubuntu 22.04

source ~/ros2_ws/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

```bash
source ~/.bashrc
```

**Verify:**
```bash
echo $ROS_DISTRO    # jazzy  or  humble
```

---

## 4 — Clone and build the workspace

```bash
git clone https://github.com/hsn07pk/turtlebot4-people-avoidance.git ros2_ws
cd ros2_ws
conda deactivate
colcon build --symlink-install
```

Source the workspace:

```bash
# zsh
source install/setup.zsh

# bash
source install/setup.bash
```

`--symlink-install` links Python source files in-place: edit code, re-run
immediately without rebuilding.

**Verify:**
```bash
ros2 pkg list | grep people_avoidance   # should print: people_avoidance
```

---

## 5 — Rebuild after editing

```bash
cd ~/ros2_ws
colcon build --packages-select people_avoidance --symlink-install
```

You need to rebuild whenever you add a new Python import.  Simple edits to
existing functions take effect immediately (symlink install).

---

## That's it

Your workspace is ready.  Continue with:
- [TURTLEBOT4_GUIDE.md](TURTLEBOT4_GUIDE.md) — learn the robot platform
- [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md) — understand what to implement
