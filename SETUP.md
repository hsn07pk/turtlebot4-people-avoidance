# Setup Guide

One-time setup on your laptop.  Takes about 20 minutes on a fresh Ubuntu install.

---

## Requirements

| | |
|--|--|
| OS | Ubuntu 24.04 LTS (Noble), 64-bit |
| RAM | 8 GB minimum |
| Disk | 5 GB free |

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

## 2 — Install ROS 2 Jazzy

```bash
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
```

---

## 3 — Configure your shell

Add to `~/.zshrc` (or `~/.bashrc` — replace `setup.zsh` with `setup.bash`):

```bash
source /opt/ros/jazzy/setup.zsh
source ~/ros2_ws/install/setup.zsh 2>/dev/null || true
export ROS_DOMAIN_ID=0        # must match the robot — your instructor will tell you the value
```

Reload: `source ~/.zshrc`

---

## 4 — Clone and build the workspace

```bash
git clone https://gitlab.example.com/course/people_avoidance.git ros2_ws
cd ros2_ws
conda deactivate
colcon build --symlink-install
source install/setup.zsh
```

`--symlink-install` links Python source files in-place so you can edit code
and re-run without rebuilding.

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
existing functions take effect immediately without rebuilding (symlink install).

---

## That's it

Your workspace is ready.  Continue with:
- [REAL_ROBOT.md](REAL_ROBOT.md) — running your code on the TurtleBot4
- [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md) — understanding what to implement
