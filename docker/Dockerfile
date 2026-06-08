# ─────────────────────────────────────────────────────────────────────────────
# People Avoidance Teaching Environment
# Base: ROS 2 Jazzy desktop (Ubuntu 24.04 Noble)
# ─────────────────────────────────────────────────────────────────────────────
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

# ── System packages ──────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-turtlebot4-simulator \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-numpy \
    wget \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

# ── Environment ──────────────────────────────────────────────────────────────
ENV ROS_DISTRO=jazzy
ENV TURTLEBOT4_MODEL=standard
# Prevents Gazebo GUI from hanging on "requesting list of world names"
ENV GZ_IP=127.0.0.1
# Software rendering fallback — safe on any machine; override at runtime for GPU
ENV LIBGL_ALWAYS_SOFTWARE=0

# ── Workspace ─────────────────────────────────────────────────────────────────
WORKDIR /ros2_ws

# Copy both packages
COPY src/people_avoidance  src/people_avoidance
COPY src/pedestrian_sim    src/pedestrian_sim

# Build
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install \
    && echo "Build complete"

# ── Pre-cache Gazebo Fuel models (warehouse world) ────────────────────────────
# Run during build so students don't need internet access.
# The simple_room world has no Fuel dependencies; these are for warehouse only.
RUN source /opt/ros/jazzy/setup.bash \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse"          2>/dev/null || true \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/MovAi/models/shelf"                     2>/dev/null || true \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/MovAi/models/shelf_big"                 2>/dev/null || true \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/MovAi/models/pallet_box_mobile"         2>/dev/null || true \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair"              2>/dev/null || true \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/CoffeeTable"        2>/dev/null || true \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/FemaleVisitorSit"   2>/dev/null || true \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/foldable_chair"     2>/dev/null || true \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey Barrier"     2>/dev/null || true \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/MaleVisitorOnPhone" 2>/dev/null || true \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/plateau/models/Casual female"           2>/dev/null || true \
    && echo "Fuel models cached"

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
