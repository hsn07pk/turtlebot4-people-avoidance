#!/bin/bash
# Docker entrypoint: source ROS 2 and workspace, then run the given command.
set -e

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"
