#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source the workspace if it has been built
if [ -f "$HOME/imav_ws/install/setup.bash" ]; then
    source "$HOME/imav_ws/install/setup.bash"
    echo "[entrypoint] Sourced imav_ws overlay."
fi

# Export Gazebo model/resource paths for PX4 models
PX4_DIR="$HOME/imav_ws/PX4-Autopilot"
if [ -d "$PX4_DIR" ]; then
    export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
    echo "[entrypoint] PX4 Gazebo resource paths set."
fi

# Ensure PX4 repo exists outside the workspace and build SITL once
if [ ! -d "$PX4_DIR/.git" ]; then
    echo "[entrypoint] Cloning PX4-Autopilot into $PX4_DIR ..."
    git clone --recursive --depth 1 git@github.com:EESC-LabRoM/PX4-Autopilot.git "$PX4_DIR"
fi

if [ -d "$PX4_DIR" ] && [ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    echo "[entrypoint] Building PX4 SITL (first run only) ..."
    cd "$PX4_DIR"
    make px4_sitl
fi

# Ensure ROS 2 workspace deps exist as sibling packages
ROS_WS_SRC="$HOME/imav_ws/src"
if [ ! -d "$ROS_WS_SRC/px4_msgs/.git" ]; then
    echo "[entrypoint] Cloning px4_msgs into $ROS_WS_SRC ..."
    git clone https://github.com/PX4/px4_msgs.git "$ROS_WS_SRC/px4_msgs"
fi

if [ ! -d "$ROS_WS_SRC/px4_ros_com/.git" ]; then
    echo "[entrypoint] Cloning px4_ros_com into $ROS_WS_SRC ..."
    git clone https://github.com/PX4/px4_ros_com.git "$ROS_WS_SRC/px4_ros_com"
fi

# Install ROS 2 dependencies and build the workspace once
ROS_WS="$HOME/imav_ws"
if [ -d "$ROS_WS" ] && [ ! -f "$ROS_WS/install/setup.bash" ]; then
    echo "[entrypoint] Installing ROS 2 dependencies via rosdep ..."
    cd "$ROS_WS"
    rosdep update
    rosdep install --from-paths src -i -y --rosdistro humble

    echo "[entrypoint] Building ROS 2 workspace (first run only) ..."
    colcon build --symlink-install
fi

echo "[entrypoint] Environment ready. ROS_DISTRO=$ROS_DISTRO"

exec "$@"
