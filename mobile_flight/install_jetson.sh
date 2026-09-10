#!/bin/bash
# =============================================================================
#  install_jetson.sh — Native installation for Jetson Orin NX
#  Installs all dependencies to run the mobile_flight stack natively.
#
#  Targets: JetPack 6.x (L4T r36.x) | Ubuntu 22.04 | ROS 2 Humble
#
#  Usage:
#    chmod +x install_jetson.sh
#    ./install_jetson.sh
#
#  What this script installs:
#    1. System / build tools
#    2. ROS 2 Humble (ros-base + required packages)
#    3. Micro XRCE-DDS Agent (from source)
#    4. PX4 toolchain build dependencies
#    5. ROS 2 workspace dependencies (px4_msgs, px4_ros_com)
#    6. Configures ~/.bashrc with all needed sourcing
# =============================================================================

set -euo pipefail

# ── Colors ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

info()    { echo -e "${GREEN}[INFO]${NC} $*"; }
warning() { echo -e "${YELLOW}[WARN]${NC} $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ── Sanity checks ────────────────────────────────────────────────────────────
[[ $(id -u) -eq 0 ]] && error "Do NOT run as root. Run as your normal user (sudo will be called when needed)."

UBUNTU_CODENAME=$(lsb_release -cs 2>/dev/null || echo "")
[[ "$UBUNTU_CODENAME" == "jammy" ]] || warning "Expected Ubuntu 22.04 (jammy). Got: '$UBUNTU_CODENAME'. Proceed with caution."

ARCH=$(uname -m)
[[ "$ARCH" == "aarch64" ]] || warning "Expected aarch64, got '$ARCH'. This script targets Jetson ARM64."

IMAV_WS="$HOME/imav_ws"
ROS_WS_SRC="$IMAV_WS/src"

info "Starting native mobile_flight installation on Jetson Orin NX"
info "Workspace: $IMAV_WS"

# =============================================================================
# 0. FIX SSL / CLOCK (must run before any HTTPS apt repo)
# =============================================================================
info "=== [0/6] Updating CA certificates and syncing clock ==="

# Update ca-certificates from default Ubuntu repos (no HTTPS needed for this)
sudo apt-get update -o Dir::Etc::sourcelist="sources.list" \
                    -o Dir::Etc::sourceparts="-" \
                    -o APT::Get::List-Cleanup="0"
sudo apt-get install -y --no-install-recommends ca-certificates openssl tzdata
sudo update-ca-certificates

# Sync system clock — an out-of-sync clock is the most common cause of cert errors
if command -v timedatectl &>/dev/null; then
    sudo timedatectl set-ntp true
    sleep 2  # give NTP a moment to kick in
fi

# =============================================================================
# 1. SYSTEM DEPENDENCIES
# =============================================================================
info "=== [1/6] Installing system dependencies ==="

# Add ROS 2 apt repository first so colcon/rosdep/vcstool are resolvable

# Workaround: packages.ros.org uses a cert that may fail name-validation on some
# Jetson/carrier-board setups. Bypass peer verification only for that host.
if [ ! -f /etc/apt/apt.conf.d/99ros-ssl-fix ]; then
    echo 'Acquire::https::packages.ros.org::Verify-Peer "false";' \
        | sudo tee /etc/apt/apt.conf.d/99ros-ssl-fix > /dev/null
    info "Applied SSL workaround for packages.ros.org"
fi

if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    info "Adding ROS 2 apt repository..."
    sudo curl -sSL --insecure https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    git \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    ca-certificates \
    software-properties-common \
    sudo \
    build-essential \
    cmake \
    ninja-build \
    python3-pip \
    gdb \
    vim \
    tmux \
    astyle \
    genromfs \
    protobuf-compiler \
    libeigen3-dev \
    libopencv-dev \
    exiftool \
    python3-jinja2 \
    python3-jsonschema \
    python3-numpy \
    python3-empy \
    python3-packaging \
    python3-toml \
    python3-future \
    python3-yaml \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    libxml2-utils \
    xmlstarlet \
    unzip

pip3 install --no-cache-dir --user pyros-genmsg pyulog kconfiglib

# =============================================================================
# 2. ROS 2 HUMBLE
# =============================================================================
info "=== [2/6] Installing ROS 2 Humble ==="

if ! dpkg -l ros-humble-ros-base &>/dev/null; then
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-std-srvs \
        ros-humble-sensor-msgs \
        ros-humble-geometry-msgs \
        ros-humble-nav-msgs \
        ros-humble-tf2-ros \
        ros-humble-actuator-msgs \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep
else
    info "ROS 2 Humble already installed — skipping."
fi

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# =============================================================================
# 3. MICRO XRCE-DDS AGENT
# =============================================================================
info "=== [3/6] Building Micro XRCE-DDS Agent from source ==="

XRCE_VERSION="v2.4.3"
XRCE_TMP="/tmp/Micro-XRCE-DDS-Agent"

if ! command -v MicroXRCEAgent &>/dev/null; then
    rm -rf "$XRCE_TMP"
    git clone -b "$XRCE_VERSION" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$XRCE_TMP"
    cmake -S "$XRCE_TMP" -B "$XRCE_TMP/build"
    cmake --build "$XRCE_TMP/build" --parallel "$(nproc)"
    sudo cmake --install "$XRCE_TMP/build"
    sudo ldconfig /usr/local/lib/
    rm -rf "$XRCE_TMP"
    info "Micro XRCE-DDS Agent installed successfully."
else
    info "MicroXRCEAgent already found — skipping build."
fi

# =============================================================================
# 4. WORKSPACE SETUP
# =============================================================================
info "=== [4/6] Setting up imav_ws ==="

mkdir -p "$ROS_WS_SRC"

# Clone px4_msgs
if [ ! -d "$ROS_WS_SRC/px4_msgs/.git" ]; then
    info "Cloning px4_msgs..."
    git clone https://github.com/PX4/px4_msgs.git "$ROS_WS_SRC/px4_msgs"
else
    info "px4_msgs already present."
fi

# Clone px4_ros_com
if [ ! -d "$ROS_WS_SRC/px4_ros_com/.git" ]; then
    info "Cloning px4_ros_com..."
    git clone https://github.com/PX4/px4_ros_com.git "$ROS_WS_SRC/px4_ros_com"
else
    info "px4_ros_com already present."
fi

# Clone mobile_flight itself if not already there
MOBILE_FLIGHT_DIR="$ROS_WS_SRC/mobile_flight"
if [ ! -d "$MOBILE_FLIGHT_DIR/.git" ]; then
    info "Cloning mobile_flight..."
    git clone https://github.com/EESC-LabRoM/mobile_flight.git "$MOBILE_FLIGHT_DIR"
else
    info "mobile_flight already present."
fi

# =============================================================================
# 5. INSTALL ROS 2 DEPS & BUILD WORKSPACE
# =============================================================================
info "=== [5/6] Installing ROS 2 package dependencies & building workspace ==="

# ROS setup.bash uses variables like AMENT_TRACE_SETUP_FILES without
# initializing them — temporarily disable 'unbound variable' check.
set +u
source /opt/ros/humble/setup.bash
set -u

cd "$IMAV_WS"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers "$(nproc)"

# =============================================================================
# 6. CONFIGURE ~/.bashrc
# =============================================================================
info "=== [6/6] Configuring ~/.bashrc ==="

BASHRC="$HOME/.bashrc"
MARKER="# >>> mobile_flight env >>>"

if ! grep -q "$MARKER" "$BASHRC"; then
    cat >> "$BASHRC" << 'EOF'

# >>> mobile_flight env >>>
source /opt/ros/humble/setup.bash

# Source imav_ws overlay if already built
if [ -f "$HOME/imav_ws/install/setup.bash" ]; then
    source "$HOME/imav_ws/install/setup.bash"
fi

# Micro XRCE-DDS Agent is installed to /usr/local/bin — no extra PATH needed

# Convenience aliases
alias build_ws="cd ~/imav_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash"
alias source_ws="source ~/imav_ws/install/setup.bash"
alias xrce_usb="MicroXRCEAgent serial --dev /dev/ttyACM0 -b 3000000"
alias xrce_udp="MicroXRCEAgent udp4 -p 8888"
# <<< mobile_flight env <<<
EOF
    info ".bashrc updated."
else
    info ".bashrc already configured — skipping."
fi

# =============================================================================
# DONE
# =============================================================================
echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}  Installation complete!${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""
echo "  Next steps:"
echo "  1. Open a new terminal (or run: source ~/.bashrc)"
echo "  2. Verify ROS 2: ros2 topic list"
echo "  3. Start uXRCE-DDS Agent via USB:  xrce_usb"
echo "     or via UDP (SITL):              xrce_udp"
echo "  4. Build the workspace:            build_ws"
echo "  5. Run the simulation:            ros2 launch mobile_gazebo simulation.launch.py"
echo ""
echo "  Workspace: $IMAV_WS"
