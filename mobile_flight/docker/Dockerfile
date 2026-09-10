###############################################################################
# PX4 + ROS 2 Humble + Gazebo Harmonic + Micro XRCE-DDS Agent
# Docker environment for UAV development & SITL simulation
###############################################################################
FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-c"]

# ── System dependencies ─────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
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
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    gdb \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (needs root; update will run as user later)
RUN rosdep init 2>/dev/null || true

# ── Gazebo Harmonic ─────────────────────────────────────────────────────────
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

# ── PX4 Build Dependencies ─────────────────────────────────────────────────
# Install PX4 toolchain dependencies (without cloning PX4 yet — that is done
# inside the container at runtime so the source stays on a mounted volume).
RUN apt-get update && apt-get install -y --no-install-recommends \
    astyle \
    ninja-build \
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
    unzip \
    && pip3 install --no-cache-dir pyros-genmsg pyulog kconfiglib \
    && rm -rf /var/lib/apt/lists/*

# ── Micro XRCE-DDS Agent ───────────────────────────────────────────────────
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cd /tmp/Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig /usr/local/lib/ \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

# ── ROS 2 additional packages ──────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gzharmonic-bridge \
    ros-humble-ros-gzharmonic-image \
    ros-humble-actuator-msgs \
    && rm -rf /var/lib/apt/lists/*

# ── Create a non-root user ─────────────────────────────────────────────────
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME 2>/dev/null || true \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 2>/dev/null || true \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f /home/$USERNAME/imav_ws/install/setup.bash ]; then source /home/$USERNAME/imav_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /home/$USERNAME/.bashrc

# ── rosdep update (must run as non-root user) ─────────────────────────────
RUN rosdep update

# ── Workspace directory ────────────────────────────────────────────────────
RUN mkdir -p ~/imav_ws/src

# ── Entrypoint ─────────────────────────────────────────────────────────────
COPY --chown=$USERNAME:$USERNAME entrypoint.sh /home/$USERNAME/entrypoint.sh
RUN chmod +x ~/entrypoint.sh

ENTRYPOINT ["/home/user/entrypoint.sh"]
CMD ["bash"]
