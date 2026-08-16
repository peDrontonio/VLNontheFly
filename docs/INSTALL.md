# Install: clean clone to a build

Target platform is the one the system was validated on: **Ubuntu 22.04 +
ROS 2 Humble**, on a Jetson Orin NX for flight.

## 1. Prerequisites

- ROS 2 Humble, plus `python3-vcstool` (`sudo apt install python3-vcstool`)
- [PX4](https://px4.io/) firmware on the flight controller, and the
  Micro XRCE-DDS agent for the ROS 2 ⟷ PX4 bridge
- Intel RealSense SDK `librealsense2` **2.56.6** — the driver's CMake pin was
  lowered to match this version, see [`DEPENDENCIES.md`](DEPENDENCIES.md)
- A TensorRT Edge-LLM build, for the quantized Qwen engine used by
  `edgellm_vlm_ros`

## 2. Clone and fetch third-party sources

```bash
git clone https://github.com/peDrontonio/VLNontheFly.git
cd VLNontheFly

vcs import < third_party.repos
```

That second command is not optional — `px4_msgs` and the NavDP depth backbone
are fetched rather than vendored. Skipping it gives you a workspace that is
missing PX4 message definitions.

## 3. Build

```bash
source /opt/ros/humble/setup.bash
source /home/orin/ros2_ws/install/setup.bash   # your PX4 / Micro-XRCE-DDS workspace, if separate

# camera, planner, and PX4 bridge
colcon build --packages-select \
  realsense2_camera plan_env ego_planner planner

# VLM package, built separately so its TensorRT options do not leak
# into the other packages
colcon build --packages-select edgellm_vlm_ros \
  --cmake-args \
    -DEDGELLM_VLM_ENABLE_EDGELLM=ON \
    -DEDGELLM_SOURCE_DIR=/path/to/TensorRT-Edge-LLM \
    -DEDGELLM_BUILD_DIR=/path/to/TensorRT-Edge-LLM/build \
    -DTRT_PACKAGE_DIR=/usr

source install/setup.bash
```

This repository doubles as the `colcon` workspace root, so packages sit
directly under it rather than in a nested `src/`.

### Building without model weights

The validated flow needs no downloaded weights, so a clean clone builds as-is.
The experimental NavDP/iPlanner paths need two large checkpoints, and their
install rules are guarded — when the directories are absent you get

```
-- planner: checkpoints/ not found -- skipping.
   Run scripts/fetch_models.sh if you need NavDP or iPlanner.
```

which is a status line, not an error. [`DEPENDENCIES.md`](DEPENDENCIES.md) has
the download list.

## 4. Verify the build

```bash
pytest tests/                    # 45 unit tests, no hardware
```

Then follow [`TESTING.md`](TESTING.md) before putting props on anything.

## 5. Run

```bash
ros2 launch planner ego_raptor.launch.py \
  start_planner:=true \
  with_xrce:=true \
  with_optitrack:=true \
  with_realsense:=true \
  with_relative_goal:=true \
  set_external:=false
```

then the VLM in observation mode:

```bash
ros2 launch edgellm_vlm_ros d435i_vlm.launch.py \
  prompt_mode:=region \
  enable_region_gate:=true \
  region_gate_params_file:=install/edgellm_vlm_ros/share/edgellm_vlm_ros/config/region_gate.yaml
```

`set_external:=false` keeps PX4 `EXTERNAL` mode inactive. Bring the stack up in
stages, props off and disarmed, and read
[`HARDWARE_BRINGUP.md`](HARDWARE_BRINGUP.md) before changing that.
