# edgellm_vlm_ros

ROS 2 package for running TensorRT Edge-LLM VLM inference on D435i RGB images and turning the result into ego-planner goals.

Implemented modes:

- Point mode: VLM selects an RGB pixel. The gate uses D435i depth to publish a fixed-z `/move_base_simple/goal`.
- Region mode: VLM selects a 3x3 image cell. The gate pairs it with aligned depth and
  historical TF, then publishes a stable map-frame planner goal.
- Primitive mode: VLM selects a short movement primitive such as `FORWARD`, `LEFT`, or `UP`.
- Supervised mode: one VLM runtime switches between point prompts and altitude-primitive prompts.

The default drone pose source is PX4:

```text
/fmu/out/vehicle_local_position
```

This topic is `px4_msgs/msg/VehicleLocalPosition` in NED. Point and primitive gates produce
body-relative `base_link` goals, which `relative_goal_to_map` composes with ENU odometry.
Region mode instead uses the RGB timestamp and the TF history to publish directly in `map`.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select edgellm_vlm_ros \
  --cmake-args \
    -DEDGELLM_VLM_ENABLE_EDGELLM=ON \
    -DEDGELLM_SOURCE_DIR=/home/orin/TensorRT-Edge-LLM \
    -DEDGELLM_BUILD_DIR=/home/orin/TensorRT-Edge-LLM/build \
    -DTRT_PACKAGE_DIR=/usr
source install/setup.bash
```

## Bag Replay

For `bag_raptor`, replay the topics needed by the VLM, D435i depth projection, and PX4 pose:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play ~/ros2_ws/bag_raptor \
  --topics /camera/camera/color/image_raw \
           /camera/camera/color/camera_info \
           /camera/camera/depth/image_rect_raw \
           /camera/camera/depth/camera_info \
           /fmu/out/vehicle_local_position
```

## Point Mode

Use this as the main horizontal navigation mode.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch edgellm_vlm_ros d435i_vlm.launch.py enable_point_gate:=true
```

The VLM should return JSON only:

```json
{"u":320,"v":260,"confidence":0.82}
```

Inspect:

```bash
ros2 topic echo /edgellm_vlm_node/result
ros2 topic echo /vlm_point_gate/proposal
ros2 topic echo /vlm_point_gate/status
```

Execute the latest accepted point:

```bash
ros2 service call /vlm_point_gate/execute_next std_srvs/srv/Trigger {}
```

Output goal:

```text
/move_base_simple/goal
```

Point goals keep the current z by default:

```yaml
goal_z_mode: "current_pose"
```

To force one global z plane, set:

```yaml
goal_z_mode: "fixed"
fixed_goal_z_m: 1.0
```

## Region Mode

Region mode keeps the paper's 3x3 cell decision while making the planner handoff
timestamped and stateful:

```bash
ros2 launch edgellm_vlm_ros d435i_vlm.launch.py \
  prompt_mode:=region \
  enable_region_gate:=true \
  region_gate_params_file:=$PWD/install/edgellm_vlm_ros/share/edgellm_vlm_ros/config/region_gate.yaml
```

For named-object navigation, omit that override (the launch default is the
target preset) and pass `target_object:="orange trash bin"` or another target.

The production path is:

```text
RGB cell + nearest aligned-depth frame
  -> camera optical point
  -> base_link at the RGB timestamp
  -> map at the RGB timestamp
  -> /move_base_simple/goal
```

The gate requires `consistency_required` consecutive decisions for the same
cell. Once published, a goal remains active until `/planning/goal_status`
reports `reached`, `rejected:*`, or `failed:*`, or until
`safety_loss_required` depth checks mark the active cell unsafe. A candidate
within `goal_update_min_distance_m` of the previous goal is suppressed.

Inspect the arbitration state with:

```bash
ros2 topic echo /vlm_region_gate/proposal
ros2 topic echo /vlm_region_gate/status
ros2 topic echo /planning/goal_status
ros2 topic echo /move_base_simple/goal
```

## Supervised Mode

Use this for the combined pipeline. It starts both gates and one supervisor. The supervisor keeps the VLM in point mode by default, switches to primitive mode after repeated point failures, executes one `UP`, `DOWN`, or `HOLD`, waits briefly, then returns to point mode.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch edgellm_vlm_ros d435i_vlm.launch.py enable_nav_supervisor:=true
```

Inspect:

```bash
ros2 topic echo /vlm_nav_supervisor/state
ros2 topic echo /vlm_nav_supervisor/status
ros2 topic echo /edgellm_vlm_node/result
```

Execute one supervisor step manually:

```bash
ros2 service call /vlm_nav_supervisor/step std_srvs/srv/Trigger {}
```

Enable automatic execution:

```bash
ros2 service call /vlm_nav_supervisor/set_auto_execute std_srvs/srv/SetBool "{data: true}"
```

The supervisor switches the single VLM runtime with:

```text
/edgellm_vlm_node/set_point_mode
```

`true` means point prompt. `false` means altitude primitive prompt.

## Primitive Mode

Use this for simple fallback moves and altitude changes.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch edgellm_vlm_ros d435i_vlm.launch.py enable_primitive_gate:=true
```

The VLM should return JSON only:

```json
{"primitive":"FORWARD","distance_m":0.5,"confidence":0.78}
```

Allowed primitives:

```text
HOLD, FORWARD, BACK, LEFT, RIGHT, UP, DOWN
```

Inspect:

```bash
ros2 topic echo /vlm_primitive_gate/proposal
ros2 topic echo /vlm_primitive_gate/status
```

Execute the latest accepted primitive:

```bash
ros2 service call /vlm_primitive_gate/execute_next std_srvs/srv/Trigger {}
```

## Config

Config files are split by node:

```text
src/edgellm_vlm_ros/config/vlm_node.yaml
src/edgellm_vlm_ros/config/point_gate.yaml
src/edgellm_vlm_ros/config/primitive_gate.yaml
src/edgellm_vlm_ros/config/region_gate.yaml
src/edgellm_vlm_ros/config/region_gate_pipeline.yaml
src/edgellm_vlm_ros/config/nav_supervisor.yaml
```

Launch overrides:

```bash
ros2 launch edgellm_vlm_ros d435i_vlm.launch.py \
  vlm_params_file:=/path/to/vlm_node.yaml \
  point_gate_params_file:=/path/to/point_gate.yaml \
  primitive_gate_params_file:=/path/to/primitive_gate.yaml \
  region_gate_params_file:=/path/to/region_gate.yaml \
  nav_supervisor_params_file:=/path/to/nav_supervisor.yaml
```

Important defaults:

```yaml
image_topic: "/camera/camera/color/image_raw"
depth_topic: "/camera/camera/depth/image_rect_raw"
depth_camera_info_topic: "/camera/camera/depth/camera_info"
vehicle_local_position_topic: "/fmu/out/vehicle_local_position"
goal_topic: "/relative_goal"
goal_frame_id: "base_link"
prompt_mode: "point"
auto_execute: false
```

Region production defaults override that common/legacy contract:

```yaml
depth_topic: "/camera/camera/aligned_depth_to_color/image_raw"
goal_output_mode: "map"
goal_topic: "/move_base_simple/goal"
body_frame_id: "base_link"
map_frame_id: "map"
consistency_required: 3
wait_for_planner_status: true
```

Optional fallback pose sources are disabled by default:

```yaml
odometry_topic: ""
pose_topic: ""
```

## Tests

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select edgellm_vlm_ros --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/edgellm_vlm_ros
```
