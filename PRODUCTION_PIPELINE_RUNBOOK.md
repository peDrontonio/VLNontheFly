# Production Pipeline Runbook

This runbook covers only the active production stack:

```text
OptiTrack -> PX4 EKF -> /fmu/out/vehicle_odometry
                         |
                         v
                  map -> base_link TF
                         |
RealSense -> base_link -> camera_link -> camera_depth_optical_frame
                         |
depth + CameraInfo + timestamped TF -> EGO-Planner occupancy -> trajectory
RGB -> EdgeLLM VLM -> proposal -> /relative_goal -> planner goal
trajectory -> Raptor EXTERNAL bridge -> PX4
```

Do not launch any `mobile_gazebo` flight stack alongside this pipeline. It can
duplicate odometry, planner, or controller publishers.

## Current Validation Boundary

The production EGO-Planner depth path now:

- indexes the current sampled depth pixel;
- consumes matching depth `CameraInfo` and actual image dimensions;
- looks up `map <- camera_depth_optical_frame` at the depth image timestamp;
- drops the image when that timestamped transform is unavailable;
- never falls back to the latest vehicle pose.

The VLM synchronization patch is not implemented yet. The region gate still
combines the latest VLM result, latest depth, and latest pose. You may launch it
and inspect proposals, but do not enable automatic execution or use it for an
autonomous flight until RGB/depth/result timestamp propagation is complete.

## Safety State

Run Sections 1 through 8 with:

- props removed;
- vehicle disarmed;
- Raptor RC switch off;
- EXTERNAL mode inactive;
- `set_external:=false`.

Do not proceed to flight merely because the software launches. The synthetic
tests and the physical front/left/right/vertical obstacle checks must pass first.

## 1. Terminal Environment

Run this in every terminal:

```bash
cd /home/orin/VLNontheFly
source /opt/ros/humble/setup.bash
source /home/orin/ros2_ws/install/setup.bash
source /home/orin/VLNontheFly/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
```

Check that every terminal agrees:

```bash
echo "$RMW_IMPLEMENTATION $ROS_DOMAIN_ID $ROS_LOCALHOST_ONLY"
```

## 2. Build

Build the camera, planner, and wrapper:

```bash
cd /home/orin/VLNontheFly
source /opt/ros/humble/setup.bash
source /home/orin/ros2_ws/install/setup.bash

colcon build --packages-select \
  realsense2_camera plan_env ego_planner planner
```

Build the VLM separately so its TensorRT options are not passed to unrelated
packages:

```bash
colcon build --packages-select edgellm_vlm_ros \
  --cmake-args \
    -DEDGELLM_VLM_ENABLE_EDGELLM=ON \
    -DEDGELLM_SOURCE_DIR=/home/orin/TensorRT-Edge-LLM \
    -DEDGELLM_BUILD_DIR=/home/orin/TensorRT-Edge-LLM/build \
    -DTRT_PACKAGE_DIR=/usr

source /home/orin/VLNontheFly/install/setup.bash
```

## 3. Run Synthetic Projection Tests

```bash
cd /home/orin/VLNontheFly
source /opt/ros/humble/setup.bash
source /home/orin/ros2_ws/install/setup.bash
source install/setup.bash

build/plan_env/test_depth_projection
```

Expected result:

```text
[  PASSED  ] 5 tests.
```

These tests cover invalid-depth handling, optical-to-FLU axis behavior, live
image capacity, and selection of an older requested TF timestamp instead of the
latest transform.

## 4. Launch the Hardware and Planner Pipeline

Before launching, confirm that no bridge from an earlier session survived:

```bash
pgrep -af optitrack_bridge_node
ros2 node list | grep '^/optitrack_bridge_node$'
```

Both commands should print nothing before launch. If a launch was interrupted
but its bridge survived, terminate that stale process before continuing. After
launch, exactly one `optitrack_bridge_node` process and one ROS node with that
name must exist.

First confirm the live OptiTrack topic name:

```bash
ros2 topic list | grep '^/base_link/pose$'
ros2 topic info /base_link/pose
ros2 topic echo --once /base_link/pose --field header
```

`/base_link/pose` means "pose of base_link." Its `PoseStamped.header.frame_id`
must identify the OptiTrack world/reference frame. Do not set that header to
`base_link`, because that would incorrectly describe base_link relative to
itself.

Use the actual topic below. This command starts XRCE, the OptiTrack bridge,
RealSense, odometry conversion, EGO-Planner, the Raptor command bridge, and the
relative-goal adapter:

```bash
ros2 launch planner ego_raptor.launch.py \
  start_planner:=true \
  with_xrce:=true \
  with_optitrack:=true \
  with_realsense:=true \
  with_relative_goal:=true \
  set_external:=false \
  camera_mount_x:=0.155 \
  camera_mount_y:=0.0 \
  camera_mount_z:=0.0 \
  camera_mount_roll_deg:=0.0 \
  camera_mount_pitch_deg:=7.0 \
  camera_mount_yaw_deg:=0.0
```

`camera_mount_z:=0.0` and `camera_mount_pitch_deg:=7.0` are configuration
values, not measured calibration results. Record that limitation in the test
record.

Startup acceptance:

- RealSense reports `RealSense Node Is Up!`.
- `/odometry` becomes live.
- EGO-Planner logs the configured and live depth intrinsics once.
- Repeated `Dropping depth frame: no timestamped TF` messages stop after TF and
  odometry startup. A few initial drops are acceptable.
- There is no second 180-degree yaw correction anywhere in the launch command.

Confirm the active runtime parameters rather than trusting the launch source:

```bash
ros2 param get /drone_0_ego_planner_node grid_map/use_tf_camera_pose
ros2 param get /drone_0_ego_planner_node grid_map/frame_id
ros2 param get /camera/camera publish_mount_tf
ros2 param get /camera/camera mount_parent_frame_id
ros2 param get /camera/camera mount_x
ros2 param get /camera/camera mount_pitch
```

Expected values are `true`, `map`, `true`, `base_link`, `0.155`, and
approximately `0.122173` radians respectively.

## 5. Verify Topics and Message Metadata

Run these in another configured terminal:

```bash
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic hz /odometry
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/depth/camera_info
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /drone_0_grid/grid_map/occupancy_inflate
```

Inspect the depth metadata:

```bash
ros2 topic echo --once /camera/camera/depth/image_rect_raw \
  --field header

ros2 topic echo --once /camera/camera/depth/camera_info
```

Pass conditions:

- depth image and depth `CameraInfo` use the same frame ID;
- expected frame is normally `camera_depth_optical_frame`;
- image width/height equal `CameraInfo.width/height`;
- `K[0]`, `K[4]`, `K[2]`, and `K[5]` are finite and nonzero;
- the planner startup log reports the live/configured intrinsic delta;
- occupancy output continues at a stable rate after startup.

## 6. Verify the TF Chain

Check each required edge:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame
ros2 run tf2_ros tf2_echo map camera_depth_optical_frame
```

Optional latency inspection:

```bash
ros2 run tf2_ros tf2_monitor map camera_depth_optical_frame
```

Expected physical interpretation in ROS FLU:

| Observation | Expected sign in `base_link` |
|---|---:|
| Camera center in front of base | `x ~= +0.155 m` |
| Point in front of image | `+X` |
| Point on image left | `+Y` |
| Point on image right | `-Y` |
| Point above camera | `+Z` |
| Point below camera | `-Z` |

Stop if `map -> camera_depth_optical_frame` is unavailable, jumps by 180
degrees, or puts the camera at approximately `x=-0.155 m` in `base_link`.

## 7. Start Foxglove

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=0.0.0.0 \
  port:=8765
```

Connect Foxglove to:

```text
ws://JETSON_IP:8765
```

Create a 3D panel with fixed frame `map` and add:

- TF frames;
- `/drone_0_grid/grid_map/occupancy_inflate` as a point cloud;
- the drone pose from `/odometry`;
- planner goal and trajectory visualization topics.

Create image panels for:

- `/camera/camera/color/image_raw`;
- `/camera/camera/depth/image_rect_raw`.

The Foxglove warnings about RealSense IDL-only service definitions do not block
TF, image, CameraInfo, point-cloud, or planner topic visualization.

## 8. Physical Obstacle Placement Test

Keep the vehicle stationary, disarmed, and approximately yaw zero in the test
area. Use one compact obstacle with a clear depth return, approximately 1 m from
the camera. Move the same object through these positions:

| Placement | Expected occupancy relative to `base_link` |
|---|---|
| Front | `+X` |
| Left | `+Y` |
| Right | `-Y` |
| Above camera center | `+Z` |
| Below camera center | `-Z` |

For every placement:

1. Confirm the object is visible in RGB and depth.
2. Confirm its inflated occupancy appears on the expected side in Foxglove.
3. Confirm front never appears behind the drone.
4. Confirm left and right are not reversed.
5. Record approximate expected and observed coordinates.

Timestamp stress test:

1. Leave the obstacle fixed in the room.
2. Slowly carry or rotate the disarmed drone while OptiTrack is active.
3. Observe the obstacle in the `map` fixed frame.
4. It should remain approximately fixed instead of following the drone or
   producing a systematic latest-pose smear.

Small sensor noise is expected. A persistent offset, mirrored side, 180-degree
rotation, or motion correlated with the drone is a failure.

## 9. Test Planner Goals Without VLM Execution

Watch planner status and commands:

```bash
ros2 topic echo /drone_0_planning/goal_status
ros2 topic echo /drone_0_planning/pos_cmd
```

In Foxglove, add a `MarkerArray` visualization for
`/drone_0_plan_vis/goal_gate` with the scene fixed frame set to `map`. The
green box is the allowed goal volume; red cylinders are keep-out regions. The
topic uses transient-local durability, so the markers remain available to a
Foxglove session that connects after planner startup.

With props removed and EXTERNAL inactive, publish a short body-relative goal:

```bash
ros2 topic pub --once /relative_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.75, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Repeat with a known obstacle on the left and then on the right. In Foxglove,
verify that the generated trajectory bends away from the occupied side. Do not
judge avoidance from visualization alone; confirm the occupancy and trajectory
are both in `map`.

## 10. Launch the VLM in Observation Mode

The RealSense is already owned by `ego_raptor.launch.py`; do not start another
camera node.

Launch region mode with the bench-safe region-gate configuration. Specifying
`region_gate.yaml` is required here because the launch file's default region
configuration is the auto-executing pipeline preset.

```bash
ros2 launch edgellm_vlm_ros d435i_vlm.launch.py \
  prompt_mode:=region \
  enable_region_gate:=true \
  region_gate_params_file:=/home/orin/VLNontheFly/install/edgellm_vlm_ros/share/edgellm_vlm_ros/config/region_gate.yaml
```

Inspect without calling `execute_next`:

```bash
ros2 topic echo /edgellm_vlm_node/result
ros2 topic echo /vlm_region_gate/proposal
ros2 topic echo /vlm_region_gate/status
ros2 param get /edgellm_vlm_node prompt_mode
ros2 param get /vlm_region_gate auto_execute
ros2 param get /vlm_region_gate selection_mode
```

Pass conditions for this stage:

- inference receives the expected 640x480 RGB image;
- the VLM result contains one valid 3x3 region name and a confidence value;
- the region gate reports `auto_execute` as `false` and `selection_mode` as
  `open_space`;
- `TOP`, `MIDDLE`, and `BOTTOM` map to the expected image rows;
- `LEFT`, `CENTER`, and `RIGHT` map to the expected image columns;
- the proposal JSON reports the same selected region, or explicitly reports a
  depth fallback;
- no `/relative_goal` is published without an explicit execute request;
- automatic supervisor execution remains disabled.

Do not call this yet for flight validation:

```text
/vlm_region_gate/execute_next
```

It currently associates independently latest VLM, depth, and pose data. That is
the next production patch.

## 11. Record a Diagnostic Bag

Start this before the obstacle tests:

```bash
mkdir -p /home/orin/VLNontheFly/test_bags

ros2 bag record -o /home/orin/VLNontheFly/test_bags/tf_depth_projection \
  /tf \
  /tf_static \
  /odometry \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /drone_0_grid/grid_map/occupancy_inflate \
  /relative_goal \
  /move_base_simple/goal \
  /drone_0_planning/pos_cmd \
  /drone_0_planning/goal_status
```

Stop recording with Ctrl-C after all six obstacle directions and the moving
drone/fixed-obstacle test.

## 12. Go/No-Go Criteria

The depth/TF patch passes bench validation only when all are true:

- all five synthetic tests pass;
- live depth and CameraInfo frames/dimensions match;
- the complete `map -> base_link -> camera -> depth optical` chain exists;
- camera translation is approximately `+0.155 m` in base X;
- front/left/right/above/below occupancy signs are correct;
- a fixed obstacle remains fixed in `map` while the drone moves slowly;
- planner trajectories avoid occupancy on the correct physical side;
- no repeated timestamped-TF drops occur during steady operation.

Autonomous VLM flight remains **NO-GO** until:

- aligned/synchronized RGB and depth are used by the VLM;
- the original RGB timestamp is propagated through the VLM result;
- corresponding depth and pose are selected by that timestamp;
- the physical obstacle-placement test is repeated after those changes;
- the existing Raptor hardware-flight safety stages are passed.
