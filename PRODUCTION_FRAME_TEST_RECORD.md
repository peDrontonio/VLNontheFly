# Production Frame Test Record

Use this file with `PRODUCTION_PIPELINE_RUNBOOK.md`.

## Session

| Field | Value |
|---|---|
| Date/time | |
| Operator | |
| Git commit | |
| ROS domain ID | `30` |
| OptiTrack topic | `/base_link/pose` |
| OptiTrack reference frame (`header.frame_id`) | |
| RealSense serial | |
| Depth resolution/rate | |
| Camera mount translation | `[0.155, 0.0, 0.0] m` configured |
| Camera mount pitch | `+7.0 deg` configured, not calibrated |
| Camera vertical offset | Unknown/unmeasured |

## Safety

- [ ] Props removed
- [ ] Vehicle disarmed
- [ ] Raptor RC switch off
- [ ] EXTERNAL inactive
- [ ] `set_external:=false`
- [ ] No duplicate mobile flight stack running

## Build and Synthetic Tests

- [ ] `realsense2_camera` built
- [ ] `plan_env` built
- [ ] `ego_planner` built
- [ ] `planner` built
- [ ] `edgellm_vlm_ros` built
- [ ] Five `test_depth_projection` tests passed
- [ ] Production launch loaded without errors

Notes:

```text

```

## Live CameraInfo

| Value | Launch constant | Live CameraInfo | Delta |
|---|---:|---:|---:|
| Width | `640` assumed by current VLM prompt | | |
| Height | `480` assumed by current VLM prompt | | |
| `fx` | `382.613` | | |
| `fy` | `382.613` | | |
| `cx` | `320.183` | | |
| `cy` | `236.455` | | |
| Frame ID | N/A | | N/A |

- [ ] Depth image and CameraInfo stamps match
- [ ] Depth image and CameraInfo frame IDs match
- [ ] Planner logged configured/live intrinsic delta
- [ ] Planner continued using the live values

## TF Chain

| Transform | Translation | Rotation/RPY | Pass |
|---|---|---|---|
| `map -> base_link` | | | [ ] |
| `base_link -> camera_link` | | | [ ] |
| `camera_link -> camera_depth_optical_frame` | | | [ ] |
| `map -> camera_depth_optical_frame` | | | [ ] |

- [ ] `base_link -> camera_link` X is approximately `+0.155 m`
- [ ] No extra 180-degree yaw is visible
- [ ] No conflicting TF publisher is present
- [ ] Timestamped TF drops stop after startup

## Physical Obstacle Results

Record coordinates in the `map` fixed frame. For a yaw-zero vehicle, the
expected relative signs are expressed in `base_link` FLU.

| Placement | Expected sign | Expected map position | Observed map position | Pass |
|---|---|---|---|---|
| Front | `+X_base` | | | [ ] |
| Left | `+Y_base` | | | [ ] |
| Right | `-Y_base` | | | [ ] |
| Above | `+Z_base` | | | [ ] |
| Below | `-Z_base` | | | [ ] |

Failure observations:

```text

```

## Timestamp Stress Test

| Measurement | Result |
|---|---|
| Fixed obstacle initial map coordinate | |
| Fixed obstacle coordinate after translating drone | |
| Fixed obstacle coordinate after yawing drone | |
| Approximate maximum smear/offset | |

- [ ] Obstacle remained fixed in `map`
- [ ] Obstacle did not follow current drone pose
- [ ] No systematic yaw-dependent arc appeared
- [ ] No repeated TF extrapolation warnings occurred

## Planner Response

| Scenario | Expected response | Observed response | Pass |
|---|---|---|---|
| Clear forward goal | Trajectory forward | | [ ] |
| Obstacle front-left | Avoid right | | [ ] |
| Obstacle front-right | Avoid left | | [ ] |
| Obstacle directly front | Stop or route around | | [ ] |
| Obstacle above/below planned altitude | Correct 3D clearance behavior | | [ ] |

- [ ] Goal, occupancy, and trajectory all use `map`
- [ ] Positive/negative lateral direction is physically correct
- [ ] Planner never selects a path behind due to a hidden 180-degree offset

## VLM Observation Only

- [ ] VLM receives live RGB
- [ ] Color CameraInfo reports the dimensions used to validate proposals
- [ ] Proposal pixels remain inside those dimensions
- [ ] Left-image proposal remains left
- [ ] Right-image proposal remains right
- [ ] Point gate does not publish without manual execution
- [ ] Supervisor auto-execution remains disabled
- [ ] No flight execution attempted before VLM timestamp patch

Example result/proposal:

```json

```

## Final Decision

### Depth/TF Planner Patch

- [ ] PASS
- [ ] FAIL

### Autonomous VLM Flight

- [ ] NO-GO: RGB/depth/result timestamp patch still pending
- [ ] GO only after that patch and repeated physical validation

Blocking issues:

```text

```
