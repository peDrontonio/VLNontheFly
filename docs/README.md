# VLN on the Fly — workspace documentation

Start here. The root [`README.md`](../README.md) is the project's front page
(what the system is, the pipeline, results, citation). This tree is the
operational documentation: how to build it, what it needs downloaded, how to
test it, and how to fly it.

| Document | Answers |
|---|---|
| [`INSTALL.md`](INSTALL.md) | How do I get from a clean clone to a build? |
| [`DEPENDENCIES.md`](DEPENDENCIES.md) | What is fetched, what is vendored, which model weights do I need and where do they come from? |
| [`TESTING.md`](TESTING.md) | How do I test any component, from a laptop up to a real flight? |
| [`HARDWARE_BRINGUP.md`](HARDWARE_BRINGUP.md) | The staged hardware bring-up plan (Stage 0 bench → Stage 3 full stack). |
| [`RUNBOOK_FIG8.md`](RUNBOOK_FIG8.md) | Step-by-step runbook for the figure-8 flight, including abort criteria. |
| [`EXTERNAL_MODE_STATUS.md`](EXTERNAL_MODE_STATUS.md) | State of the PX4 `EXTERNAL` mode / Raptor integration, and what is still open. |
| [`imav_bt/`](imav_bt/) | The IMAV behaviour-tree mission controller: architecture, tree semantics, interfaces, and how to add a behaviour or a mission. **Not flight-validated.** |

---

## Validation status

**Not everything in this workspace has been flown.** The 15-flight evaluation in
the IMAV 2026 paper exercised one specific path through the stack. Other
packages here are research alternatives that share the repo but not the
evidence. Treat this table as authoritative before you arm anything.

### Flight-validated

Flown on the real airframe, 15 open-volume flights plus 6 cluttered-environment
trials. This is the path `ego_raptor.launch.py` takes.

| Component | Role in the validated flow |
|---|---|
| `edgellm_vlm_ros` | INT4 Qwen-3.5-2B grounding, **region** mode + region gate |
| `ego-planner-swarm/src/planner` | `ego_planner_node` and `traj_server` (B-spline planning, occupancy map) |
| `ego-planner-swarm/raptor_path_tracker.py` | bench-only carrot tracker; the flight path uses `pos_cmd` |
| `ego-planner-swarm/src/quadrotor_msgs` | message definitions `plan_manage` depends on |
| `planner_wrapper` | `odometry_converter.py`, `pos_cmd_to_raptor.py`, `relative_goal_to_map.py`, `set_external_mode.py` |
| `vio_bridge` → `optitrack_bridge_node` | OptiTrack pose into the PX4 EKF |
| `realsense-ros/realsense2_camera` | D435i driver, **including our mount-TF patch** (see `DEPENDENCIES.md`) |
| `px4_msgs` | PX4 v1.16.2 interfaces |

### Experimental — never flown

Present, buildable, and useful research paths. None of them has flight
evidence, and some have never run at all.

| Component | Status |
|---|---|
| `planner_wrapper` → `navdp.launch.py` | **Never run.** The NavDP checkpoint is an unresolved Git LFS pointer; the weights were never downloaded. |
| `planner_wrapper` → `iplanner*.launch.py` | **Never run.** Same — `iplanner.pt` is an LFS pointer. |
| `depth_estimator` | Monocular Depth-Anything-V2 as an alternative to D435i stereo depth. Not used in any validated run. |
| `vio_bridge` → `openvins_bridge_node`, `vio_bridge_node` | The path toward replacing OptiTrack with onboard localization. Not flown. |
| `mobile_flight` | Gazebo SITL and PX4 offboard velocity control. **Simulation only.** |
| `edgellm_vlm_ros` → `point` / `primitive` modes | Only `region` mode was evaluated. The others are implemented and unit-tested but not flown. |
| `imav_bt` | **Never flown.** The IMAV 2026 behaviour-tree mission controller. Ticks end-to-end against `scripts/mock_arena.py` and has 71 unit tests, but has never commanded a real aircraft, and the per-mission logic is deliberately left as documented stubs. It publishes goals to ego-planner and never writes actuator commands, so it cannot bypass the validated stack's safety layer. |

### Why this distinction is enforced, not just documented

- Experimental launch files are named separately and never included by
  `ego_raptor.launch.py`.
- The optional model directories they need are guarded in
  `planner_wrapper/CMakeLists.txt`, so their absence cannot break a build of
  the validated flow.
- Changes to validated and non-validated components are kept in separate pull
  requests, so a reviewer can approve the flown half without inheriting the
  rest.
