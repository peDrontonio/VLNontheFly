# Hardware Test Plan — ego-planner → Raptor EXTERNAL bring-up

All individual parts are tested (firmware flashed + SITL-validated EXTERNAL mode, isolated
ego-planner, odometry converter, pos_cmd bridge smoke-tested on the Jetson at 100 Hz).
What remains is proving the **integrated chain on hardware**, isolating one new variable per
stage. Abort at any stage = release the RC switch → `Switching to original controller` → PX4 manual.

```
Stage 1: Raptor regression          (firmware only — no companion, no EXTERNAL)
Stage 2: EXTERNAL + fixed figure-8  (production pipeline, ego-planner swapped for fig8_pos_cmd)
Stage 3: EXTERNAL + ego-planner     (full stack, VLM goals; gate setup in "Stage 3 prep")
```

Rationale for Stage 2: the figure-8 is streamed as `PositionCommand` on
`/drone_0_planning/pos_cmd` — the *exact* interface, bridge, DDS link, QoS and 100 Hz rate the
real flight uses. If Stage 2 tracks well and Stage 3 misbehaves, the problem is in the planner
(map/odometry/goals), not in the transport or the policy.

## Launch ownership and caveats

Use the same ROS environment in every Jetson terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/orin/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
```

`planner ego_raptor.launch.py` is the Raptor EXTERNAL wrapper, not the whole old
`mobile_gazebo ego_planner_flight.launch.py` stack. It always starts:

- `/fmu/out/vehicle_odometry` → `/odometry`
- `/drone_0_planning/pos_cmd` → `/fmu/in/trajectory_setpoint_raptor`
- optionally `ego_planner` (`start_planner:=true`)

It can now also start the hardware prerequisites, but they are **off by default** so tests can
isolate one variable at a time:

```bash
ros2 launch planner ego_raptor.launch.py \
  with_xrce:=true \
  with_optitrack:=true pose_topic:=/drone/pose \
  with_realsense:=true \
  set_external:=false
```

Important defaults:

- `set_external:=false`: MAVProxy usually owns `/dev/ttyACM0` for QGC, so set EXTERNAL from
  QGC/MAVLink console (`rl_tools_commander set_mode EXTERNAL`) or run `set_external_mode.py`
  only when the MAVLink endpoint is free.
- `with_realsense:=false`: not needed for Stage 2; required for full planner Stage 3.
- `with_optitrack:=false`: if enabled, `pose_topic` must match the live OptiTrack
  `geometry_msgs/PoseStamped` topic (`/drone/pose` or `/robot/pose`).
- Do **not** run `mobile_gazebo ego_planner_flight.launch.py` at the same time as
  `ego_raptor`; it starts the old offboard velocity controller and duplicates parts of the
  stack.

---

## Stage 0 — Bench preflight (PROPS OFF, once per session)

> **2026-06-12, bench (disarmed, no OptiTrack):** everything below passed except the
> position frame check (EKF `xy_valid=False` without a position source — redo when OptiTrack
> is up). Stream 100 Hz Jetson→FC, target tracking + freeze verified on the FC, set-EXTERNAL
> helper works over `/dev/ttyACM0`. **Converter orientation bug found and fixed** (yaw was
> −90° off in both odometry converters; yaw chain now verified exactly end-to-end).
> Details in `EXTERNAL_MODE_TODO.md` §2.

Must be running:

- [ ] OptiTrack PC streaming to the ROS mocap topic (`/drone/pose` or `/robot/pose`).
- [ ] MAVProxy for QGC only:
      `mavproxy.py --master=/dev/ttyACM0 --baudrate=57600 --out=udpout:192.168.0.233:14550`
- [ ] Raptor wrapper with DDS + OptiTrack bridge, no planner:
      `ros2 launch planner ego_raptor.launch.py start_planner:=false with_xrce:=true with_optitrack:=true pose_topic:=/drone/pose set_external:=false`
      (use `pose_topic:=/robot/pose` if that is the live topic).

Must **not** be running:

- [ ] `mobile_gazebo ego_planner_flight.launch.py` or `mobile_gazebo optitrack_full_pipeline.launch.py`
      during this Raptor test; those launch the old offboard-control path.
- [ ] RealSense, unless debugging Stage 3 planner perception.

Checks:

- [ ] FC shell: `uxrce_dds_client status` connected.
- [ ] FC shell/QGC console: `rl_tools_commander set_mode EXTERNAL`, then
      `rl_tools_commander status` → `mode: EXTERNAL`.
- [ ] `ros2 topic echo /fmu/out/vehicle_local_position --field xy_valid --once` → `True`.
- [ ] `ros2 topic echo /fmu/out/vehicle_local_position --field z_valid --once` → `True`.
- [ ] `ros2 topic echo /fmu/out/vehicle_local_position --field heading_good_for_control --once`
      → `True`.
- [ ] `ros2 topic hz /fmu/out/vehicle_odometry` — EKF odometry live.
- [ ] `ros2 topic echo /odometry` — sane ENU pose/velocity while moving the drone by hand.
- [ ] No goal / no fig8 running → `ros2 topic hz /fmu/in/trajectory_setpoint_raptor` shows **nothing**.
- [ ] `ros2 run planner fig8_pos_cmd.py` → topic hz ≈ **100 Hz**; FC `listener trajectory_setpoint_raptor` live.
- [ ] **Frame check (critical):** first setpoint NED position ≈ `vehicle_local_position`
      (drone still ⇒ fig8 anchors at current position, so they must match to ~cm; yaw consistent).
      If they don't line up, STOP — the drone would bolt.
- [ ] Ctrl-C the fig8 node → stream stops instantly → `rl_tools_commander status` shows the
      target **frozen** (no chase to zero).
- [ ] RC switch OFF → `dmesg` prints `Switching to original controller`.

## Stage 1 — Raptor regression flight (no companion in the loop)

Purpose: confirm the **flashed** firmware (new EXTERNAL code on board) did not regress the
already-proven behaviors. Nothing from the Jetson is involved.

- [ ] Arm, take off by hand, hover. RC switch → Raptor. Stable hover in its default mode.
- [ ] Run the same **figure-eight test as before** (firmware-internal trajectory, POSITION mode)
      — behavior identical to the pre-flash flights.
- [ ] RC switch off mid-trajectory → clean handback to PX4 manual.

**Gate:** any difference from pre-flash behavior → stop, investigate the firmware build.

## Stage 2 — EXTERNAL + fixed figure-8 (production pipeline, no planner)

Purpose: prove the EXTERNAL pipeline tracks a known shape — "for a given format, can
ego_raptor take this". Tethered for the first runs.

Setup (before the flight):
```bash
ros2 launch planner ego_raptor.launch.py \
  start_planner:=false \
  with_xrce:=true \
  with_optitrack:=true pose_topic:=/drone/pose \
  set_external:=false
ros2 bag record /odometry /drone_0_planning/pos_cmd /fmu/in/trajectory_setpoint_raptor /fmu/out/vehicle_local_position
```

Before takeoff:

- [ ] QGC sees the vehicle through MAVProxy.
- [ ] EXTERNAL mode is set manually and confirmed: `rl_tools_commander status`.
- [ ] `/odometry` and `/fmu/out/vehicle_odometry` are live.
- [ ] `xy_valid`, `z_valid`, and `heading_good_for_control` are all `True`.
- [ ] `/fmu/in/trajectory_setpoint_raptor` is silent until `fig8_pos_cmd.py` starts.
- [ ] RealSense and `ego_planner` are not running in this stage.

Flight:
- [ ] Arm, take off by hand, hover at test height. RC switch → Raptor (EXTERNAL already
      manually set ⇒ frozen at activation target — confirm it holds for ~10 s).
- [ ] First run, **small and slow**:
      `ros2 run planner fig8_pos_cmd.py --ros-args -p size_x:=0.75 -p size_y:=0.4 -p period:=40.0 -p laps:=1.0`
      (peak ≈ 0.2 m/s; the 8 anchors at the current hover point and ramps from zero velocity —
      no jump at start). Watch tracking; RC abort ready.
- [ ] After the lap it ramps down and **holds** — confirm stable hover hold, then Ctrl-C the node
      (freeze) and RC off to land or continue.
- [ ] Build up: default `fig8_pos_cmd.py` (2.0 × 1.0 m, 30 s/lap, peak ≈ 0.3 m/s, 2 laps),
      then shorter periods up to ~0.5 m/s peak — the planner's max_vel — to match Stage 3 dynamics.
- [ ] Mid-run aborts once each: (a) Ctrl-C the fig8 node → freeze-hover, (b) RC switch → PX4 manual.

**Pass criteria:** tracking error qualitatively small and bounded (check the bag: setpoint vs
`vehicle_local_position`, expect roughly < 0.3 m at these speeds), no oscillation/divergence,
both aborts clean.
**Gate:** if tracking is bad here, the issue is policy/transport/frames — fix before ever
launching the planner. Common suspects: odometry origin mismatch, yaw convention, setpoint rate.

## Stage 3 prep — safety zone (goal gate) setup  ← do once, in the OptiTrack room, props off

The FSM validates every incoming goal (**goal gate**: allowed box + tripod keep-out
cylinders) and acknowledges each goal on `/drone_0_planning/goal_status`
(`accepted` / `rejected:z|zone|keepout` / `failed:plan` / `reached`). Gate logic was
bench-verified 2026-06-12. The current launch has the gate **enabled** with the surveyed
room numbers in `real_single_drone.launch.py`; redo this section if the room, origin, or
tripod layout changes.

### What the zero point is

All gate numbers live in the **planner world frame**: the frame `/odometry` reports,
i.e. the ENU conversion of the PX4 EKF local frame. With OptiTrack fused into the EKF this
is *nominally* the OptiTrack calibration origin (where the calibration square sat on the
floor, ground = z 0) — but the authoritative answer is **whatever `/odometry` reads**, and
EKF/EV alignment settings can shift it. So never measure the room with a tape against the
OptiTrack origin and hand-convert axes. **Survey with the drone**: the same sensor chain
that will judge the goals tells you the coordinates of every landmark.

### Step 1 — survey the room with the drone (drone = measuring tape)

```bash
# terminal 1 — DDS bridge to the FC
ros2 launch planner ego_raptor.launch.py \
  start_planner:=false \
  with_xrce:=true \
  with_optitrack:=true pose_topic:=/drone/pose \
  set_external:=false

# terminal 2 — sanity first: EKF must have a valid position (OptiTrack fused)
ros2 topic echo /fmu/out/vehicle_local_position --field xy_valid --once   # must be True

# live readout while you carry the drone around:
ros2 topic echo /odometry --field pose.pose.position
```

Carry the drone (powered, props OFF) to each landmark, hold it still ~2 s, write down x/y
(z too at the floor — it should read ≈ 0 there; if not, note the offset):

| landmark | what to record |
|---|---|
| floor at OptiTrack origin | x, y, z — sanity: all ≈ 0 if frames align as expected |
| each corner of the intended flyable area | x, y |
| **each tripod** (hold the drone against the tripod column) | x, y |

Also sanity-check the axes: walk the drone ~2 m in one direction and confirm which
coordinate grows — that tells you which way +x and +y point in this room. Don't assume.

### Step 2 — choose the numbers

- **Box** = min/max of the corner readings, pulled **inward by ≥ 0.5 m** (wall/net margin;
  remember the gate filters *goals*, the drone still needs maneuvering room around them).
- **z band**: `z_min` 0.3 (never send the drone to the floor), `z_max` 2.2 (below the
  planner's `virtual_ceil_height` 2.5).
- **Keep-out radius per tripod** ≥ **1.0 m**: leg splay ~0.5 m + `obstacles_inflation`
  0.25 m + tracking error margin. The depth camera sees tripod legs poorly — the gate is
  the *primary* protection for the tripods, so be generous.
- Best layout: if the tripods ring the room (typical OptiTrack), set the box **inside** the
  tripod circle — then the keep-outs are belt-and-braces, and no legal goal-to-goal path
  even points at a tripod (the gate does not check the path, only the destination).

### Step 3 — set, rebuild, verify (still props off)

Edit the goal-gate block in
`src/planner/plan_manage/launch/real_single_drone.launch.py`, e.g. for a 6×6 m area with
two tripods:

```python
{'fsm/goal_gate_enable': True},
{'fsm/goal_gate_x_min': -2.5}, {'fsm/goal_gate_x_max': 2.5},   # surveyed corners − 0.5 m
{'fsm/goal_gate_y_min': -2.5}, {'fsm/goal_gate_y_max': 2.5},
{'fsm/goal_gate_z_min': 0.3},  {'fsm/goal_gate_z_max': 2.2},
{'fsm/keepout_x':      [ 2.9, -2.9]},      # surveyed tripod positions
{'fsm/keepout_y':      [ 2.9,  2.9]},      # parallel arrays: i-th entry = i-th tripod
{'fsm/keepout_radius': [ 1.0,  1.0]},
```

```bash
# launch files are COPIED on install — a rebuild is mandatory after editing:
cd /home/orin/ros2_ws && colcon build --packages-select ego_planner

# start the planner and confirm the gate is armed (look for this exact line):
ros2 launch ego_planner real_single_drone.launch.py
#   [drone_0_ego_planner_node]: Goal gate ON: box x[...] y[...] z[...], N keep-out(s)

# terminal 2 — watch the acks:
ros2 topic echo /drone_0_planning/goal_status

# terminal 3 — fire one goal per failure mode + one good goal:
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 99.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"   # → rejected:zone
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 2.9, y: 2.9, z: 1.0}, orientation: {w: 1.0}}}"    # → rejected:keepout  (use a real tripod xy)
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 0.0, y: 0.0, z: 2.4}, orientation: {w: 1.0}}}"    # → rejected:zone     (above z_max)
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 1.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"    # → accepted (in-zone)
```

All four acks correct → the zone is configured. This doubles as the still-open
"frame check with a valid EKF origin" from `EXTERNAL_MODE_TODO.md` §2: while surveying,
`/odometry` matching where the drone physically is *is* that check.

## Stage 3 — Full stack: EXTERNAL + ego-planner + VLM goals

Goal source: a **VLM node** (separate stack, pure oracle) publishing on
`/move_base_simple/goal` and listening to `/drone_0_planning/goal_status` (contract in
`INTEGRATION_plan.md` "Goal interface (VLM)"). All geometric safety is planner-side: the
goal gate from the prep section above **must be enabled** (`Goal gate ON` line at launch).
For first runs, RViz / `ros2 topic pub` stands in for the VLM — identical interface.

Setup: RealSense running, RViz on the ground station, and a terminal watching the acks:

```bash
ros2 launch planner ego_raptor.launch.py \
  with_xrce:=true \
  with_optitrack:=true pose_topic:=/drone/pose \
  with_realsense:=true \
  set_external:=false

ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo /drone_0_planning/goal_status
ros2 bag record /odometry /drone_0_planning/pos_cmd /fmu/in/trajectory_setpoint_raptor \
  /fmu/out/vehicle_local_position /drone_0_planning/bspline /move_base_simple/goal \
  /drone_0_planning/goal_status
```

Before any goal:

- [ ] MAVProxy is still the only process using `/dev/ttyACM0`.
- [ ] EXTERNAL mode manually confirmed in FC shell/QGC console.
- [ ] `/odometry`, `/fmu/out/vehicle_odometry`, and `/camera/camera/depth/image_rect_raw` live.
- [ ] `Goal gate ON` appears in the planner log.
- [ ] `/fmu/in/trajectory_setpoint_raptor` is silent before the first accepted goal.

Phase A — manual goals (VLM stand-in):

- [ ] Arm, take off by hand, RC switch → Raptor →
      `ros2 launch planner ego_raptor.launch.py ... with_realsense:=true ...`
      (planner included; check `Goal gate ON` and FC `mode: EXTERNAL`).
- [ ] No goal yet → drone keeps hovering (bridge silent). Confirm ~10 s.
- [ ] **Close goal first**: ~1–2 m ahead, obstacle-free, current altitude —
      `ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
        "{pose: {position: {x: 1.5, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"`
      → `accepted` on goal_status; planner max_vel = 0.5 m/s, same dynamics as Stage 2.
- [ ] At the goal: `reached` on goal_status, traj_server holds the final point — stable hover.
- [ ] **Deliberate rejection in flight**: publish a goal inside a tripod keep-out →
      `rejected:keepout`, drone keeps hovering, completely unaffected.
- [ ] Farther goals, then goals behind an obstacle (avoidance), then replanning by sending a new
      goal mid-flight (expect a fresh `accepted`).
- [ ] Mid-flight aborts once each: kill the launch (→ freeze-hover) and RC switch (→ manual).

Phase B — hand over to the VLM node:

- [ ] VLM node up with the stack on the bench first (planner running, **drone on the ground,
      disarmed, Raptor off**): let it publish a few real decisions, confirm every goal gets
      `accepted`/`rejected:*` and that rejected ones make the VLM re-decide, not re-spam.
- [ ] In flight, repeat the Phase A protocol but with the VLM publishing: first decision in
      open space, then with obstacles. Operator keeps the goal_status echo visible — every
      `accepted` should correspond to a goal the operator would have allowed.
- [ ] One full mission: several consecutive VLM goals chained by `reached`, RC abort at the end.

**Pass criteria:** same tracking quality as Stage 2; obstacle avoidance with margin
(`obstacles_inflation` = 0.25 m); clean goal-hold and aborts.

---

## Quick reference

| Item | Command |
|------|---------|
| Pipeline w/o planner | `ros2 launch planner ego_raptor.launch.py start_planner:=false with_xrce:=true with_optitrack:=true pose_topic:=/drone/pose set_external:=false` |
| Full stack | `ros2 launch planner ego_raptor.launch.py with_xrce:=true with_optitrack:=true pose_topic:=/drone/pose with_realsense:=true set_external:=false` |
| Fixed figure-8 | `ros2 run planner fig8_pos_cmd.py` (`size_x, size_y, period, laps, ramp, max_speed`) |
| Set EXTERNAL manually | FC shell/QGC console: `rl_tools_commander set_mode EXTERNAL`; only use `ros2 run planner set_external_mode.py /dev/ttyACM0` if MAVProxy is not using `/dev/ttyACM0` |
| FC verify | `rl_tools_commander status` · `listener trajectory_setpoint_raptor` · `dmesg` |
| Jetson verify | `ros2 topic hz /fmu/in/trajectory_setpoint_raptor` (expect ~100 Hz) |
| EKF validity | `ros2 topic echo /fmu/out/vehicle_local_position --field xy_valid --once` · `z_valid` · `heading_good_for_control` |
| Depth input | `ros2 topic hz /camera/camera/depth/image_rect_raw` |
| Goal acks | `ros2 topic echo /drone_0_planning/goal_status` |
| Send a goal | `ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped "{pose: {position: {x: 1.5, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"` |
| Survey readout | `ros2 topic echo /odometry --field pose.pose.position` |
| Wrapper launch args | `ros2 launch planner ego_raptor.launch.py --show-args` |
| Gate config | goal-gate block in `real_single_drone.launch.py` → `colcon build --packages-select ego_planner` (launch files are copied, not symlinked) |

Safety nets (all independent of the Jetson): RC switch master override → multiplexer
`SWITCH_BACK`; EXTERNAL 300 ms staleness freeze; multiplexer 30/100 ms timeouts;
`fig8_pos_cmd` refuses to run above `max_speed` (default 1.0 m/s) and anchors at the current
position with a zero-velocity ramp.

Jetson-side net (Stage 3): the FSM **goal gate** rejects any goal outside the allowed box or
inside a tripod keep-out before planning starts — independent of who publishes the goal
(VLM, RViz, topic pub). It filters destinations only; en-route avoidance stays with the
grid map.
