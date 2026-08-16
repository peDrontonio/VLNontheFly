# Figure-8 on Raptor EXTERNAL — launch runbook

Fixed figure-eight flown through the **production Raptor chain with ego-planner swapped
out**. This is Stage 2 of `HARDWARE_TESTS.md`: it proves the EXTERNAL pipeline tracks a
known shape before any planner is trusted with the vehicle.

Chain exercised:

```
fig8_pos_cmd.py ──/drone_0_planning/pos_cmd (ENU, 100 Hz)──> pos_cmd_to_raptor.py
   ──/fmu/in/trajectory_setpoint_raptor (NED)──> uXRCE-DDS ──> rl_tools_commander (EXTERNAL)
```

`fig8_pos_cmd.py` impersonates ego-planner's `traj_server` exactly — same message type,
same topic, same 100 Hz. Nothing else in the chain changes.

---

## Safety contract — read before flying

- **Raptor is engaged only by the operator's RC switch.** Nothing in this runbook arms the
  vehicle, takes off, or engages Raptor from ROS. `pos_cmd_to_raptor.py` has, by design, no
  authority over activation. Arm and take off **by hand / RC**, then flip the switch.
- **Keep the RC abort switch in hand for the entire run.** Releasing it prints
  `Switching to original controller` in `dmesg` and hands back to PX4 manual.
- **Tether the first runs.**
- Ctrl-C on the fig8 node stops the stream instantly; the FC's 300 ms staleness freeze then
  holds the last target. That is a *freeze*, not a land — it is an abort step, not an exit.
- `start_planner:=false` here is deliberate. RealSense and `ego_planner` must **not** run in
  this stage.

---

## 1. Preconditions

- OptiTrack PC streaming the tracked body pose (the NatNet driver publishing `/drone/pose`).
- MAVProxy for QGC only:
  ```bash
  mavproxy.py --master=/dev/ttyACM0 --baudrate=57600 --out=udpout:192.168.0.233:14550
  ```
- No stale processes from an earlier session. Exactly one XRCE agent must exist once the
  launch is up — multiple agents fight over `/dev/ttyTHS1` and the FC silently never
  connects:
  ```bash
  pgrep -af MicroXRCEAgent          # expect nothing before launch
  pgrep -af optitrack_bridge_node   # expect nothing before launch
  ros2 node list | grep '^/optitrack_bridge_node$'
  ```
  Kill anything left over before continuing.
- Workspace built and sourced:
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/orin/VLNontheFly/install/setup.bash
  ```

---

## 2. Launch the pipeline (terminal 1)

```bash
ros2 launch planner ego_raptor.launch.py \
  start_planner:=false \
  with_xrce:=true \
  with_optitrack:=true \
  set_external:=false
```

`set_external:=false` because MAVProxy owns `/dev/ttyACM0` — EXTERNAL gets set manually in
step 4. Do not pass `set_external:=true` while MAVProxy is running; they will contend for
the same port.

## 3. Record the bag (terminal 2)

Start this **before** the flight — the pass criteria are judged from it.

```bash
ros2 bag record \
  /odometry \
  /drone_0_planning/pos_cmd \
  /fmu/in/trajectory_setpoint_raptor \
  /fmu/out/vehicle_local_position
```

---

## 4. Pre-takeoff checks

All of these must pass. Any failure = stop.

```bash
# EKF has a valid position/heading solution
ros2 topic echo /fmu/out/vehicle_local_position --field xy_valid --once
ros2 topic echo /fmu/out/vehicle_local_position --field z_valid --once
ros2 topic echo /fmu/out/vehicle_local_position --field heading_good_for_control --once

# odometry live end to end
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic hz /odometry

# the Raptor setpoint topic must be SILENT until fig8 starts
ros2 topic hz /fmu/in/trajectory_setpoint_raptor
```

- [ ] `xy_valid`, `z_valid`, `heading_good_for_control` → all `True`.
- [ ] `/odometry` and `/fmu/out/vehicle_odometry` both publishing.
- [ ] `/fmu/in/trajectory_setpoint_raptor` silent.
- [ ] QGC sees the vehicle through MAVProxy.
- [ ] EXTERNAL set manually from the FC shell / QGC console and confirmed:
      ```
      rl_tools_commander set_mode EXTERNAL
      rl_tools_commander status        # → mode: EXTERNAL
      ```
- [ ] RealSense and `ego_planner` are **not** running.

---

## 5. Fly it

1. **Arm and take off by hand / RC.** Hover at test height.
2. **RC switch → Raptor.** EXTERNAL was already set, so it engages frozen at the activation
   target — confirm it holds steady for ~10 s before streaming anything.
3. **Start the figure-8 (terminal 3). First run small and slow:**

   ```bash
   ros2 run planner fig8_pos_cmd.py --ros-args \
     -p size_x:=0.75 \
     -p size_y:=0.4 \
     -p period:=40.0 \
     -p laps:=1.0
   ```

   Peak ≈ 0.2 m/s. The 8 anchors at the current `/odometry` position and yaw and ramps from
   zero velocity, so there is no jump at start. The node publishes nothing until it has
   anchored on odometry.

4. **Frame check — critical, do this on the first run.** With the drone still hovering, the
   first commanded NED setpoint must match `vehicle_local_position` to ~cm, with consistent
   yaw:

   ```bash
   ros2 topic echo /fmu/in/trajectory_setpoint_raptor --once
   ros2 topic echo /fmu/out/vehicle_local_position --once
   ```

   If they do not line up, **STOP and abort** — the drone would bolt. Suspects: odometry
   origin mismatch, yaw convention, setpoint rate.

5. **Watch tracking through the lap.** After `laps` laps the node ramps down and holds the
   final point with zero velocity (hover hold), logging `laps done — ramping down to hover
   hold`. Confirm the hold is stable, then Ctrl-C (freeze) and RC off to land, or continue.

6. **Build up gradually**, re-checking tracking at each step:
   - default: `ros2 run planner fig8_pos_cmd.py` → 2.0 × 1.0 m, 30 s/lap, peak ≈ 0.3 m/s, 2 laps
   - then shorter `period` up to ~0.5 m/s peak — the planner's `max_vel`, matching Stage 3 dynamics

7. **Exercise both aborts once each:**
   - Ctrl-C the fig8 node → stream stops → target freezes (no chase to zero)
   - RC switch off mid-trajectory → clean handback to PX4 manual

---

## Parameters

| Param | Default | Meaning |
|---|---|---|
| `size_x` | 1.0 | m, East half-width of the 8 |
| `size_y` | 0.5 | m, North half-height of the 8 |
| `period` | 30.0 | s per lap |
| `laps` | 2.0 | laps before ramp-down (0 = endless) |
| `ramp` | 3.0 | s, speed ramp up/down |
| `rate_hz` | 100.0 | publish rate |
| `max_speed` | 1.0 | m/s — node **refuses to start** above this |

Shape (ENU, anchored at the hover point `x0,y0,z0`):

```
x(tau) = x0 + size_x * sin(tau)      z = z0 (constant altitude)
y(tau) = y0 + size_y * sin(2*tau)    yaw held at the anchor yaw
```

`max_speed` is a hard refusal, not a clamp: if the peak speed implied by `size_*`/`period`
exceeds it the node exits with the minimum `period` you would need. Increase `period` or
shrink the 8 — do not raise `max_speed` to get past it.

---

## Pass criteria

- Tracking error qualitatively small and bounded — from the bag, setpoint vs
  `vehicle_local_position`, expect roughly **< 0.3 m** at these speeds.
- No oscillation or divergence.
- Both aborts clean.

**Gate:** if tracking is bad here, the problem is policy / transport / frames. Fix it before
ever launching the planner — do not proceed to Stage 3.

---

## Troubleshooting

**`fig8_pos_cmd.py` prints "waiting for odometry to anchor..." and never starts**
`/odometry` is not publishing. It comes from `odometry_converter.py`, which only republishes
PX4's own EKF output — so it stays silent whenever `/fmu/out/vehicle_odometry` is silent.
Check the XRCE session first: `ros2 topic hz /fmu/out/vehicle_odometry`. If that is dead,
the FC's `uxrce_dds_client` is not connected — verify exactly one `MicroXRCEAgent` owns
`/dev/ttyTHS1` (`fuser -v /dev/ttyTHS1`), then restart the client from the FC shell.

**Nodes die instantly at launch (exit code 127 or 1)**
Missing build artifacts or a stale CMake cache. Rebuild:
`colcon build --symlink-install --packages-skip mockamap local_sensing realsense2_ros_mqtt_bridge`.
Exit 127 = a shared library was not found; exit 1 on a Python node is usually a missing
message package. Never Ctrl-C a `colcon build` — an interrupted build leaves packages that
*look* installed (marker files only, no libraries) and every dependent node then fails at
launch.

**`ros2 launch` reports "executable '<x>.py' not found on the libexec directory"**
That script lost its executable bit. `chmod +x planner_wrapper/src/<x>.py`, then rebuild.

**`/fmu/in/trajectory_setpoint_raptor` stays silent once fig8 is running**
`pos_cmd_to_raptor.py` is purely event-driven — it publishes only on receiving a `pos_cmd`.
Confirm the source: `ros2 topic hz /drone_0_planning/pos_cmd` (expect ~100 Hz).

**Nodes appear duplicated in `ros2 node list`**
Orphans from an interrupted launch survive their parent. `pgrep -af` the node names and kill
them before relaunching; stale DDS discovery entries clear on their own after a few seconds.

---

## Quick reference

| What | Command |
|---|---|
| Pipeline | `ros2 launch planner ego_raptor.launch.py start_planner:=false with_xrce:=true with_optitrack:=true set_external:=false` |
| Figure-8 (first run) | `ros2 run planner fig8_pos_cmd.py --ros-args -p size_x:=0.75 -p size_y:=0.4 -p period:=40.0 -p laps:=1.0` |
| Figure-8 (default) | `ros2 run planner fig8_pos_cmd.py` |
| Setpoint rate | `ros2 topic hz /fmu/in/trajectory_setpoint_raptor` |
| EXTERNAL status | `rl_tools_commander status` (FC shell) |
| Abort — freeze | Ctrl-C the fig8 node |
| Abort — manual | RC switch off |

Related: `HARDWARE_TESTS.md` (Stage 2 checklist), `PRODUCTION_PIPELINE_RUNBOOK.md`
(full pipeline with the planner).
