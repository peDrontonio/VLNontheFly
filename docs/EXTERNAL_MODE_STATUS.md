# Raptor EXTERNAL Mode — Flash & Bring-up To-Do

Fly ego-planner paths with the embedded Raptor RL policy via a new `Mode::EXTERNAL` in
`rl_tools_commander`, while the **RC switch stays the master override**.

**Status:** firmware built + SITL-validated. Remaining work is all on real hardware.

---

## Done (verified)

- [x] Firmware: `Mode::EXTERNAL = 3` added to `rl_tools_commander` (300 ms staleness freeze,
      `set_mode EXTERNAL` shell cmd, `trajectory_setpoint_raptor` subscription).
- [x] Firmware: `TrajectorySetpoint.msg` topic alias + `dds_topics.yaml` inbound subscriber.
- [x] **FMU-V6C binary built** — `.../build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`
      (built after source edits; ELF confirmed to contain `trajectory_setpoint_raptor` +
      `set_mode {...,EXTERNAL}`; flash 96.27%, ~70 KB free).
- [x] SITL dry-run `sanity/test_external_mode.py` — **PASS**: 0.151 m freeze drift in 10 s
      with no setpoints (limit 0.5 m); `set_mode EXTERNAL` and return to `POSITION` both work.
- [x] Companion `companion/raptor_path_tracker.py` — carrot @ 0.8 m/s, ENU→NED, re-anchors on
      replan, publishes nothing before first path (safe). Reviewed, ready.
- [x] **Integration bridge** — `planner_wrapper/src/pos_cmd_to_raptor.py` forwards
      `/drone_0_planning/pos_cmd` (ego-planner traj_server, ENU, 100 Hz) 1:1 as NED
      `TrajectorySetpoint` → 100 Hz stream. `ego_raptor.launch.py` starts odometry
      converter + ego-planner + bridge + one-shot `set_external_mode.py` (MAVLink shell).
      The carrot tracker is now **bench-only** (canned paths); flight path uses pos_cmd.
      See `INTEGRATION_plan.md`.

---

## To-Do (real hardware)

### 1. Flash the FMU-V6C  ← you can do this now
- [ ] QGroundControl → **Vehicle Setup → Firmware** → flash custom file:
      `rl-tools/embedded_platforms/px4/px4_autopilot/build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`
- [ ] After reboot, FC shell: `rl_tools_commander status` runs and
      `rl_tools_commander set_mode EXTERNAL` is accepted (no "unknown mode").
- [ ] `listener trajectory_setpoint_raptor` exists (topic registered, even if 0 msgs yet).
- [ ] Sanity: existing hover / figure-eight still behave (no regression in POSITION mode).

### 2. Bench — DDS path (PROPS OFF) ← DONE 2026-06-12 (drone disarmed, no OptiTrack yet)
- [x] Jetson: agent over the FMU link — `MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600`.
      FC client connected immediately; all `/fmu/*` topics up **including
      `/fmu/in/trajectory_setpoint_raptor`** (flashed firmware confirmed on the wire).
- [x] `/odometry` from `odometry_converter.py` (on `/fmu/out/vehicle_odometry`) sane.
      **BUG FOUND & FIXED:** both odometry converters used `T·R·Tᵀ` for orientation →
      yaw was −90° off (and roll/pitch swapped under tilt). Fixed to `T·R·D`
      (D = diag(1,−1,−1), body FRD→FLU); verified live: `yaw_enu = π/2 − heading` exactly.
- [x] `set_external_mode.py /dev/ttyACM0` → `mode confirmed: EXTERNAL`, exit 0
      (MAVLink shell over USB works; FC heartbeat shows DISARMED).
- [x] fig8_pos_cmd → pos_cmd_to_raptor stream: `ros2 topic hz` = **100 Hz**; FC
      `listener trajectory_setpoint_raptor` shows live setpoints (<15 ms old).
- [x] **Yaw chain verified end-to-end with real attitude:** FC heading 0.086 rad →
      `/odometry` ENU 1.485 → fig8 holds it → bridge → setpoint NED yaw 0.086. Exact round trip.
- [x] `rl_tools_commander status`: `mode: EXTERNAL`, `target_position` **tracking** the stream.
- [x] Stop the fig8 node → bridge silent → target **frozen** (two status reads identical).
- [ ] **Frame check with a valid EKF origin — still open:** EKF had `xy_valid=False`
      (no OptiTrack/VIO yet), so positions were all ≈ 0. Redo the position part of the frame
      check once OptiTrack feeds the EKF. Yaw side is already verified.
- [ ] With ego-planner in the loop (RViz goal instead of fig8): repeat hz + listener checks.

### 3. Bench — RC override (PROPS OFF)
- [ ] Activate Raptor via RC switch, then flip it OFF → `dmesg` prints
      `Switching to original controller` and PX4 manual is restored.
- [ ] Confirm killing/spamming the companion node CANNOT keep Raptor engaged.

### 4. Tethered flight

> Staged flight procedure now lives in **`HARDWARE_TESTS.md`**: Stage 1 firmware figure-8
> regression → Stage 2 EXTERNAL + fixed figure-8 via the production pipeline
> (`fig8_pos_cmd.py`, planner swapped out) → Stage 3 full ego-planner stack.

- [ ] Hover under Raptor (POSITION mode), confirm stable.
- [ ] With the stack launched (EXTERNAL set), publish a **close** RViz goal (~1–2 m) —
      planner max_vel is 0.5 m/s — RC switch ready as abort.
- [ ] Build up to farther goals / full paths. Abort = release RC switch at any time.

---

## Quick reference

| Item | Path / command |
|------|----------------|
| FMU-V6C binary | `rl-tools/.../build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4` |
| Flight stack | `ros2 launch planner ego_raptor.launch.py` (see `INTEGRATION_plan.md`) |
| pos_cmd bridge | `planner_wrapper/src/pos_cmd_to_raptor.py` |
| Set EXTERNAL (auto) | `planner_wrapper/src/set_external_mode.py /dev/ttyACM0` |
| Bench-only canned-path node | `python3 companion/raptor_path_tracker.py` |
| Companion docs | `companion/README.md` |
| SITL test | `python3 sanity/test_external_mode.py` |
| Activate mode | `rl_tools_commander set_mode EXTERNAL` |
| FC verify | `rl_tools_commander status` · `listener trajectory_setpoint_raptor` · `dmesg` |
| Jetson verify | `ros2 topic hz /fmu/in/trajectory_setpoint_raptor` |
| Agent | `MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600` |

## Safety nets (layered, all independent of the companion)
1. **RC switch (AUX1)** → `SWITCH_BACK` — master override, companion has zero authority over `.active`.
2. **EXTERNAL freeze** — holds last target, zero velocity, on >300 ms setpoint staleness or before
   the first setpoint.
3. **Multiplexer timeouts** — activation 30 ms / RL-output 100 ms fall back to PX4.
4. **Carrot speed cap** — 0.8 m/s; keep first flights tethered with a small path.
