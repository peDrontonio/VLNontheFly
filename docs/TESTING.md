# Testing: how to exercise every component

Testing here is a ladder. Each rung adds exactly one new variable, and each has
a pass criterion you can check before climbing. Do not skip rungs — the whole
point of the staging is that when something misbehaves you know which variable
introduced it.

| Rung | What it adds | Hardware | Props |
|---|---|---|---|
| [1. Unit tests](#1-unit-tests) | nothing — pure logic | none | — |
| [2. Bench, no FC](#2-bench-without-a-flight-controller) | real sensors, real ROS graph | camera / bag | off |
| [3. Bench with the FC](#3-bench-with-the-flight-controller) | DDS link, PX4, EXTERNAL mode | full airframe | **off** |
| [4. Goal-gate setup](#4-goal-gate-safety-zone-setup) | the safety volume | full airframe | **off** |
| [5. Staged flight](#5-staged-flight) | flight | full airframe | on |

---

## 1. Unit tests

No hardware, no weights, no GPU. Runs in about a second.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
pytest tests/
```

**Pass criterion:** all tests pass. Currently `45 passed`.

Covers the VLM goal gates and the nav supervisor — the JSON parsing, the
region-to-3D projection, the depth read-out, and the accept/reject decisions of
the safety gates. These are the components that decide whether a grounded goal
is allowed to become a trajectory, so they are the ones worth testing off-robot.

ROS 2 must be sourced because the gates exchange `geometry_msgs`,
`sensor_msgs` and `px4_msgs` types. `px4_msgs` is fetched, so
`vcs import < third_party.repos` and one `colcon build` must have happened.

See [`tests/README.md`](../tests/README.md) for the layout and for how to add a
package's tests.

## 2. Bench without a flight controller

Everything perception-side can be exercised with no drone at all.

### VLM grounding, live camera

```bash
ros2 launch edgellm_vlm_ros d435i_vlm.launch.py \
  prompt_mode:=region \
  enable_region_gate:=true
```

**Pass criteria:**
- the node reports a median query time near **0.79 s** on an Orin NX
  (0.60 s time-to-first-token); much slower means the INT4 engine did not load
  and it fell back
- every response parses as one minified JSON object — the gate rejects anything
  else, and a rejection storm means a prompt or engine problem
- pointing the camera at a referent it should not find yields `NONE` with low
  confidence, not a confident wrong cell

### VLM grounding, recorded bag

Preferred for regression work, because it is repeatable. Record:

```
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
/tf  /tf_static
/odometry
```

Replay with `ros2 bag play`, run the launch above, and compare the accepted
goals against the previous run. Grounding is stochastic; the *gate decisions*
should not swing.

### Depth

`depth_estimator` is the monocular alternative and is **experimental** — it is
not part of the validated flow. Its model downloads itself from HuggingFace on
first run.

## 3. Bench with the flight controller

**Props off. Disarmed.** This is Stage 0 and Stage 2 of
[`HARDWARE_BRINGUP.md`](HARDWARE_BRINGUP.md), which is the authoritative
procedure — the summary here is only so you know what the rung covers.

```bash
# DDS bridge to the FC
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
```

**Pass criteria:**
- all `/fmu/*` topics appear, **including `/fmu/in/trajectory_setpoint_raptor`**
  (its presence is what proves the right firmware is flashed)
- `/odometry` is sane, and the yaw round-trip holds: FC heading → ENU odometry
  → setpoint NED yaw comes back to the same number
- `ros2 topic hz /fmu/in/trajectory_setpoint_raptor` reads **100 Hz**
- `rl_tools_commander status` shows `mode: EXTERNAL` and a tracking target
- **stop the publisher and the FC target must freeze**, not drift

The last one is the safety property that matters most: the EXTERNAL freeze on
>300 ms setpoint staleness.

### RC override

Also props off, and non-negotiable before flight: flip the RC switch off and
confirm `dmesg` prints `Switching to original controller` and PX4 manual
control returns. Then confirm that killing or spamming the companion node
**cannot** keep Raptor engaged. The RC switch is the master override; the
companion has no authority over it.

## 4. Goal-gate safety zone setup

Done once per room, props off. Full procedure in
[`HARDWARE_BRINGUP.md`](HARDWARE_BRINGUP.md) ("Stage 3 prep"): survey the room
by carrying the drone, choose the box and keep-out numbers, rebuild, and fire
one goal per failure mode plus one good goal.

**Pass criterion:** the planner logs the gate as armed —

```
[drone_0_ego_planner_node]: Goal gate ON: box x[...] y[...] z[...], N keep-out(s)
```

— and every out-of-volume goal is rejected while the good goal is accepted.
Launch files are copied on install, so **a rebuild is mandatory after editing
them**.

## 5. Staged flight

Props on. Follow [`HARDWARE_BRINGUP.md`](HARDWARE_BRINGUP.md) in order:

1. **Stage 1** — Raptor regression, firmware only, no companion, no EXTERNAL.
2. **Stage 2** — EXTERNAL with a fixed figure-8. Same interface, bridge, DDS
   link, QoS and 100 Hz rate as a real flight, with `fig8_pos_cmd.py`
   substituted for the planner. Step-by-step in
   [`RUNBOOK_FIG8.md`](RUNBOOK_FIG8.md), which includes its own pass criteria
   and troubleshooting.
3. **Stage 3** — the full stack with VLM goals. Start with a close goal
   (1–2 m; planner `max_vel` is 0.5 m/s), RC switch in hand.

Stage 2 exists so that a Stage 3 failure is attributable: if the figure-8
tracks well and the full stack does not, the problem is in the planner —
mapping, odometry or goals — not in the transport or the policy.

**Abort at any stage: release the RC switch.**

---

## Testing something not listed here

If you add a component, add its rung. Concretely: unit tests go in
`tests/<package_name>/`, and if the component can fail in a way that endangers
the airframe, it needs a props-off bench check with a written pass criterion
before it is allowed near a flight.
