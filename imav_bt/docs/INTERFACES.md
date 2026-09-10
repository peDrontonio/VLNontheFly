# Interfaces

Everything `imav_bt` exchanges with the rest of the workspace. The authoritative
copy of the names is `imav_bt/interfaces.py`; this document is the prose version
and explains the parts that are not obvious.

---

## Published

| Topic | Type | Notes |
|---|---|---|
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Absolute goal in the ENU `map` frame. Consumed by ego-planner's MANUAL_TARGET waypoint callback. **Only `pose.position` is used** — the planner ignores orientation (`ego_replan_fsm.cpp:447`). |
| `/relative_goal` | `geometry_msgs/PoseStamped` | Body-relative FLU offset (+x forward, +y left, +z up), converted to a map goal by `planner/src/relative_goal_to_map.py`. Available for behaviours that reason in the drone frame; nothing uses it yet. |
| `~/status` | `std_msgs/String` | One line per tick: tick count, root status, safety reason, mission outcomes. |
| `~/snapshot` | `std_msgs/String` | Rendered tree, every `snapshot_every_n_ticks` ticks. |

QoS on the goal publisher is **RELIABLE, VOLATILE, depth 1**. Volatile is
deliberate: `TRANSIENT_LOCAL` would replay a stale goal to a planner that
restarts mid-flight, which is the opposite of what you want.

## Subscribed

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `/odometry` | `nav_msgs/Odometry` | **BEST_EFFORT** | ENU `map` frame, from `planner/src/odometry_converter.py`. The only pose source the tree reads. |
| `/planning/goal_status` | `std_msgs/String` | Reliable | ego-planner goal feedback. See below. |
| `/battery_status` | `sensor_msgs/BatteryState` | BEST_EFFORT | Optional. Absence does **not** ground the drone. |

> **QoS matters here.** `/odometry` is published BEST_EFFORT. A RELIABLE
> subscription is incompatible and will silently never connect — you get no
> error, just no messages, and the localization guard trips forever.

## Services offered

| Service | Type | Effect |
|---|---|---|
| `~/set_running` | `std_srvs/SetBool` | `true` arms the selected flight plan. `false` returns to idle **and clears the abort latch**. |
| `~/abort` | `std_srvs/Trigger` | Latches an operator abort; the safety branch takes over on the next tick. |

## Services called

All `std_srvs/Trigger`, all currently mocked by `scripts/payload_node.py`:

| Service | Mission | Worth |
|---|---|---|
| `/payload/drop_cone` | 3 | `Dr` = 4 pts in the hot box, 2 in another |
| `/payload/blink_hotspot_led` | 3 | `Hb` = 3 pts, **awarded on the blink alone** |
| `/payload/grab_ring` | 4 | `grabRing` = 4 pts |
| `/payload/release_ring` | 4 | — |
| `/payload/read_continuity` | 4 | `touchWind` = 5 pts |

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `config_dir` | installed `share/imav_bt/config` | Directory holding `bt.yaml`, `arena.yaml`, `missions.yaml`. |
| `flight_plan` | `flight_a` | `idle` \| `flight_a` \| `flight_b`. **Rejected while running** — stop first. |
| `autostart` | `false` | Arm at startup. Bench only; `imav_indoor.launch.py` keeps it false. |

---

## The `goal_status` contract

ego-planner publishes `std_msgs/String` on `planning/goal_status`
(`ego_replan_fsm.cpp:154`, published at `:313`). This is a **first-party addition
to ego-planner, not upstream**, and it is only published when the planner is in
MANUAL_TARGET mode — which is the mode the tree drives it in.

| String | Emitted at | Meaning | `GotoAnchor` result |
|---|---|---|---|
| `accepted:safety_bubble` | `:483` | Goal passed the goal-gate, odometry and FSM-state checks | RUNNING |
| `accepted` | `:253` | Global trajectory successfully planned | RUNNING |
| `rejected:<reason>` | `:461` | Goal-gate or `z`-bound violation | FAILURE |
| `rejected:not_ready` | `:272`, `:477` | No odometry, or FSM not in `WAIT_TARGET`/`EXEC_TRAJ` | FAILURE |
| `failed:plan` | `:282` | `planGlobalTraj` failed | FAILURE |
| `reached` | `:833` | Local target == global target and the trajectory duration elapsed | SUCCESS |

An unrecognised string is treated as RUNNING and logged — if ego-planner grows a
new status, the tree keeps waiting rather than guessing.

### ⚠ The gap: there is no failure status for getting stuck

**ego-planner publishes no replan-failure and no collision status.** Replan
failures only change internal FSM state; nothing goes on the wire.

So if the planner accepts a goal and then gets stuck — trapped by newly-observed
occupancy, replanning forever, or simply not moving — it publishes **nothing
further**. `reached` never arrives. No failure ever appears. The topic just goes
quiet.

A leaf that waits on `goal_status` will therefore return RUNNING forever, and the
flight ends with the drone hovering until the battery runs out.

This cannot be fixed inside the leaf, because the information genuinely does not
exist. Two mitigations, both in `behaviors/flight.py`:

1. **`Deadline` on every goal-following leaf.** Wall-clock time is the only
   observable that distinguishes "working on it" from "stuck". Use the `goto()`
   helper, which applies the decorator for you — **a bare `GotoAnchor` is always
   a mistake.**
2. **`arrival_tolerance_m`**, optional. Succeed when the drone is physically
   within tolerance of the target, independent of what the planner says. Off by
   default, because the planner is normally the authority.

Reproduce it on the bench with `planner_mode:=silent`; see
[TESTING.md](TESTING.md).

### ⚠ The other trap: stale status

The status topic still holds the **previous** goal's result at the moment you
publish the next one. A leaf that reads it naively sees a leftover `reached` and
reports instant success without having moved.

`GotoAnchor` records the publish time and ignores any status whose receipt
timestamp predates it. `test_stale_status_is_ignored` pins this.

Note that staleness is measured on **receipt time from the monotonic clock**, not
on message header stamps. A publisher that wedges while still stamping fresh
headers is a real failure mode, and comparing headers to ROS time would miss it.
The same reasoning applies to `odom_stamp`.

---

## Who must not run at the same time

Three things in this workspace can publish to `/move_base_simple/goal`. If more
than one runs, they fight over the drone:

- **this tree**
- **`vlm_region_gate`** with `auto_execute: true` — both shipped configs
  (`edgellm_vlm_ros/config/region_gate.yaml:25`,
  `region_gate_pipeline.yaml:28`) point its `goal_topic` straight at
  `/move_base_simple/goal`, bypassing `relative_goal_to_map`.
- **`vlm_nav_supervisor`**, which drives the gates' `~/execute_next` services and
  flips the VLM prompt mode.

In this phase the tree is the sole goal publisher, and `imav_indoor.launch.py`
simply does not start the VLM stack.

If the VLM is reintroduced later, run the gates with `auto_execute: false` and
have a behaviour call their `~/execute_next` (`std_srvs/Trigger`) services, so
arbitration stays in one place — the tree.

> Note for whoever does that: `vlm_node`'s `target_object` is read **once at
> construction** (`vlm_node.cpp:393`, into a `const`) and baked into the prompt,
> and `~/set_point_mode` only toggles `point`↔`primitive` — it cannot select
> `region` mode at all (`:596-599`). Retargeting the VLM per mission therefore
> needs a parameter callback added to `vlm_node.cpp` first.

---

## Blackboard keys

Namespace `/imav`. Declared in `imav_bt/blackboard.py:SCHEMA`, which is the
authority. Keys divide by **who is allowed to write them**:

| Class | Written by | Keys |
|---|---|---|
| `SENSED` | `bt_engine` subscriptions **only** | `odom_valid`, `position`, `yaw`, `odom_stamp`, `goal_status`, `goal_status_stamp`, `battery_valid`, `battery_fraction` |
| `COMMANDED` | operator services / parameters | `flight_plan`, `running`, `abort_requested` |
| `DERIVED` | behaviours | `mission_outcomes`, `objectives`, `active_goal`, `safety_reason` |

A behaviour writing a `SENSED` key is a bug — it would be fabricating sensor
data. `bb.client()` enforces READ vs WRITE registration, so this is caught at
construction rather than in flight.
