# imav_bt — IMAV 2026 indoor mission behaviour tree

The decision-making layer for VLNontheFly's indoor competition entry: which
mission runs, in what order, when to give up on a sub-objective, and when to
abort the flight.

It sits **above** the existing flight stack and does not fly the drone. It
publishes goals to ego-planner and reads back status; ego-planner, `traj_server`
and `pos_cmd_to_raptor` do the flying exactly as they do today.

```
imav_bt  ──PoseStamped──▶  /move_base_simple/goal  ──▶  ego_planner
   ▲                                                        │
   └──── std_msgs/String ◀── planning/goal_status ◀──────────┘
                                                            │
                            traj_server ──▶ pos_cmd_to_raptor ──▶ PX4 (EXTERNAL)
```

## Status: framework complete, mission logic not written

This package currently contains a **finished, tested tree skeleton** and
**placeholder mission logic**. That split is deliberate. The tree's control flow
— ordering, retry policy, what happens when a mission fails — is the part that
is expensive to change later, so it is built and tested first. The leaf bodies,
which are cheap to swap, are stubs.

| Component | State |
|---|---|
| Blackboard, decorators, guards, tree assembly | Complete, 71 tests |
| `GotoAnchor` (the ego-planner goal handshake) | Complete |
| Safety branch, flight-plan selection, idle | Complete |
| Mission 1–4 subtrees | **Placeholders** — structure and scoring rationale documented, every leaf a `Stub` |
| Precision landing | **Placeholder** |
| Perception (ArUco, gate detection, counting, thermal) | Not started — none exists in the repo |
| Payload drivers | Mocked behind `scripts/payload_node.py` |

To fill in a mission, see **[docs/ADDING_A_MISSION.md](../docs/imav_bt/ADDING_A_MISSION.md)**.

## Installation

`py_trees` 2.5 is the only new dependency:

```bash
sudo apt install ros-humble-py-trees      # preferred
pip install --user "py_trees>=2.2,<2.6"   # if you cannot use apt
```

`py_trees_ros` is deliberately **not** required — see
[docs/ARCHITECTURE.md](../docs/imav_bt/ARCHITECTURE.md#why-not-py_trees_ros).

```bash
cd /home/orin/VLNontheFly
colcon build --packages-select imav_bt
source install/setup.bash
```

## Run it on a desk, in three commands

No aircraft, no hardware, no ROS bag:

```bash
colcon build --packages-select imav_bt && source install/setup.bash
ros2 launch imav_bt bt_bench.launch.py
ros2 topic echo /imav_bt/snapshot --field data
```

`bt_bench.launch.py` starts a fake drone and a fake ego-planner
(`scripts/mock_arena.py`) alongside the tree. You will watch it take off, fly
missions 1–3, and land.

Then break it on purpose:

```bash
# ego-planner accepts a goal and then never says anything again.
# This is a REAL failure mode with no status of its own -- see INTERFACES.md.
ros2 launch imav_bt bt_bench.launch.py planner_mode:=silent

# Drain the battery mid-mission and watch the safety branch preempt.
ros2 launch imav_bt bt_bench.launch.py battery_drain_per_min:=20.0
```

The full fault-injection matrix is in [docs/imav_bt/TESTING.md](../docs/imav_bt/TESTING.md).

## Flying it

The flight protocol is unchanged — this package does **not** automate arming or
takeoff:

1. Arm.
2. Take off by hand.
3. Enable Raptor with the RC switch.
4. `ros2 launch imav_bt imav_indoor.launch.py`
5. `ros2 service call /imav_bt/set_running std_srvs/srv/SetBool "{data: true}"`

The tree ticks from step 4 but sits in its `Idle` branch commanding nothing until
step 5. The RC kill switch remains the real override throughout.

```bash
ros2 service call /imav_bt/abort std_srvs/srv/Trigger   # latched abort
ros2 param set /imav_bt flight_plan flight_b            # only while stopped
```

## The tree

```
Root (Selector, memory=False — reactive, re-checked every tick)
├── SafetyOverride            battery / geofence / pose lost / operator abort
│                               └── EmergencyLand
├── FlightPlanRunner (Selector, memory=False)
│   ├── FlightA               ← missions 1-3 chained: this is where the points are
│   │     ArmAndTakeoff
│   │     SkipOnFailure( Mission 1: Obstacle Course )
│   │     SkipOnFailure( Mission 2: Dark Room )
│   │     SkipOnFailure( Mission 3: Hot Spot Drop )
│   │     Precision Landing   moving platform → static → plain descent
│   └── FlightB               mission 4 alone; the rules forbid combining it
│         ArmAndTakeoff
│         SkipOnFailure( Mission 4: Turbine Inspection )
│         LandNearTurbineBase
└── Idle
```

### Why `SkipOnFailure` is on every mission

Rulebook 4.3.3/4.3.4/4.3.6: the multi-mission bonus (2 pts for 3 missions in one
flight) and the landing bonus (2 pts for the moving platform) are each awarded to
**every mission in the flight**. Chaining missions 1–3 into one flight that ends
on the moving platform is therefore worth **+12 points** over flying them
separately.

So a failed mission must not abort the flight — that would forfeit the bonuses
belonging to the missions that already succeeded. One failure would cost points
on work that went fine. `SkipOnFailure` converts a failed mission into SUCCESS so
the sequence continues to the landing.

This is asserted by
`test_skip_on_failure_keeps_the_flight_alive` and
`test_a_failed_mission_still_reaches_the_landing`. If either goes red, the
strategy is broken.

## Configuration

Three plain YAML files in `config/`, read at startup:

| File | Holds |
|---|---|
| `bt.yaml` | tick rate, safety thresholds, topic overrides |
| `arena.yaml` | named anchors, ArUco ids, landing-platform spec |
| `missions.yaml` | per-mission deadlines, attempts, enable flags, stub controls |

> **Every coordinate in `arena.yaml` is a placeholder** and will be wrong on the
> day. Behaviours refer to anchors *by name*, so re-surveying the arena is a
> change to that file only — and the localization backend (OptiTrack, OpenVINS,
> UWB beacons) can change without touching any behaviour.

## Tests

```bash
pytest tests/imav_bt/          # this package only
pytest tests/                  # the whole workspace
# or a single file directly, no ROS graph needed:
python3 tests/imav_bt/test_decorators.py
```

71 tests, no ROS graph, no hardware, sub-second. See
[docs/imav_bt/TESTING.md](../docs/imav_bt/TESTING.md).

## Documentation

| Document | Read it when |
|---|---|
| [ARCHITECTURE.md](../docs/imav_bt/ARCHITECTURE.md) | you want the design rationale and the boundaries |
| [TREE_SEMANTICS.md](../docs/imav_bt/TREE_SEMANTICS.md) | **before writing any tree code** — the py_trees rules that bite |
| [ADDING_A_BEHAVIOR.md](../docs/imav_bt/ADDING_A_BEHAVIOR.md) | you are replacing a stub with a real leaf |
| [ADDING_A_MISSION.md](../docs/imav_bt/ADDING_A_MISSION.md) | you are filling in or adding a mission subtree |
| [INTERFACES.md](../docs/imav_bt/INTERFACES.md) | you need the exact topics, services and status strings |
| [TESTING.md](../docs/imav_bt/TESTING.md) | you are verifying a change, bench to flight |
