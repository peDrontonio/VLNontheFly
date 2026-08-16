# Testing

Four rungs, cheapest first. Do not skip up the ladder — every rung catches a
class of bug the next one makes expensive to find.

---

## 1. Unit tests — seconds, no ROS graph

```bash
colcon test --packages-select imav_bt --event-handlers console_direct+

# or directly, which is faster while iterating
python3 test/test_decorators.py
python3 test/test_trees_smoke.py
```

71 tests across four files. They tick the **real tree structure** with
placeholder leaves, against a fake node — no `rclpy.init()`, no ROS graph, no
hardware.

| File | Covers |
|---|---|
| `test_blackboard.py` | schema, defaults, type validation, access control |
| `test_decorators.py` | `SkipOnFailure`, `AttemptBudget`, `Deadline`, the `guarded()` stack |
| `test_flight.py` | `GotoAnchor`'s ego-planner handshake |
| `test_trees_smoke.py` | tree assembly, flight plans, safety preemption |

### The two tests that must never go red

**`test_skip_on_failure_keeps_the_flight_alive`** (and its end-to-end sibling
`test_a_failed_mission_still_reaches_the_landing`). These assert that a failed
mission does not abort the flight. If they break, the tree forfeits the landing
and multi-mission bonuses on missions that already succeeded — up to 12 points,
silently. `test_undecorated_sequence_would_lose_the_landing` is kept alongside as
the counterexample, so the reason the decorator exists stays visible.

**`test_stale_status_is_ignored`.** The `goal_status` topic still holds the
previous goal's result when a new goal is published. Without the guard,
`GotoAnchor` reads a leftover `reached` and reports instant success without
moving. That bug looks perfect on the bench and skips waypoints in the air.

### Keeping the suite ROS-free

`test_setup_needs_no_ros_node` guards the property that the whole placeholder
tree can be set up with no node. Please keep it passing — it is what makes this
rung cost one second instead of thirty.

---

## 2. Bench — the whole tree, no aircraft

```bash
ros2 launch imav_bt bt_bench.launch.py
ros2 topic echo /imav_bt/snapshot --field data      # the live tree
ros2 topic echo /imav_bt/status --field data        # one line per tick
```

`scripts/mock_arena.py` provides a fake drone and a fake ego-planner: it publishes
`/odometry` and `/battery_status`, accepts goals on `/move_base_simple/goal`,
flies toward them, and replies on `/planning/goal_status`.

### Fault-injection matrix

Each row reproduces a real failure mode. Run it, and check the tree does the
thing in the last column.

| Command | Simulates | Expected |
|---|---|---|
| `planner_mode:=silent` | **ego-planner accepts a goal then never reports anything again** | `Deadline` fires; the leaf fails instead of hanging forever |
| `planner_mode:=reject` | Goal gate refuses every goal | `GotoAnchor` returns FAILURE promptly; retries then skip |
| `battery_drain_per_min:=20.0` | Battery dies mid-mission | Safety branch preempts within one tick; `EmergencyLand` runs |
| `odom_stops_after_s:=10.0` | Localization lost in flight | Safety branch preempts; `safety_reason` says "odometry stale by …" |
| `payload_fail:=grab_ring` | Jammed grabber | Mission 4 skipped, drone still lands at the base |
| `flight_plan:=flight_b` | The turbine flight | Mission 4 runs; **no** landing-platform objectives recorded |

`planner_mode:=silent` is the important one. It is the failure ego-planner has no
status for — no replan-failure message, no collision message, just silence — and
it is the entire reason `Deadline` is mandatory rather than defensive. See
[INTERFACES.md](INTERFACES.md#-the-gap-there-is-no-failure-status-for-getting-stuck).

### Mission-level fault injection, no code changes

Any placeholder step can be made to fail from `config/missions.yaml`:

```yaml
missions:
  mission2:
    steps:
      ApproachRoom: {outcome: failure}    # success | failure | running
```

Then confirm on the bench that missions 1 and 3 still report `success`, mission 2
reports `skipped`, and the drone still lands. That is the +12-point behaviour,
verified end to end.

`outcome: running` makes a step never finish, which is how you exercise a
`Deadline` from inside a mission rather than from the planner.

---

## 3. Against the real stack, not flying

Confirms `GotoAnchor` sees genuine `goal_status` transitions with nothing armed.

```bash
ros2 launch planner ego_raptor.launch.py start_traj_server:=false
ros2 launch imav_bt imav_indoor.launch.py
ros2 service call /imav_bt/set_running std_srvs/srv/SetBool "{data: true}"
```

`start_traj_server:=false` means the planner plans but never publishes a
trajectory, so nothing actuates. Watch for:

- `/planning/goal_status` carrying `accepted` / `rejected:` / `failed:` strings.
- `/imav_bt/status` reflecting them.
- **QoS actually connecting.** `/odometry` is BEST_EFFORT; if the tree reports
  "no odometry received" while `ros2 topic hz /odometry` is healthy, that is a
  QoS mismatch, not a bug in the guard.

Also worth replaying the bags in `bags/` and `bags_kido/` through any perception
node you add, before it ever runs on the aircraft.

---

## 4. Flight

In this order. Do not compress it.

1. **Tethered hover, tree idle.** Launch the tree but never call `set_running`.
   Confirms it commands nothing while idle.
2. **One `GotoAnchor`.** Set a mission with a single goto step and fly it.
   Confirms the goal handshake end to end.
3. **One mission at a time.** `missions.yaml` → `enabled: false` for the others.
4. **The full FlightA chain.** Only after each mission has flown alone.

Throughout: the RC kill switch is the real override. `~/abort` is a software-level
response and is not a substitute for it.

```bash
ros2 service call /imav_bt/abort std_srvs/srv/Trigger
```

### Reading the flight back

`~/status` carries the mission outcomes and objectives on every tick, so a rosbag
of that one topic is enough to reconstruct what scored:

```bash
ros2 bag record /imav_bt/status /imav_bt/snapshot /planning/goal_status
```

`mission_outcomes` distinguishes `success` from `skipped` — a distinction the
parent sequence cannot see, since `SkipOnFailure` makes both look like SUCCESS.

---

## Adding tests

Plain `unittest`, run directly by CTest — no pytest, no `launch_testing`,
matching `edgellm_vlm_ros/test/`. Add an `add_test` entry in `CMakeLists.txt` and
include the file in the `set_tests_properties` list so it gets `PYTHONPATH`.

Always `bb.reset()` in `setUp()`: the py_trees blackboard is process-global and
state leaks between cases otherwise.

Use `tick_until_terminal()` rather than a fixed tick count — ticking a finished
node **restarts** it, so a fixed count can sail past the result you meant to
assert on. See [TREE_SEMANTICS.md §3](TREE_SEMANTICS.md#3-ticking-a-finished-node-restarts-it).
