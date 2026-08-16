# Architecture

## What this package is for

VLNontheFly could already fly and avoid obstacles before this package existed.
What it could not do was *decide*: run mission 2 now, give up on that window and
move on, abort because the battery is low. That decision layer is what `imav_bt`
adds.

The only arbitration in the repo before this was
`edgellm_vlm_ros/scripts/vlm_nav_supervisor.py` — a four-state machine
(`POINT_NAV` / `ALTITUDE_ADJUST` / `SETTLE` / `HOLD`) that knows how to explore
toward open space and nothing else. It has no concept of missions, retries,
flight phases or scoring, and it does not even know the region gate exists.
Growing it into a four-mission controller would have produced an unmaintainable
switch statement.

## Where it sits

```
                    ┌──────────────────────────────────┐
                    │            imav_bt               │   decides WHAT
                    │  safety · plans · missions       │
                    └───────┬──────────────────▲───────┘
       PoseStamped goal     │                  │  String status
                            ▼                  │
                 /move_base_simple/goal   planning/goal_status
                            │                  │
                    ┌───────▼──────────────────┴───────┐
                    │          ego_planner             │   decides HOW
                    │   grid_map · replan FSM · bspline│
                    └───────────────┬──────────────────┘
                                    ▼
                    traj_server → /drone_0_planning/pos_cmd
                                    ▼
                    pos_cmd_to_raptor → /fmu/in/trajectory_setpoint_raptor
                                    ▼
                              PX4 / Raptor (EXTERNAL)
```

**The tree never writes actuator commands.** That is the central boundary. All
motion goes through the existing path, so the safety properties already
established there — the Raptor activation target, the RC override, the offboard
heartbeat failsafe — are untouched by anything in this package. The worst a
buggy behaviour can do is send a bad *goal*, which ego-planner's own goal gate
will reject.

## The scoring drives the structure

The top-level shape is not a design preference; it falls out of the rulebook.

**Rulebook 4.3.3 / 4.3.4 / 4.3.6.** The multi-mission bonus (2 pts for three
missions in one flight) and the landing bonus (2 pts for the moving platform) are
each awarded to *every mission in the flight*. Chaining missions 1–3 into a
single flight ending on the moving platform is worth **+12 points** over flying
them separately. The rulebook says this is intentional, "to encourage efficient
mission chaining".

Consequence: **finishing the flight is worth more than any single mission.** A
mission that fails must not abort the flight, or the missions that already
succeeded lose their bonuses too. Hence `SkipOnFailure` on every mission slot
rather than a plain `Sequence`. See `decorators/policy.py`.

**Rulebook 4.4.8.** Mission 4 "cannot be combined with the others and does not
require landing on the landing pad" — and Table 5 confirms it structurally, with
no `La` and no `B` term. Hence two flight plans rather than one ordering of four
missions.

**Rulebook 4.1 / 4.3.1.** The autonomy factor `A` multiplies the raw score before
the mass factor: 1.0 fully onboard, 0.7 for off-board computation *or* for using
beacons. That is roughly 4 points per mission after the mass factor, ~12 across
missions 1–3. Everything here runs onboard the Orin.

Two scoring subtleties are encoded as *behaviour*, because leaving them to a
human under time pressure would lose points:

- **Mission 2 counting is asymmetric.** `Ba = 3·(Counted/Nba_max)`, but if
  `Counted > Nba_max` the term collapses to **0**. Undercounting costs a
  fraction; overcounting costs everything. The counting behaviour must be tuned
  against false positives, not for balanced accuracy.
- **Mission 3's `Hb` = 3 pts is awarded solely on the LED blink** (rulebook
  4.4.6, "based SOLELY on this criterion"), not on the cone landing. So the tree
  blinks *before* it drops, and the blink is never nested under the drop.

## Design rules

### The tree is the sole goal publisher

Three things in this workspace can publish to `/move_base_simple/goal` — this
tree, `vlm_region_gate` with `auto_execute: true`, and `vlm_nav_supervisor`. Any
two of them running together fight over the drone. In this phase the VLM stack is
simply not launched. See [INTERFACES.md](INTERFACES.md#who-must-not-run-at-the-same-time).

### Pose is pluggable

Every behaviour reads pose from `/odometry` and nothing else, and every waypoint
is a **named anchor** resolved through `config/arena.yaml` — never a literal
coordinate in code.

This is not tidiness. OptiTrack will not be available at INSA Strasbourg and the
replacement is undecided; using beacons would cost `A` = 0.7. Because behaviours
only ever ask for "the room face", swapping the localization backend is a change
to one YAML file.

### A missing sensor must not ground the drone — except pose

The two guards take deliberately opposite stances:

- **Battery**: does nothing until a battery message has actually arrived. If the
  topic is unmapped on a given airframe, the guard stays quiet rather than
  tripping on a default of 0.0 and refusing to fly.
- **Localization**: absence *is* the emergency. ego-planner cannot plan without
  pose and rejects every goal with `rejected:not_ready`, so a drone with no
  odometry is not going to do anything useful anyway.

### Everything external is behind a stub

Payload, perception and the planner all have mock backends, so the whole tree
ticks with zero hardware from day one. That is what makes the fault-injection
matrix in [TESTING.md](TESTING.md) a one-line launch argument instead of a
flight test.

### Build the tree without side effects

Constructing a tree is free of side effects; ROS entities are created in
`setup_ros()`. Leaves that need no ROS set `requires_node = False`, which is what
lets the entire placeholder tree be unit-tested with no ROS graph.

## Why not py_trees_ros

`py_trees_ros` provides tree-introspection topics and the `py-trees-tree-watcher`
CLI, which are genuinely useful. It is apt-only (`ros-humble-py-trees-ros`) and is
not installed on this machine, so depending on it would mean shipping code that
could not be run or tested here.

Instead `bt_engine` publishes a rendered snapshot as a plain string on
`~/snapshot`, which covers the same need with no extra dependency:

```bash
ros2 topic echo /imav_bt/snapshot --field data
```

If you later install `py_trees_ros`, swapping in its `BehaviourTree` is a change
to `bt_engine.py` alone — no behaviour and no subtree is affected.

## Threading

Everything runs on the single-threaded default executor: subscriptions write
blackboard keys, then the timer ticks the tree. A tick therefore always sees a
self-consistent snapshot, and no behaviour needs a lock. Do not move the tick
onto its own callback group without revisiting that.

## Deliberately out of scope

Currently placeholders or absent, with the interfaces they will plug into already
defined:

| Not built | Plugs into |
|---|---|
| Mission 1–4 logic | `trees/mission*.py`, all leaves are `Stub` |
| ArUco detection | new node; ids recorded in `config/arena.yaml` |
| Gate / window / bar detection | new node behind the same detection interface |
| Doll counting | `baby_counter` — the objective classical CV handles worst |
| Thermal sensing | new node; no thermal code exists in the repo |
| Payload drivers | `scripts/payload_node.py`, `backend: mock` |
| Score calculator | rulebook Tables 2–5; `objectives` bookkeeping already records the inputs |
| Arming / takeoff | `behaviors/flight.py`; still a manual step in the flight protocol |
| Re-attaching the VLM | needs a parameter callback in `vlm_node.cpp` first |
