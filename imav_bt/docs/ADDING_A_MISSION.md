# Adding or filling in a mission

Two jobs are covered here:

- **Filling in** one of the four placeholder missions (the common case).
- **Adding** a fifth mission subtree from scratch.

You should not need to open `bt_engine.py` for either.

---

## The contract

Every mission module exposes exactly one function:

```python
def build(config: dict | None, arena: dict | None) -> py_trees.behaviour.Behaviour
```

- `config` — that mission's section of `config/missions.yaml`.
- `arena` — the anchor table from `config/arena.yaml`.
- Returns a tickable behaviour. Both arguments must tolerate `None`, because
  the tests build every subtree bare.

Nothing else. The flight plans call `build()` and wrap the result; the engine
never sees a mission at all.

---

## Filling in a placeholder

Each mission subtree is already a real composite with `Stub` leaves. Replace the
stubs one at a time; the structure and the decorators stay as they are.

```python
# imav_bt/trees/mission3_hotspot.py — before
step("ScanBoxes", "sweep the 3 boxes with the thermal array"),

# after
ScanBoxes("ScanBoxes", sensor_topic="/thermal/array"),
```

Work in this order and you can test at every stage:

1. Replace one stub with a real leaf ([ADDING_A_BEHAVIOR.md](ADDING_A_BEHAVIOR.md)).
2. Run `python3 test/test_trees_smoke.py` — the mission must still tick to
   SUCCESS.
3. Run the bench: `ros2 launch imav_bt bt_bench.launch.py`.
4. Repeat.

**Read the module docstring before you change a mission subtree.** Each one
records the scoring rationale behind its shape — `mission3_hotspot.py` explains
why the LED blink is sequenced before the cone drop, and moving it costs 3
points. Those orderings are not arbitrary.

---

## The patterns already in the tree

### Degradation ladder — try the high-scoring option first

Used wherever the rulebook offers more than one way to score an objective. From
`mission1_obstacle.py`, where a red window is worth 1.0 pt and a blue 0.5:

```python
Selector("Window A", memory=True, children=[
    red_window_attempt,      # 1.0 pt
    blue_window_attempt,     # 0.5 pt
    Stub("Window A: skip", outcome="success"),
])
```

`memory=True` so a failed rung is not retried ahead of the fallback
([TREE_SEMANTICS.md §1](TREE_SEMANTICS.md#1-memory-is-required-and-it-changes-everything)).

### Whether the ladder ends in a skip is a scoring decision

This is the judgement call to get right, and the two cases look identical in
code:

- **Mission 1 ends in a skip.** Its objectives are independent — missing one
  gate costs only that gate's points — so no gate may fail the mission.
- **Mission 2 does not.** If the drone cannot get through a window there is
  nothing to count, so the ladder is allowed to fail and the mission is skipped
  wholesale by its `SkipOnFailure` wrapper.

Ask: *if this step fails, is the rest of the mission still worth anything?*

### Per-step stub control

Open every mission builder with a step factory, so each placeholder can be
configured — and made to fail — from `missions.yaml` without a code change:

```python
from imav_bt.behaviors.stubs import step_factory

def build(config=None, arena=None):
    step = step_factory(config)
    return py_trees.composites.Sequence("Mission N", memory=True, children=[
        step("DoThing", "what the real implementation will do"),
    ])
```

The note becomes the feedback message, so `~/snapshot` doubles as a live list of
what is still stubbed.

---

## Adding a fifth mission

### 1. Write the module

`imav_bt/trees/mission5_example.py`:

```python
"""
mission5_example.py — Mission 5, <name>. PLACEHOLDER.

Rulebook <section> / Table <n>:

    S5 = (<objective terms>) · W5 + ...

Explain here anything about the scoring that dictates the structure -- the
ordering of steps, whether a failed step should be fatal, which option is worth
attempting first. That reasoning is the part a future reader cannot reconstruct.
"""

import py_trees

from imav_bt.behaviors.bookkeeping import MarkObjective
from imav_bt.behaviors.stubs import step_factory

MISSION = "mission5"


def build(config=None, arena=None):
    step = step_factory(config)
    return py_trees.composites.Sequence(
        "Mission 5: Example",
        memory=True,
        children=[
            step("ApproachTarget", "fly to the target anchor"),
            step("DoTheThing", "the scoring action"),
            MarkObjective("Mark done", "m5.done"),
        ],
    )
```

### 2. Add it to a flight plan

In `imav_bt/trees/flight_a.py`, add to `CHAIN`:

```python
from imav_bt.trees import mission5_example

CHAIN = (
    ("mission1", mission1_obstacle),
    ("mission2", mission2_darkroom),
    ("mission3", mission3_hotspot),
    ("mission5", mission5_example),
)
```

`mission_slot()` handles the rest — the `guarded()` decorator stack, the
`enabled` flag and the outcome bookkeeping all come for free.

> **Before adding a fourth mission to FlightA, check the rules.** The
> multi-mission bonus caps at 2 pts for three missions (rulebook 4.3.4); a fourth
> adds its own score but no extra bonus, while lengthening the flight and
> spending battery that the landing bonus depends on. Longer is not always
> better.

### 3. Add its config

In `config/missions.yaml`:

```yaml
  mission5:
    enabled: true
    deadline_s: 120.0
    attempts: 1
    steps: {}
```

### 4. Add its anchors

In `config/arena.yaml`, under `anchors:`. Names, not coordinates, in the tree.

### 5. Test

`test/test_trees_smoke.py` picks new missions up automatically once they are in
`CHAIN`, but add the module to the list in
`test_every_mission_builder_ticks_to_completion`:

```bash
python3 test/test_trees_smoke.py
ros2 launch imav_bt bt_bench.launch.py
```

---

## Deadlines and attempts

Set in `config/missions.yaml`, applied by `guarded()` in
`decorators/policy.py`:

```yaml
  mission3:
    deadline_s: 150.0   # per attempt, not across all attempts
    attempts: 2
```

`deadline_s` bounds **each attempt** — that nesting is deliberate, so one slow
attempt cannot consume the whole budget and starve the retries.

Retries are worth it when a mission is a short there-and-back and a failure is
likely transient (mission 3: an inconclusive thermal scan). They are not worth it
when the mission degrades internally already (mission 1, whose gates each fall
back on their own), because a retry would mostly re-fly gates that already
succeeded.

Remember the whole competition slot is 30 minutes for everything (rulebook 2.2).
Deadlines are a budget, not a safety margin.

---

## What you get for free

Once a mission is in a flight plan:

- **`SkipOnFailure`** — a failed mission does not abort the flight, so the
  missions that succeeded keep their landing and multi-mission bonuses. This is
  worth up to 12 points; see [ARCHITECTURE.md](ARCHITECTURE.md).
- **`Deadline`** — a hung mission cannot stall the flight forever.
- **`AttemptBudget`** — retries, if configured.
- **Outcome recording** — `mission_outcomes` distinguishes `success` from
  `skipped`, which are otherwise indistinguishable to the parent sequence.
- **Safety preemption** — the reactive root interrupts your mission mid-step on
  low battery, geofence breach, pose loss or operator abort. Your leaves get
  `terminate()` called; release what you own there.
