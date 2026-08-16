# Tree semantics — the py_trees rules that actually bite

Read this before writing tree code. Every item below is something that either
bit us while building this package, or would have.

Version: `py_trees` 2.5.0.

---

## 1. `memory` is required, and it changes everything

In py_trees 2.x, `Sequence` and `Selector` take `memory` as a **required**
argument. There is no default, and the two settings produce genuinely different
robots.

```python
py_trees.composites.Sequence("name", memory=True, children=[...])
py_trees.composites.Selector("name", memory=False, children=[...])
```

**`memory=True`** — resume where you left off. The composite remembers which
child was RUNNING and ticks straight into it. Children that already finished are
not re-ticked.

**`memory=False`** — re-evaluate from the top, every tick. Earlier children get
re-ticked even if they succeeded last time. This is "priority interrupt"
behaviour.

### The rules this package uses

| Composite | memory | Why |
|---|---|---|
| `Root` selector | `False` | So the safety branch is re-checked **every tick** and can preempt a running mission |
| `SafetyOverride` sequence | `False` | So `SafetyTripped` is re-evaluated every tick, not just once when recovery started |
| `FlightPlanRunner` selector | `False` | So a flight-plan change takes effect on the next tick |
| `FlightA` / `FlightB` sequence | `True` | So the flight resumes mid-mission instead of restarting from takeoff |
| Mission sequences | `True` | So a completed step is not redone |
| Degradation ladders (`Selector`) | `True` | So a failed option is not retried ahead of the fallback |

### Getting the root wrong is a crash, not a style issue

With `memory=True` on the root, the selector would resume directly into the
running mission branch and **never look at the safety check again** until that
branch finished. A battery that dies mid-mission would go unnoticed.

That is the single most important line in the package:

```python
py_trees.composites.Selector("Root", memory=False, children=[...])
```

### Why ladders need `memory=True`

A degradation ladder tries the high-scoring option first and falls back:

```python
Selector("Window A", memory=True, children=[
    red_window_attempt,    # 1.0 pt
    blue_window_attempt,   # 0.5 pt
    skip,
])
```

With `memory=False` this would re-tick `red_window_attempt` from the top on
every tick and never make progress down the ladder. With `memory=True`, once
`red_window_attempt` fails the selector advances and stays there — for as long as
the selector itself remains RUNNING.

That last clause matters: a Selector's `current_child` resets to `children[0]`
whenever the selector's own status is not RUNNING. So the ladder resets between
*attempts*, not within one.

---

## 2. `setup()` does not recurse, and `setup_with_descendants()` drops kwargs

This one cost real debugging time. Three functions look interchangeable and are
not:

| Call | Recurses? | Passes kwargs? |
|---|---|---|
| `behaviour.setup(node=n)` | ❌ | ✅ (to that one behaviour) |
| `behaviour.setup_with_descendants()` | ✅ | ❌ **takes no arguments at all** |
| `py_trees.trees.setup(root=r, node=n)` | ✅ | ✅ |

Only the third does what you want. Use it:

```python
py_trees.trees.setup(root=self.root, node=self)   # correct
self.root.setup(node=self)                        # silently sets up ONE node
```

The failure is quiet and confusing: a `GotoAnchor` under a `Deadline` never gets
its `node`, so `self._publisher` stays `None`, and the leaf logs "goal → …"
cheerfully while publishing nothing at all. It looks like the planner is
ignoring you.

`BehaviourTree.setup(node=…)` also works — it delegates to `py_trees.trees.setup`.

---

## 3. Ticking a finished node restarts it

A node whose previous status was SUCCESS or FAILURE gets `initialise()` called
again on the next tick. There is no "done" state.

In a real tree this rarely matters, because the parent composite stops ticking a
finished child. In a **test** it matters constantly:

```python
# WRONG: sails past the result into a second run
status = tick(node, times=10)

# RIGHT: stop at the first terminal status, like a parent composite would
status = tick_until_terminal(node)
```

`tests/imav_bt/test_decorators.py` has a `tick_until_terminal` helper for this. The bug it
prevents: an `AttemptBudget(attempts=3)` given a permanently-failing child
returns FAILURE on tick 3, then **restarts** and is RUNNING again on tick 4.
Asserting at a fixed tick count observes the second cycle.

---

## 4. The blackboard is process-global

`py_trees.blackboard.Blackboard.storage` is a **class attribute**, shared by
every tree, client and test in the process.

Always call `blackboard.reset()` in `setUp()`:

```python
def setUp(self):
    bb.reset()
```

Without it, state leaks between test cases and failures depend on test ordering.

Access control is enforced, and that is a feature — a client registered READ
raises `AttributeError` on write:

```python
reader = bb.client("x", read=["running"])
reader.running = True        # AttributeError
```

Declare every key in `imav_bt/blackboard.py:SCHEMA`. `bb.client()` validates
names at construction, so a typo fails loudly instead of silently creating a key
nothing ever writes.

---

## 5. Decorators wrap exactly one child

`py_trees.decorators.Decorator.__init__(self, name, child)` — one child, and the
signature is positional-ish. To decorate several things, wrap a composite.

The nesting order carries meaning. This package's house stack is built by
`decorators.policy.guarded()`:

```
SkipOnFailure( AttemptBudget( Deadline( child ) ) )
```

Read outside-in: *skip the mission if, after N attempts each bounded by a
deadline, it still has not succeeded.*

Putting `Deadline` outermost instead would bound the **whole retry sequence**, so
one slow attempt would consume the entire budget and the retries would never
happen. `test_deadline_applies_per_attempt_not_across_all_attempts` pins this.

---

## 6. RUNNING means "I own something"

A leaf that returns RUNNING is holding state — a published goal, a service call
in flight, a servo mid-travel. Two obligations follow.

**Release it in `terminate()`.** py_trees calls `terminate(new_status)` when the
behaviour stops for any reason, including being preempted by a higher-priority
branch. `GotoAnchor` clears its `active_goal` there, so a stale target cannot
mislead the next leaf:

```python
def terminate(self, new_status):
    if new_status != Status.RUNNING:
        self._bb.active_goal = None
```

**Never return RUNNING forever with no external bound.** If the thing you are
waiting on can fail silently, you cannot detect it from inside the leaf. That is
exactly the ego-planner situation (see [INTERFACES.md](INTERFACES.md)), and it is
why `GotoAnchor` must be wrapped in a `Deadline` — use the `goto()` helper, which
does it for you.

---

## 7. Build the tree without side effects

Constructing a tree must be free of side effects: trees get built, rendered and
thrown away by tests and by tooling. Publishers, subscriptions and service
clients belong in `setup_ros()`, which only runs when someone intends to fly.

`RosBehaviour` enforces the split. Leaves that need no ROS at all set
`requires_node = False`, which is what lets the entire placeholder tree be ticked
in a unit test with no ROS graph. `test_setup_needs_no_ros_node` guards that
property — please keep it passing.

---

## 8. Rendering the tree

```python
py_trees.display.unicode_tree(root, show_status=True)
```

`bt_engine` publishes this on `~/snapshot` every N ticks:

```bash
ros2 topic echo /imav_bt/snapshot --field data
```

`feedback_message` is what shows up next to each node, so make it specific.
"battery 0.18 below floor 0.25" beats "battery". Stubs put their TODO note there,
which turns the snapshot into a live list of what is not implemented yet.
