# Adding a behaviour

How to replace a `Stub` with a real leaf. You should not need to read
`bt_engine.py` to do this.

Read [TREE_SEMANTICS.md](TREE_SEMANTICS.md) first if you have not — sections 2
(`setup` does not recurse), 4 (the blackboard is global) and 6 (RUNNING means you
own something) are the ones that bite.

---

## Pick a base class

| Base | Use when | `requires_node` |
|---|---|---|
| `Condition` | one yes/no question, answered in a single tick | `False` by default |
| `RosBehaviour` | anything that owns ROS state or takes several ticks | `True` by default |

Both live in `imav_bt/behaviors/base.py`.

---

## Recipe 1 — a condition

Conditions implement `check()` and return `(ok, reason)`. The reason becomes the
py_trees feedback message, so it shows up in `~/snapshot` and in the failure
logs. **Make it specific**: `"battery 0.18 below floor 0.25"` beats `"battery"`.

```python
from imav_bt import blackboard as bb
from imav_bt.behaviors.base import Condition


class AboveAltitude(Condition):
    """SUCCESS when the drone is above `minimum_m`."""

    def __init__(self, name: str, minimum_m: float):
        super().__init__(name)
        self.minimum_m = minimum_m
        # Declare key access up front. A typo raises here, not in flight.
        self._bb = bb.client(f"{name}:alt", read=["position", "odom_valid"])

    def check(self):
        if not self._bb.odom_valid:
            return False, "no odometry"
        altitude = self._bb.position[2]
        if altitude < self.minimum_m:
            return False, f"altitude {altitude:.2f}m below {self.minimum_m:.2f}m"
        return True, f"altitude {altitude:.2f}m"
```

That is the whole thing — no `update()`, no ROS.

---

## Recipe 2 — a leaf that calls a payload service

Multi-tick, owns a service call, so it is a `RosBehaviour`.

```python
from py_trees.common import Status
from std_srvs.srv import Trigger

from imav_bt import interfaces
from imav_bt.behaviors.base import RosBehaviour


class DropCone(RosBehaviour):
    """Call /payload/drop_cone and wait for the reply."""

    def __init__(self, name: str = "DropCone"):
        super().__init__(name)
        self._client = None
        self._future = None

    def setup_ros(self, node):
        # ROS entities here, NEVER in __init__ -- trees get built and thrown
        # away by tests and tooling.
        self._client = node.create_client(
            Trigger, interfaces.SRV_PAYLOAD_DROP_CONE
        )

    def initialise(self):
        self._future = None

    def update(self):
        if not self._client.service_is_ready():
            self.feedback_message = "waiting for /payload/drop_cone"
            return Status.RUNNING          # a Deadline above will bound this

        if self._future is None:
            self._future = self._client.call_async(Trigger.Request())
            self.feedback_message = "drop requested"
            return Status.RUNNING

        if not self._future.done():
            return Status.RUNNING

        result = self._future.result()
        self.feedback_message = result.message
        return Status.SUCCESS if result.success else Status.FAILURE

    def terminate(self, new_status):
        # Release what you own, including when preempted by the safety branch.
        if new_status != Status.RUNNING:
            self._future = None
```

### The four rules that matter

1. **ROS entities in `setup_ros()`, never `__init__`.**
2. **`initialise()` resets per-entry state.** It runs every time the leaf is
   entered, including on a retry.
3. **`terminate()` releases what you own.** It runs when the leaf is preempted,
   not just when it finishes.
4. **Never return RUNNING unboundedly.** If the thing you wait on can fail
   silently, wrap the leaf in a `Deadline`.

---

## Swap it into the tree

Find the stub and replace it. Keep the same name so the tree diagram and any
`missions.yaml` step entry still line up:

```python
# before
step("DropCone", "release the cone into the box below"),

# after
DropCone("DropCone"),
```

Nothing else changes. That is the whole point of building the structure first.

---

## Test it without a ROS graph

Pass a fake node. `test/test_flight.py` has a reusable `FakeNode`; copy the
pattern:

```python
class FakeNode:
    def create_client(self, srv_type, name):
        return FakeClient()
    def get_logger(self):
        return FakeLogger()


def test_drop_cone_succeeds():
    bb.reset()                      # the blackboard is process-global
    leaf = DropCone()
    py_trees.trees.setup(root=leaf, node=FakeNode())   # NOT leaf.setup()
    leaf.tick_once()
    assert leaf.status == Status.RUNNING
```

> `py_trees.trees.setup()`, not `leaf.setup()` — see
> [TREE_SEMANTICS.md §2](TREE_SEMANTICS.md#2-setup-does-not-recurse-and-setup_with_descendants-drops-kwargs).
> With `setup()` the leaf under a decorator silently never receives its node, and
> the failure looks like the planner ignoring you.

Add the file to `CMakeLists.txt` alongside the existing `add_test` entries.

---

## Recording what was scored

If your behaviour earns a scorable objective, mark it immediately after the step
that earns it — so the record reflects what happened rather than what the tree
intended:

```python
from imav_bt.behaviors.bookkeeping import MarkObjective

Sequence("CrossRedWindow", memory=True, children=[
    DetectRedWindow("DetectWindowARed"),
    TraverseWindow("TraverseWindowARed"),
    MarkObjective("Mark", "m1.window_a_red"),
])
```

Convention is `"<mission>.<item>"`. Mission-level outcomes are handled for you by
`SkipOnFailure`; you only need `MarkObjective` for the individual scorable items
inside a mission.

---

## Adding a blackboard key

Do not invent keys at the call site. Declare them in
`imav_bt/blackboard.py:SCHEMA`:

```python
Key(
    "ring_held", bool, lambda: False, "DERIVED",
    "True once VerifyGrab has confirmed the copper ring is held. Gates the "
    "tether-radius guard for the rest of the flight.",
),
```

Pick the writer class honestly — `SENSED` keys are written **only** by
`bt_engine`'s subscriptions. A behaviour writing one would be fabricating sensor
data.

`test_blackboard.py` will then check its default against the declared type
automatically.
