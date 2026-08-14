"""
policy.py — the three decorators that encode the competition strategy.

These are small, but they are the most consequential code in the package: they
decide what the drone does when something goes wrong, and getting them subtly
wrong costs points in a way that is invisible on the bench. Hence the dedicated
test file (test/test_decorators.py).

Each one wraps a py_trees built-in rather than reimplementing it. The built-ins
(FailureIsSuccess, Retry, Timeout) already have the right control-flow
semantics; what they lack is the bookkeeping and the injectable clock this
project needs.


Why SkipOnFailure exists (this is the important one)
----------------------------------------------------
IMAV 2026 indoor scoring, rulebook 4.3.3 / 4.3.4 / 4.3.6:

    multi-mission bonus  2 pts for completing 3 missions in a single flight
    landing bonus        2 pts for landing on the moving platform

and crucially, BOTH are awarded to *each* mission in the flight, not once per
flight. Chaining missions 1, 2 and 3 into one flight that ends on the moving
platform is therefore worth 12 points more than flying them separately.

Now consider the naive tree:

    Sequence[ Mission1, Mission2, Mission3, PrecisionLand ]

If Mission2 fails, a Sequence returns FAILURE immediately. The flight never
reaches PrecisionLand, so Mission1 -- which already succeeded -- loses its
landing bonus AND its multi-mission bonus. One failed mission has just cost
points on a mission that went fine.

SkipOnFailure converts a failed child into SUCCESS so the sequence continues:

    Sequence[ Skip(Mission1), Skip(Mission2), Skip(Mission3), PrecisionLand ]

Mission2's own points are lost either way -- that was already true -- but the
flight still lands, and everything else keeps its bonuses. The rule of thumb
this encodes: *finishing the flight is worth more than any single objective.*

The one place NOT to use it is a step the rest of the flight depends on. If
takeoff fails there is nothing to skip to, so ArmAndTakeoff is a bare Sequence
child. See trees/flight_a.py.


Why Deadline is mandatory, not optional
---------------------------------------
ego-planner publishes no replan-failure and no collision status (see
interfaces.py). A goal that gets accepted and then never completes produces
silence, not a failure. Without a wall-clock cap, a GotoAnchor leaf waiting on
`reached` returns RUNNING forever and the flight quietly ends with the drone
hovering until the battery gives out. Deadline is the only mechanism that
observes that failure mode.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

import py_trees
from py_trees import common

from imav_bt import blackboard as bb


class SkipOnFailure(py_trees.decorators.FailureIsSuccess):
    """Convert child FAILURE into SUCCESS so the flight continues.

    Optionally records the result to the ``mission_outcomes`` blackboard key,
    so a post-flight log can distinguish "succeeded" from "was skipped" -- both
    of which look like SUCCESS to the parent sequence.

    Args:
        name: behaviour name.
        child: the subtree to protect.
        mission: mission key to record under (e.g. ``"mission1"``). When None,
            nothing is recorded and this behaves exactly like
            :class:`py_trees.decorators.FailureIsSuccess`.

    Example:
        >>> SkipOnFailure("Skip M2", mission2_subtree, mission="mission2")
    """

    def __init__(
        self,
        name: str,
        child: py_trees.behaviour.Behaviour,
        mission: Optional[str] = None,
    ):
        super().__init__(name=name, child=child)
        self.mission = mission
        self._bb = (
            bb.client(f"{name}:skip", write=["mission_outcomes"])
            if mission is not None
            else None
        )

    def update(self) -> common.Status:
        status = super().update()
        if self._bb is None or status == common.Status.RUNNING:
            return status
        # The child is finished. Distinguish a genuine success from a skip by
        # looking at the child, since super() has already masked FAILURE.
        outcome = (
            bb.Outcome.SKIPPED
            if self.decorated.status == common.Status.FAILURE
            else bb.Outcome.SUCCESS
        )
        # Read-modify-write: the dict is shared, so mutate a copy and reassign
        # to keep py_trees' change tracking meaningful.
        outcomes = dict(self._bb.mission_outcomes)
        outcomes[self.mission] = outcome
        self._bb.mission_outcomes = outcomes
        if outcome == bb.Outcome.SKIPPED:
            self.feedback_message = f"{self.mission} skipped; continuing the flight"
        return status


class AttemptBudget(py_trees.decorators.Retry):
    """Retry a child up to ``attempts`` times, then report FAILURE.

    A thin, clearly-named wrapper over :class:`py_trees.decorators.Retry`
    (whose ``num_failures`` argument reads as "how many failures to tolerate"
    when it actually means "how many total attempts to make").

    Note the child is re-entered from ``initialise()`` on each attempt, so a
    child that publishes a goal will publish it again. That is the intent --
    a retried GotoAnchor should re-send its goal.

    Args:
        name: behaviour name.
        child: the subtree to retry.
        attempts: total attempts, including the first. Must be >= 1.
    """

    def __init__(self, name: str, child: py_trees.behaviour.Behaviour, attempts: int):
        if attempts < 1:
            raise ValueError(f"{name}: attempts must be >= 1, got {attempts}")
        super().__init__(name=name, child=child, num_failures=attempts)

    @property
    def attempts(self) -> int:
        """Total attempts this decorator will make."""
        return self.num_failures


class Deadline(py_trees.decorators.Timeout):
    """Fail a child that is still RUNNING after ``duration`` seconds.

    Identical in behaviour to :class:`py_trees.decorators.Timeout`, except the
    clock is injectable so tests can drive it deterministically instead of
    sleeping. The reimplemented ``initialise``/``update`` are only here because
    the upstream versions call ``time.monotonic()`` directly.

    Args:
        name: behaviour name.
        child: the subtree to bound.
        duration: seconds before the child is cancelled and FAILURE returned.
        time_source: callable returning monotonically increasing seconds.
            Defaults to :func:`time.monotonic`.
    """

    def __init__(
        self,
        name: str,
        child: py_trees.behaviour.Behaviour,
        duration: float,
        time_source: Callable[[], float] = time.monotonic,
    ):
        if duration <= 0.0:
            raise ValueError(f"{name}: duration must be > 0, got {duration}")
        super().__init__(name=name, child=child, duration=duration)
        self._now = time_source

    def initialise(self) -> None:
        self.finish_time = self._now() + self.duration
        self.feedback_message = ""

    def update(self) -> common.Status:
        if self.decorated.status != common.Status.RUNNING:
            self.feedback_message = "child finished before the deadline"
            return self.decorated.status

        remaining = self.finish_time - self._now()
        if remaining <= 0.0:
            self.feedback_message = f"deadline exceeded after {self.duration:.1f}s"
            # Cancel the child so it releases whatever it owns (e.g. a goal).
            self.decorated.stop(common.Status.INVALID)
            return common.Status.FAILURE

        self.feedback_message = f"running [{remaining:.1f}s remaining]"
        return common.Status.RUNNING


def guarded(
    name: str,
    child: py_trees.behaviour.Behaviour,
    *,
    deadline: float,
    attempts: int = 1,
    mission: Optional[str] = None,
    time_source: Callable[[], float] = time.monotonic,
) -> py_trees.behaviour.Behaviour:
    """Apply the standard decorator stack to a mission subtree.

    The nesting order matters and is easy to get backwards, so this helper
    exists to make the house pattern the path of least resistance::

        SkipOnFailure( AttemptBudget( Deadline( child ) ) )

    Read outside-in: *skip the mission if, after N attempts each bounded by a
    deadline, it still has not succeeded.*

    Putting Deadline outermost instead would bound the whole retry sequence
    rather than each attempt, so one slow attempt would eat the entire budget
    and the retries would never happen.

    Args:
        name: base name; the decorators append their own suffixes.
        child: the mission subtree.
        deadline: per-attempt wall-clock cap in seconds.
        attempts: how many attempts to make before giving up.
        mission: mission key for outcome bookkeeping. When None, failure still
            converts to SUCCESS but nothing is recorded.
        time_source: injectable clock, forwarded to Deadline.

    Returns:
        The decorated subtree, ready to add to a sequence.
    """
    node: py_trees.behaviour.Behaviour = Deadline(
        name=f"{name} Deadline", child=child, duration=deadline,
        time_source=time_source,
    )
    if attempts > 1:
        node = AttemptBudget(name=f"{name} Attempts", child=node, attempts=attempts)
    return SkipOnFailure(name=f"{name} Skip", child=node, mission=mission)
