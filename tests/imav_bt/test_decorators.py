#!/usr/bin/env python3
"""
test_decorators.py — the policy decorators, which encode the scoring strategy.

The most important case in this file is
`test_skip_on_failure_keeps_the_flight_alive`: it asserts the exact property
the +12-point mission-chaining strategy depends on. If that test ever goes red,
the tree will abort a flight on the first failed mission and silently forfeit
the landing and multi-mission bonuses on the missions that already succeeded.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "imav_bt"))

import py_trees  # noqa: E402
from py_trees.common import Status  # noqa: E402

from imav_bt import blackboard as bb  # noqa: E402
from imav_bt.decorators.policy import (  # noqa: E402
    AttemptBudget,
    Deadline,
    SkipOnFailure,
    guarded,
)


class Scripted(py_trees.behaviour.Behaviour):
    """A leaf that returns a scripted sequence of statuses, and counts entries.

    The last status repeats once the script runs out, so a test can tick as
    many times as it likes without falling off the end.
    """

    def __init__(self, name, statuses):
        super().__init__(name)
        self.script = list(statuses)
        self.index = 0
        self.entries = 0  # how many times initialise() ran = attempts made

    def initialise(self):
        self.entries += 1

    def update(self):
        status = self.script[min(self.index, len(self.script) - 1)]
        self.index += 1
        return status


class FakeClock:
    """Manually advanced monotonic clock, so deadlines are tested without sleeping."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


def tick(node, times=1):
    """Tick a behaviour n times and return its final status."""
    status = None
    for _ in range(times):
        node.tick_once()
        status = node.status
    return status


def tick_until_terminal(node, limit=50):
    """Tick until the node returns SUCCESS or FAILURE, then stop.

    Ticking a node that has already finished RESTARTS it -- py_trees calls
    initialise() again because the previous status was not RUNNING. So a test
    that ticks a fixed number of times can sail straight past the result it
    meant to assert on and observe a second run instead. In a real tree the
    parent composite stops ticking a finished child, which is what this helper
    emulates. See docs/TREE_SEMANTICS.md.
    """
    for _ in range(limit):
        node.tick_once()
        if node.status in (Status.SUCCESS, Status.FAILURE):
            return node.status
    raise AssertionError(f"{node.name} never reached a terminal status in {limit} ticks")


class TestSkipOnFailure(unittest.TestCase):
    def setUp(self):
        bb.reset()

    def test_failure_becomes_success(self):
        node = SkipOnFailure("s", Scripted("c", [Status.FAILURE]))
        self.assertEqual(tick(node), Status.SUCCESS)

    def test_success_passes_through(self):
        node = SkipOnFailure("s", Scripted("c", [Status.SUCCESS]))
        self.assertEqual(tick(node), Status.SUCCESS)

    def test_running_passes_through(self):
        node = SkipOnFailure("s", Scripted("c", [Status.RUNNING]))
        self.assertEqual(tick(node), Status.RUNNING)

    def test_records_skip_distinctly_from_success(self):
        """Both outcomes look like SUCCESS to the parent, so the record is the
        only way to tell afterwards what actually happened."""
        failed = SkipOnFailure("s1", Scripted("c", [Status.FAILURE]), mission="mission2")
        tick(failed)
        self.assertEqual(
            bb.snapshot()["mission_outcomes"], {"mission2": bb.Outcome.SKIPPED}
        )

        ok = SkipOnFailure("s2", Scripted("c", [Status.SUCCESS]), mission="mission1")
        tick(ok)
        self.assertEqual(
            bb.snapshot()["mission_outcomes"],
            {"mission2": bb.Outcome.SKIPPED, "mission1": bb.Outcome.SUCCESS},
        )

    def test_nothing_recorded_while_running(self):
        node = SkipOnFailure("s", Scripted("c", [Status.RUNNING]), mission="mission1")
        tick(node)
        self.assertEqual(bb.snapshot()["mission_outcomes"], {})

    def test_skip_on_failure_keeps_the_flight_alive(self):
        """*** The property the whole chaining strategy rests on. ***

        A failed mission in the middle of FlightA must not prevent the flight
        from reaching PrecisionLand, because the landing and multi-mission
        bonuses are awarded per-mission and would be lost for the missions
        that already succeeded.
        """
        landed = Scripted("PrecisionLand", [Status.SUCCESS])
        flight = py_trees.composites.Sequence(
            "FlightA",
            memory=True,
            children=[
                SkipOnFailure("m1", Scripted("M1", [Status.SUCCESS]), mission="mission1"),
                SkipOnFailure("m2", Scripted("M2", [Status.FAILURE]), mission="mission2"),
                SkipOnFailure("m3", Scripted("M3", [Status.SUCCESS]), mission="mission3"),
                landed,
            ],
        )
        self.assertEqual(tick(flight), Status.SUCCESS)
        self.assertEqual(landed.entries, 1, "PrecisionLand never ran")
        self.assertEqual(
            bb.snapshot()["mission_outcomes"],
            {
                "mission1": bb.Outcome.SUCCESS,
                "mission2": bb.Outcome.SKIPPED,
                "mission3": bb.Outcome.SUCCESS,
            },
        )

    def test_undecorated_sequence_would_lose_the_landing(self):
        """The counterexample the decorator exists to prevent."""
        landed = Scripted("PrecisionLand", [Status.SUCCESS])
        flight = py_trees.composites.Sequence(
            "FlightA (naive)",
            memory=True,
            children=[
                Scripted("M1", [Status.SUCCESS]),
                Scripted("M2", [Status.FAILURE]),
                Scripted("M3", [Status.SUCCESS]),
                landed,
            ],
        )
        self.assertEqual(tick(flight), Status.FAILURE)
        self.assertEqual(landed.entries, 0)


class TestAttemptBudget(unittest.TestCase):
    def setUp(self):
        bb.reset()

    def test_rejects_zero_attempts(self):
        with self.assertRaises(ValueError):
            AttemptBudget("a", Scripted("c", [Status.SUCCESS]), attempts=0)

    def test_succeeds_without_retrying_when_the_child_succeeds(self):
        child = Scripted("c", [Status.SUCCESS])
        node = AttemptBudget("a", child, attempts=3)
        self.assertEqual(tick(node), Status.SUCCESS)
        self.assertEqual(child.entries, 1)

    def test_gives_up_after_exactly_n_attempts(self):
        child = Scripted("c", [Status.FAILURE])
        node = AttemptBudget("a", child, attempts=3)
        self.assertEqual(tick_until_terminal(node), Status.FAILURE)
        self.assertEqual(child.entries, 3, "should make exactly `attempts` attempts")

    def test_recovers_when_a_later_attempt_succeeds(self):
        child = Scripted("c", [Status.FAILURE, Status.SUCCESS])
        node = AttemptBudget("a", child, attempts=3)
        self.assertEqual(tick_until_terminal(node), Status.SUCCESS)
        self.assertEqual(child.entries, 2)

    def test_attempts_property_reads_back(self):
        node = AttemptBudget("a", Scripted("c", [Status.SUCCESS]), attempts=4)
        self.assertEqual(node.attempts, 4)


class TestDeadline(unittest.TestCase):
    def setUp(self):
        bb.reset()

    def test_rejects_non_positive_duration(self):
        with self.assertRaises(ValueError):
            Deadline("d", Scripted("c", [Status.RUNNING]), duration=0.0)

    def test_running_child_survives_until_the_deadline(self):
        clock = FakeClock()
        node = Deadline("d", Scripted("c", [Status.RUNNING]), 5.0, time_source=clock)
        self.assertEqual(tick(node), Status.RUNNING)
        clock.advance(4.9)
        self.assertEqual(tick(node), Status.RUNNING)

    def test_fires_once_the_deadline_passes(self):
        """The silent-stall case: ego-planner accepts a goal and then says
        nothing further, so only wall-clock time can detect the failure."""
        clock = FakeClock()
        child = Scripted("c", [Status.RUNNING])
        node = Deadline("d", child, 5.0, time_source=clock)
        tick(node)
        clock.advance(5.1)
        self.assertEqual(tick(node), Status.FAILURE)
        self.assertEqual(child.status, Status.INVALID, "child should be cancelled")

    def test_child_finishing_early_passes_straight_through(self):
        clock = FakeClock()
        node = Deadline("d", Scripted("c", [Status.SUCCESS]), 5.0, time_source=clock)
        self.assertEqual(tick(node), Status.SUCCESS)


class TestGuardedStack(unittest.TestCase):
    """`guarded()` composes the three decorators; the nesting order matters."""

    def setUp(self):
        bb.reset()

    def test_deadline_applies_per_attempt_not_across_all_attempts(self):
        """If Deadline were outermost, one slow attempt would consume the whole
        budget and the retries would never happen."""
        clock = FakeClock()
        child = Scripted("c", [Status.RUNNING])
        node = guarded(
            "M", child, deadline=1.0, attempts=3, mission="mission1",
            time_source=clock,
        )
        for _ in range(3):  # let each attempt time out in turn
            tick(node)
            clock.advance(1.1)
            tick(node)
        self.assertEqual(child.entries, 3, "each attempt should get its own deadline")

    def test_stack_skips_the_mission_and_records_it(self):
        clock = FakeClock()
        node = guarded(
            "M", Scripted("c", [Status.FAILURE]), deadline=1.0, attempts=2,
            mission="mission3", time_source=clock,
        )
        self.assertEqual(
            tick_until_terminal(node), Status.SUCCESS, "failure must be masked"
        )
        self.assertEqual(
            bb.snapshot()["mission_outcomes"], {"mission3": bb.Outcome.SKIPPED}
        )

    def test_single_attempt_omits_the_retry_layer(self):
        node = guarded("M", Scripted("c", [Status.SUCCESS]), deadline=1.0, attempts=1)
        self.assertIsInstance(node, SkipOnFailure)
        self.assertIsInstance(node.decorated, Deadline)


if __name__ == "__main__":
    unittest.main(verbosity=2)
