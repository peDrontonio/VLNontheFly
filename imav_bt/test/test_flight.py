#!/usr/bin/env python3
"""
test_flight.py — GotoAnchor's handshake with ego-planner.

The case that matters most is `test_stale_status_is_ignored`. The status topic
still holds the previous goal's result when a new goal is published, so a leaf
that reads it naively reports instant success without having moved. That bug
would look like a working tree on the bench and a drone that skips waypoints in
the air.

Runs with a fake node -- no rclpy.init(), no ROS graph.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import py_trees  # noqa: E402
from py_trees.common import Status  # noqa: E402

from imav_bt import blackboard as bb  # noqa: E402
from imav_bt.behaviors.flight import GotoAnchor, goto, resolve_anchor  # noqa: E402
from imav_bt.decorators.policy import Deadline  # noqa: E402

ANCHORS = {"takeoff_pad": [0.0, 0.0, 1.2], "room_face": [4.0, 1.5, 1.4]}


class FakePublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeLogger:
    def info(self, msg):
        pass

    def warn(self, msg):
        pass


class FakeNode:
    """Just enough rclpy.Node surface for a behaviour's setup_ros()."""

    def __init__(self):
        self.publisher = FakePublisher()

    def create_publisher(self, msg_type, topic, qos):
        self.topic = topic
        return self.publisher

    def get_logger(self):
        return FakeLogger()

    def get_clock(self):
        return self

    def now(self):
        return self

    def to_msg(self):
        from builtin_interfaces.msg import Time

        return Time()


class FakeClock:
    def __init__(self):
        self.t = 100.0  # start non-zero, so "stamp 0" reads as genuinely old

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


class GotoFixture(unittest.TestCase):
    def setUp(self):
        bb.reset()
        self.clock = FakeClock()
        self.node = FakeNode()
        self.bb = bb.client(
            "test",
            write=["goal_status", "goal_status_stamp", "position", "odom_valid"],
        )

    def make(self, **kwargs):
        kwargs.setdefault("anchor", "room_face")
        kwargs.setdefault("anchors", ANCHORS)
        leaf = GotoAnchor(name="Goto", time_source=self.clock, **kwargs)
        self.setup_tree(leaf)
        return leaf

    def setup_tree(self, root):
        """Set up a whole subtree, delivering the `node` kwarg to every leaf.

        Behaviour.setup() does NOT recurse, and setup_with_descendants() does
        recurse but passes no kwargs -- so neither one gets `node` down to a
        leaf that sits under a decorator. py_trees.trees.setup() is the only
        one that both crawls and distributes kwargs. See docs/TREE_SEMANTICS.md.
        """
        py_trees.trees.setup(root=root, node=self.node)
        return root

    def set_status(self, status):
        """Publish a status stamped at the current (fake) time."""
        self.bb.goal_status = status
        self.bb.goal_status_stamp = self.clock()


class TestAnchorResolution(unittest.TestCase):
    def test_resolves_a_known_anchor(self):
        self.assertEqual(resolve_anchor("room_face", ANCHORS), (4.0, 1.5, 1.4))

    def test_unknown_anchor_lists_the_known_ones(self):
        with self.assertRaises(KeyError) as ctx:
            resolve_anchor("nope", ANCHORS)
        self.assertIn("room_face", str(ctx.exception))

    def test_malformed_anchor_is_rejected(self):
        with self.assertRaises(ValueError):
            resolve_anchor("bad", {"bad": [1.0, 2.0]})


class TestGotoAnchor(GotoFixture):
    def test_requires_exactly_one_of_position_or_anchor(self):
        with self.assertRaises(ValueError):
            GotoAnchor("g")
        with self.assertRaises(ValueError):
            GotoAnchor("g", position=(0, 0, 1), anchor="room_face", anchors=ANCHORS)

    def test_publishes_the_goal_once_on_entry(self):
        leaf = self.make()
        leaf.tick_once()
        self.assertEqual(len(self.node.publisher.published), 1)
        msg = self.node.publisher.published[0]
        self.assertAlmostEqual(msg.pose.position.x, 4.0)
        self.assertAlmostEqual(msg.pose.position.z, 1.4)
        self.assertEqual(msg.header.frame_id, "map")
        self.assertAlmostEqual(msg.pose.orientation.w, 1.0)

    def test_waits_when_the_planner_has_said_nothing(self):
        leaf = self.make()
        leaf.tick_once()
        self.assertEqual(leaf.status, Status.RUNNING)

    def test_stale_status_is_ignored(self):
        """*** The trap. *** A 'reached' left over from the PREVIOUS goal must
        not make the new goal succeed instantly."""
        self.set_status("reached")  # stamped before this leaf publishes
        self.clock.advance(1.0)

        leaf = self.make()
        leaf.tick_once()
        self.assertEqual(
            leaf.status, Status.RUNNING, "leftover 'reached' was treated as ours"
        )

        # A genuinely new 'reached' does complete it.
        self.clock.advance(1.0)
        self.set_status("reached")
        leaf.tick_once()
        self.assertEqual(leaf.status, Status.SUCCESS)

    def test_succeeds_on_reached(self):
        leaf = self.make()
        leaf.tick_once()
        self.clock.advance(1.0)
        self.set_status("reached")
        leaf.tick_once()
        self.assertEqual(leaf.status, Status.SUCCESS)

    def test_running_while_accepted(self):
        for status in ("accepted", "accepted:safety_bubble"):
            with self.subTest(status=status):
                bb.reset()
                leaf = self.make()
                leaf.tick_once()
                self.clock.advance(1.0)
                self.set_status(status)
                leaf.tick_once()
                self.assertEqual(leaf.status, Status.RUNNING)

    def test_fails_on_rejection_or_planning_failure(self):
        for status in ("rejected:not_ready", "rejected:z", "failed:plan"):
            with self.subTest(status=status):
                bb.reset()
                leaf = self.make()
                leaf.tick_once()
                self.clock.advance(1.0)
                self.set_status(status)
                leaf.tick_once()
                self.assertEqual(leaf.status, Status.FAILURE)

    def test_unknown_status_keeps_waiting_rather_than_guessing(self):
        leaf = self.make()
        leaf.tick_once()
        self.clock.advance(1.0)
        self.set_status("something:new")
        leaf.tick_once()
        self.assertEqual(leaf.status, Status.RUNNING)

    def test_arrival_tolerance_succeeds_without_planner_feedback(self):
        leaf = self.make(arrival_tolerance_m=0.5)
        self.bb.odom_valid = True
        self.bb.position = (4.1, 1.5, 1.4)  # 0.1 m from the anchor
        leaf.tick_once()
        self.assertEqual(leaf.status, Status.SUCCESS)

    def test_arrival_tolerance_is_off_by_default(self):
        leaf = self.make()
        self.bb.odom_valid = True
        self.bb.position = (4.0, 1.5, 1.4)  # exactly on target
        leaf.tick_once()
        self.assertEqual(leaf.status, Status.RUNNING, "planner should be authoritative")

    def test_active_goal_is_claimed_then_released(self):
        leaf = self.make()
        leaf.tick_once()
        self.assertEqual(bb.snapshot()["active_goal"], (4.0, 1.5, 1.4))
        self.clock.advance(1.0)
        self.set_status("reached")
        leaf.tick_once()
        self.assertIsNone(bb.snapshot()["active_goal"], "goal ownership must be released")

    def test_explicit_position_is_used_verbatim(self):
        leaf = self.make(anchor=None, position=(1.0, 2.0, 3.0))
        leaf.tick_once()
        msg = self.node.publisher.published[0]
        self.assertAlmostEqual(msg.pose.position.y, 2.0)


class TestGotoHelper(GotoFixture):
    def test_goto_wraps_in_a_deadline(self):
        node = goto("Go", anchor="room_face", anchors=ANCHORS, deadline=5.0)
        self.assertIsInstance(node, Deadline)  # noqa: E501
        self.assertIsInstance(node.decorated, GotoAnchor)

    def test_goto_deadline_rescues_a_silent_planner(self):
        """The failure mode ego-planner cannot report: accepted, then silence."""
        node = goto(
            "Go", anchor="room_face", anchors=ANCHORS, deadline=10.0,
            time_source=self.clock,
        )
        self.setup_tree(node)

        node.tick_once()
        self.clock.advance(1.0)
        self.set_status("accepted")
        node.tick_once()
        self.assertEqual(node.status, Status.RUNNING)

        self.clock.advance(20.0)  # planner never says anything again
        node.tick_once()
        self.assertEqual(node.status, Status.FAILURE, "deadline should have fired")

    def test_goto_with_retries_resends_the_goal(self):
        node = goto(
            "Go", anchor="room_face", anchors=ANCHORS, deadline=5.0, attempts=2,
            time_source=self.clock,
        )
        self.setup_tree(node)
        node.tick_once()
        self.clock.advance(1.0)
        self.set_status("failed:plan")
        node.tick_once()  # attempt 1 fails, Retry re-enters
        node.tick_once()  # attempt 2 publishes again
        self.assertEqual(len(self.node.publisher.published), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
