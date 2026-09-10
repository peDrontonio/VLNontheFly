#!/usr/bin/env python3
"""
test_blackboard.py — schema, defaults and access-control checks.

Runs as a plain unittest script with no ROS graph, matching the house test
style in edgellm_vlm_ros/test/.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import py_trees  # noqa: E402

from imav_bt import blackboard as bb  # noqa: E402


class TestBlackboard(unittest.TestCase):
    def setUp(self):
        bb.reset()

    def test_reset_populates_every_key_with_its_default(self):
        snap = bb.snapshot()
        self.assertEqual(set(snap), set(bb.key_names()))
        self.assertEqual(snap["flight_plan"], bb.FlightPlan.IDLE)
        self.assertFalse(snap["running"])
        self.assertFalse(snap["odom_valid"])
        self.assertEqual(snap["mission_outcomes"], {})

    def test_defaults_satisfy_the_schema(self):
        self.assertEqual(bb.validate(), [])

    def test_validate_catches_a_mistyped_value(self):
        writer = bb.client("t", write=["battery_fraction"])
        writer.battery_fraction = "not a float"
        problems = bb.validate()
        self.assertTrue(any("battery_fraction" in p for p in problems), problems)

    def test_optional_key_may_be_none_but_required_key_may_not(self):
        writer = bb.client("t", write=["active_goal", "safety_reason"])
        writer.active_goal = None  # declared optional
        self.assertEqual(bb.validate(), [])
        writer.safety_reason = None  # NOT optional
        self.assertTrue(any("safety_reason" in p for p in bb.validate()))

    def test_mutable_defaults_are_not_shared_between_resets(self):
        """A dict default must be a fresh object each reset, not one shared instance."""
        writer = bb.client("t", write=["mission_outcomes"])
        writer.mission_outcomes = {"mission1": bb.Outcome.SUCCESS}
        bb.reset()
        self.assertEqual(bb.snapshot()["mission_outcomes"], {})

    def test_unknown_key_is_rejected_at_client_construction(self):
        with self.assertRaises(KeyError) as ctx:
            bb.client("t", read=["postion"])  # deliberate typo
        self.assertIn("not a declared imav_bt blackboard key", str(ctx.exception))

    def test_read_access_is_enforced(self):
        reader = bb.client("t", read=["running"])
        with self.assertRaises(AttributeError):
            reader.running = True

    def test_keys_are_namespaced(self):
        writer = bb.client("t", write=["running"])
        writer.running = True
        self.assertIn(
            f"{bb.NAMESPACE}/running", py_trees.blackboard.Blackboard.keys()
        )

    def test_every_key_declares_a_known_writer_class(self):
        for key in bb.SCHEMA:
            self.assertIn(key.writer, ("SENSED", "COMMANDED", "DERIVED"), key.name)

    def test_writer_partition_covers_the_whole_schema(self):
        total = sum(
            len(bb.keys_by_writer(w)) for w in ("SENSED", "COMMANDED", "DERIVED")
        )
        self.assertEqual(total, len(bb.SCHEMA))


if __name__ == "__main__":
    unittest.main(verbosity=2)
