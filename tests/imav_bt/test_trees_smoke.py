#!/usr/bin/env python3
"""
test_trees_smoke.py — the assembled tree ticks, and its control flow is correct.

These tests run the REAL tree structure with the placeholder leaves in place.
That is the point of the current phase: the control flow -- ordering, priority,
what happens when a mission fails -- is the part that is expensive to change
later, so it is tested now, while the leaf bodies are still stubs.

No ROS graph and no rclpy node: every placeholder leaf declares
``requires_node = False``, and `test_setup_needs_no_ros_node` guards that
property so the suite stays runnable anywhere.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "imav_bt"))

import py_trees  # noqa: E402
from py_trees.common import Status  # noqa: E402

from imav_bt import blackboard as bb  # noqa: E402
from imav_bt.trees import (  # noqa: E402
    flight_a,
    flight_b,
    landing,
    mission1_obstacle,
    mission2_darkroom,
    mission3_hotspot,
    mission4_turbine,
)
from imav_bt.trees.root import build_root  # noqa: E402

#: Safety needs odometry, which nothing publishes under unit test. Turning the
#: requirement off keeps these tests focused on mission control flow; the
#: safety branch has its own tests below.
NO_ODOM_REQUIRED = {"safety": {"require_odom": False}}

TICK_LIMIT = 4000


def run_until_done(root, limit=TICK_LIMIT):
    """Tick until the root reports SUCCESS or FAILURE, then stop."""
    for count in range(1, limit + 1):
        root.tick_once()
        if root.status in (Status.SUCCESS, Status.FAILURE):
            return root.status, count
    raise AssertionError(f"tree did not finish within {limit} ticks")


def arm(plan):
    """Put the tree into a flying state with the given plan selected."""
    writer = bb.client("test", write=["running", "flight_plan"])
    writer.running = True
    writer.flight_plan = plan


class TestSubtreesBuild(unittest.TestCase):
    """Every builder shares the build(config, arena) signature and ticks clean."""

    def setUp(self):
        bb.reset()

    def test_every_mission_builder_ticks_to_completion(self):
        for module in (
            mission1_obstacle,
            mission2_darkroom,
            mission3_hotspot,
            mission4_turbine,
            landing,
        ):
            with self.subTest(module=module.__name__):
                bb.reset()
                status, _ = run_until_done(module.build(None, None))
                self.assertEqual(status, Status.SUCCESS)

    def test_builders_accept_none_config(self):
        for module in (mission1_obstacle, mission2_darkroom, landing):
            with self.subTest(module=module.__name__):
                self.assertIsNotNone(module.build(None, None))


class TestRoot(unittest.TestCase):
    def setUp(self):
        bb.reset()
        self.root = build_root(NO_ODOM_REQUIRED, {})

    def test_tree_renders(self):
        """If this raises, the tree is malformed in a way ticking may not show."""
        text = py_trees.display.unicode_tree(self.root)
        self.assertIn("Root", text)
        self.assertIn("SafetyOverride", text)
        self.assertIn("FlightPlanRunner", text)

    def test_setup_needs_no_ros_node(self):
        """Guards the property that keeps this suite ROS-free."""
        py_trees.trees.setup(root=self.root)

    def test_idles_until_the_operator_arms_it(self):
        for _ in range(20):
            self.root.tick_once()
            self.assertEqual(self.root.status, Status.RUNNING)
        self.assertEqual(bb.snapshot()["mission_outcomes"], {})

    def test_selected_plan_does_not_run_while_not_running(self):
        writer = bb.client("test", write=["flight_plan"])
        writer.flight_plan = bb.FlightPlan.FLIGHT_A
        for _ in range(20):
            self.root.tick_once()
        self.assertEqual(bb.snapshot()["mission_outcomes"], {})


class TestFlightA(unittest.TestCase):
    """The chained flight -- where the +12 bonus points live."""

    def setUp(self):
        bb.reset()

    def test_flies_all_three_missions_and_lands(self):
        arm(bb.FlightPlan.FLIGHT_A)
        status, _ = run_until_done(build_root(NO_ODOM_REQUIRED, {}))
        self.assertEqual(status, Status.SUCCESS)

        outcomes = bb.snapshot()["mission_outcomes"]
        self.assertEqual(
            outcomes,
            {
                "mission1": bb.Outcome.SUCCESS,
                "mission2": bb.Outcome.SUCCESS,
                "mission3": bb.Outcome.SUCCESS,
            },
        )
        self.assertTrue(bb.snapshot()["objectives"].get("landing.moving"))

    def test_a_completed_flight_does_not_take_off_again(self):
        """*** Regression: an unattended second takeoff. ***

        The tree ticks forever and py_trees restarts a subtree whose previous
        status was not RUNNING. Without StopRunning, FlightA lands, returns
        SUCCESS, and the root re-enters it from the top on the very next tick --
        straight back into ArmAndTakeoff, seconds after touchdown.
        """
        arm(bb.FlightPlan.FLIGHT_A)
        root = build_root(NO_ODOM_REQUIRED, {})
        run_until_done(root)

        self.assertFalse(
            bb.snapshot()["running"], "flight should disarm itself when complete"
        )

        # Keep ticking: the tree must sit in Idle, not start a second flight.
        for _ in range(200):
            root.tick_once()
            self.assertEqual(root.status, Status.RUNNING)
        self.assertFalse(bb.snapshot()["running"])

    def test_operator_can_re_arm_for_another_attempt(self):
        """The 30-minute slot allows several attempts, so disarming must not be
        a one-way door."""
        arm(bb.FlightPlan.FLIGHT_A)
        root = build_root(NO_ODOM_REQUIRED, {})
        run_until_done(root)
        self.assertFalse(bb.snapshot()["running"])

        arm(bb.FlightPlan.FLIGHT_A)  # operator calls set_running again
        status, _ = run_until_done(root)
        self.assertEqual(status, Status.SUCCESS)

    def test_a_failed_mission_still_reaches_the_landing(self):
        """*** The property the whole chaining strategy depends on. ***

        Mission 2 fails outright. Missions 1 and 3 must still complete and the
        drone must still land, because the landing and multi-mission bonuses
        are awarded per-mission and would otherwise be lost for missions 1
        and 3 as well.
        """
        config = dict(NO_ODOM_REQUIRED)
        config["missions"] = {
            "mission2": {"steps": {"ApproachRoom": {"outcome": "failure"}}}
        }
        arm(bb.FlightPlan.FLIGHT_A)
        status, _ = run_until_done(build_root(config, {}))

        self.assertEqual(status, Status.SUCCESS)
        outcomes = bb.snapshot()["mission_outcomes"]
        self.assertEqual(outcomes["mission1"], bb.Outcome.SUCCESS)
        self.assertEqual(outcomes["mission2"], bb.Outcome.SKIPPED)
        self.assertEqual(outcomes["mission3"], bb.Outcome.SUCCESS)
        self.assertTrue(
            bb.snapshot()["objectives"].get("landing.moving"),
            "the flight must still land after a mission failure",
        )

    def test_landing_degrades_to_the_static_platform(self):
        """Moving-platform landing is worth 2 pts per mission, static 1 -- but a
        failed moving approach must still put the drone down."""
        config = dict(NO_ODOM_REQUIRED)
        config["landing"] = {"steps": {"FindMovingMarker": {"outcome": "failure"}}}
        arm(bb.FlightPlan.FLIGHT_A)
        run_until_done(build_root(config, {}))

        objectives = bb.snapshot()["objectives"]
        self.assertNotIn("landing.moving", objectives)
        self.assertTrue(objectives.get("landing.static"))

    def test_landing_always_puts_the_drone_down(self):
        """Both marker approaches fail; the plain-descent rung must still run."""
        config = dict(NO_ODOM_REQUIRED)
        config["landing"] = {
            "steps": {
                "FindMovingMarker": {"outcome": "failure"},
                "FindStaticMarker": {"outcome": "failure"},
            }
        }
        arm(bb.FlightPlan.FLIGHT_A)
        status, _ = run_until_done(build_root(config, {}))
        self.assertEqual(status, Status.SUCCESS)

    def test_a_disabled_mission_is_skipped_without_failing(self):
        config = dict(NO_ODOM_REQUIRED)
        config["missions"] = {"mission3": {"enabled": False}}
        arm(bb.FlightPlan.FLIGHT_A)
        status, _ = run_until_done(build_root(config, {}))
        self.assertEqual(status, Status.SUCCESS)
        self.assertNotIn("mission3", bb.snapshot()["mission_outcomes"])

    def test_mission1_gate_degrades_from_red_to_blue(self):
        """A red window is worth 1.0 pt and a blue one 0.5, so red is attempted
        first -- but failing to find red must not lose the gate entirely."""
        cfg = {"steps": {"DetectWindowARed": {"outcome": "failure"}}}
        status, _ = run_until_done(mission1_obstacle.build(cfg, None))
        self.assertEqual(status, Status.SUCCESS)
        objectives = bb.snapshot()["objectives"]
        self.assertNotIn("m1.window_a_red", objectives)
        self.assertTrue(objectives.get("m1.window_a_blue"))

    def test_mission1_survives_a_gate_it_cannot_cross(self):
        """Mission 1's objectives are independent, so one unusable gate costs
        only that gate's points -- it must not fail the mission."""
        cfg = {
            "steps": {
                "DetectWindowARed": {"outcome": "failure"},
                "DetectWindowABlue": {"outcome": "failure"},
            }
        }
        status, _ = run_until_done(mission1_obstacle.build(cfg, None))
        self.assertEqual(status, Status.SUCCESS)
        self.assertTrue(bb.snapshot()["objectives"].get("m1.tubes"))


class TestFlightB(unittest.TestCase):
    def setUp(self):
        bb.reset()

    def test_runs_mission4_and_lands_at_the_base(self):
        arm(bb.FlightPlan.FLIGHT_B)
        status, _ = run_until_done(build_root(NO_ODOM_REQUIRED, {}))
        self.assertEqual(status, Status.SUCCESS)
        self.assertEqual(
            bb.snapshot()["mission_outcomes"], {"mission4": bb.Outcome.SUCCESS}
        )

    def test_failed_grab_still_lands_near_the_base(self):
        """Not about bonuses here -- an airborne drone with a failed mission
        still has to come down, and the wire tether dictates where."""
        config = dict(NO_ODOM_REQUIRED)
        config["missions"] = {
            "mission4": {"steps": {"GrabRing": {"outcome": "failure"}}}
        }
        arm(bb.FlightPlan.FLIGHT_B)
        status, _ = run_until_done(build_root(config, {}))
        self.assertEqual(status, Status.SUCCESS)
        self.assertEqual(
            bb.snapshot()["mission_outcomes"], {"mission4": bb.Outcome.SKIPPED}
        )

    def test_flight_b_does_not_use_the_landing_platform(self):
        """Rulebook 4.4.7: land near the turbine base so the 4 m wire survives.
        Mission 4 earns no landing bonus, so flying to the pad is pure risk."""
        arm(bb.FlightPlan.FLIGHT_B)
        run_until_done(build_root(NO_ODOM_REQUIRED, {}))
        objectives = bb.snapshot()["objectives"]
        self.assertNotIn("landing.moving", objectives)
        self.assertNotIn("landing.static", objectives)


class TestSafetyPreemption(unittest.TestCase):
    """The root is a reactive Selector so safety can interrupt a running mission."""

    def setUp(self):
        bb.reset()

    def test_abort_preempts_a_running_mission(self):
        # Park mission 2 on a step that never finishes, so the flight is
        # genuinely mid-mission when the abort lands. (Success-only stubs let a
        # Sequence run all its children in a single tick, so without this the
        # flight would already be over.)
        config = dict(NO_ODOM_REQUIRED)
        config["missions"] = {
            "mission2": {"steps": {"ApproachRoom": {"outcome": "running"}}}
        }
        root = build_root(config, {})
        arm(bb.FlightPlan.FLIGHT_A)
        for _ in range(5):
            root.tick_once()
        self.assertTrue(bb.snapshot()["running"], "flight should still be in progress")

        bb.client("test", write=["abort_requested"]).abort_requested = True
        root.tick_once()

        self.assertEqual(bb.snapshot()["safety_reason"], "operator abort requested")
        self.assertIn(
            "EmergencyLand",
            py_trees.display.unicode_tree(root, show_status=True),
        )

    def test_missing_odometry_is_an_emergency_once_armed(self):
        """Absence of pose IS the emergency -- unlike a missing battery sensor."""
        root = build_root({"safety": {"require_odom": True}}, {})
        arm(bb.FlightPlan.FLIGHT_A)
        root.tick_once()
        self.assertEqual(bb.snapshot()["safety_reason"], "no odometry received")

    def test_missing_battery_sensor_does_not_ground_the_drone(self):
        """A drone with no battery topic must still fly."""
        root = build_root(NO_ODOM_REQUIRED, {})
        arm(bb.FlightPlan.FLIGHT_A)
        root.tick_once()
        self.assertEqual(bb.snapshot()["safety_reason"], "")

    def test_low_battery_trips_once_a_reading_exists(self):
        root = build_root(
            {"safety": {"require_odom": False, "min_battery": 0.25}}, {}
        )
        writer = bb.client("test", write=["battery_valid", "battery_fraction"])
        writer.battery_valid = True
        writer.battery_fraction = 0.18
        arm(bb.FlightPlan.FLIGHT_A)
        root.tick_once()
        self.assertIn("battery 0.18", bb.snapshot()["safety_reason"])

    def test_geofence_breach_trips(self):
        root = build_root(
            {
                "safety": {
                    "require_odom": False,
                    "geofence": [0.0, 14.0, 0.0, 7.0, 0.0, 4.0],
                }
            },
            {},
        )
        writer = bb.client("test", write=["odom_valid", "position"])
        writer.odom_valid = True
        writer.position = (20.0, 3.0, 1.5)  # outside the 14 m arena length
        arm(bb.FlightPlan.FLIGHT_A)
        root.tick_once()
        self.assertIn("outside geofence", bb.snapshot()["safety_reason"])

    def test_safety_is_quiet_while_idle(self):
        """On the ground with no odometry, an unarmed tree must not scream."""
        root = build_root({"safety": {"require_odom": True}}, {})
        for _ in range(10):
            root.tick_once()
        self.assertEqual(bb.snapshot()["safety_reason"], "")


if __name__ == "__main__":
    unittest.main(verbosity=2)
