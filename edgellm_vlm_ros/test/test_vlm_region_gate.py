import importlib.util
import json
import sys
import unittest
from pathlib import Path

import numpy as np
from geometry_msgs.msg import TransformStamped


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# vlm_region_gate imports vlm_point_gate; register the dependency first.
_load("vlm_point_gate")
gate = _load("vlm_region_gate")


def result_with_text(text):
    return json.dumps({"text": text, "image_width": 640, "image_height": 480})


class RegionTableTest(unittest.TestCase):
    def test_3x3_names_and_indices(self):
        table = gate.build_region_table(3, 3)
        self.assertEqual(len(table), 9)
        self.assertEqual(table["TOP-LEFT"], (0, 0))
        self.assertEqual(table["CENTER"], (1, 1))
        self.assertEqual(table["BOTTOM-RIGHT"], (2, 2))
        self.assertIn("MIDDLE-LEFT", table)
        self.assertIn("TOP-CENTER", table)


class ParseRegionTest(unittest.TestCase):
    def setUp(self):
        self.known = list(gate.build_region_table(3, 3).keys())

    def test_clean_json(self):
        proposal = gate.parse_region_result(
            result_with_text('{"region":"CENTER","confidence":0.8}'), self.known)
        self.assertEqual(proposal.region, "CENTER")
        self.assertEqual(proposal.confidence, 0.8)

    def test_truncated_json_recovers_region(self):
        # Closing brace/confidence lost, but the region name is present.
        proposal = gate.parse_region_result(
            result_with_text('{"region":"TOP-LEFT","confidence":0.'), self.known)
        self.assertEqual(proposal.region, "TOP-LEFT")

    def test_longest_match_wins(self):
        # "TOP-CENTER" must not be mistaken for bare "CENTER".
        proposal = gate.parse_region_result(
            result_with_text('garbage TOP-CENTER garbage'), self.known)
        self.assertEqual(proposal.region, "TOP-CENTER")

    def test_normalizes_case_and_separators(self):
        proposal = gate.parse_region_result(
            result_with_text('{"region":"bottom_right","confidence":0.6}'), self.known)
        self.assertEqual(proposal.region, "BOTTOM-RIGHT")

    def test_unknown_region_raises(self):
        with self.assertRaises(ValueError):
            gate.parse_region_result(result_with_text('{"region":"NORTH"}'), self.known)

    def test_none_sentinel_for_missing_target(self):
        proposal = gate.parse_region_result(
            result_with_text('{"region":"NONE","confidence":0.0}'), self.known)
        self.assertEqual(proposal.region, "NONE")
        self.assertEqual(proposal.confidence, 0.0)

    def test_preserves_rgb_metadata_for_timestamped_tf(self):
        payload = json.dumps({
            "text": '{"region":"CENTER","confidence":0.9}',
            # Exact integer fields must win over a low-precision legacy stamp.
            "stamp": 123.0,
            "stamp_sec": 123,
            "stamp_nanosec": 250_000_000,
            "frame_id": "camera_color_optical_frame",
            "image_width": 848,
            "image_height": 480,
        })
        proposal = gate.parse_region_result(payload, self.known)
        self.assertEqual(proposal.source_stamp_s, 123.25)
        self.assertEqual(proposal.source_frame_id, "camera_color_optical_frame")
        self.assertEqual(proposal.image_width, 848)
        self.assertEqual(proposal.image_height, 480)


class GoalArbitrationTest(unittest.TestCase):
    @staticmethod
    def accepted(region):
        return gate.CellDecision(True, "test", 1, 1, region, 2.0)

    def test_requires_consecutive_same_region(self):
        filt = gate.ConsecutiveRegionFilter(required=3)
        self.assertFalse(filt.observe(self.accepted("CENTER")))
        self.assertFalse(filt.observe(self.accepted("CENTER")))
        self.assertTrue(filt.observe(self.accepted("CENTER")))

    def test_region_change_restarts_consensus(self):
        filt = gate.ConsecutiveRegionFilter(required=2)
        self.assertFalse(filt.observe(self.accepted("CENTER")))
        self.assertFalse(filt.observe(self.accepted("TOP-RIGHT")))
        self.assertEqual(filt.region, "TOP-RIGHT")
        self.assertEqual(filt.count, 1)

    def test_rejected_decision_clears_consensus(self):
        filt = gate.ConsecutiveRegionFilter(required=2)
        filt.observe(self.accepted("CENTER"))
        self.assertFalse(filt.observe(gate.CellDecision(False, "blocked")))
        self.assertIsNone(filt.region)
        self.assertEqual(filt.count, 0)

    def test_terminal_planner_statuses_release_goal(self):
        self.assertTrue(gate.planner_status_releases_goal("reached"))
        self.assertTrue(gate.planner_status_releases_goal("rejected:zone"))
        self.assertTrue(gate.planner_status_releases_goal("failed:plan"))
        self.assertFalse(gate.planner_status_releases_goal("accepted"))
        self.assertFalse(gate.planner_status_releases_goal("accepted:safety_bubble"))

    def test_pose_distance_supports_close_goal_suppression(self):
        a = gate.PoseStamped()
        b = gate.PoseStamped()
        b.pose.position.x = 0.3
        b.pose.position.y = 0.4
        self.assertAlmostEqual(gate.pose_distance(a, b), 0.5)

    def test_proposal_serializes_consensus_state(self):
        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, msg):
                self.messages.append(msg)

        node = object.__new__(gate.VlmRegionGate)
        node.consensus = gate.ConsecutiveRegionFilter(required=3, region="CENTER", count=2)
        node.active_region = None
        node.active_goal = None
        node.proposal_pub = FakePublisher()

        node._publish_proposal(self.accepted("CENTER"))

        payload = json.loads(node.proposal_pub.messages[0].data)
        self.assertEqual(payload["consistency_count"], 2)
        self.assertEqual(payload["consistency_required"], 3)


class TimestampedMapGoalTest(unittest.TestCase):
    class FakeBuffer:
        def __init__(self):
            self.calls = []

        def lookup_transform(self, target, source, stamp, timeout):
            del timeout
            self.calls.append((target, source, stamp.nanoseconds))
            tf = TransformStamped()
            tf.header.frame_id = target
            tf.child_frame_id = source
            tf.transform.rotation.w = 1.0
            if target == "base_link":
                tf.transform.translation.x = 0.2
            else:
                tf.transform.translation.x = 10.0
                tf.transform.translation.z = 2.0
            return tf

    def test_uses_rgb_timestamp_for_camera_and_map_tf(self):
        node = object.__new__(gate.VlmRegionGate)
        node.tf_buffer = self.FakeBuffer()
        node.body_frame_id = "base_link"
        node.map_frame_id = "map"
        node.tf_timeout_s = 0.1
        node.selection_mode = "open_space"
        node.max_goal_distance_m = 1.5
        node.goal_z_mode = "current_pose"
        node.fixed_goal_z_m = 1.0
        node.depth_scale_m = 0.001
        node.min_depth_m = 0.25
        node.max_depth_m = 4.0
        node.grid_cols = 1
        node.grid_rows = 1
        node._now = lambda: gate.Time(nanoseconds=200_000_000_000)

        depth = gate.Image()
        depth.header.frame_id = "camera_color_optical_frame"
        depth.width = 3
        depth.height = 3
        depth.encoding = "32FC1"
        depth.step = 12
        depth.data = np.full((3, 3), 2.0, dtype=np.float32).tobytes()
        info = gate.CameraInfo()
        info.header.frame_id = "camera_color_optical_frame"
        info.k = [1.0, 0.0, 1.5, 0.0, 1.0, 1.5, 0.0, 0.0, 1.0]
        proposal = gate.RegionProposal(
            "CENTER", 0.9, source_stamp_s=123.25,
            source_frame_id="camera_color_optical_frame")
        decision = gate.CellDecision(True, "test", 0, 0, "CENTER", 2.0)

        goal, sampled_depth = node._build_map_goal(decision, depth, info, proposal)

        self.assertAlmostEqual(sampled_depth, 2.0)
        self.assertAlmostEqual(goal.pose.position.x, 10.2)
        self.assertAlmostEqual(goal.pose.position.z, 2.0)
        expected_stamp = 123_250_000_000
        self.assertEqual(node.tf_buffer.calls, [
            ("base_link", "camera_color_optical_frame", expected_stamp),
            ("map", "base_link", expected_stamp),
        ])


class StandoffTest(unittest.TestCase):
    def test_shortens_by_standoff(self):
        dx, dy = gate.apply_standoff(3.0, 0.0, 0.8)
        self.assertAlmostEqual(dx, 2.2)
        self.assertAlmostEqual(dy, 0.0)

    def test_zero_when_within_standoff(self):
        self.assertEqual(gate.apply_standoff(0.5, 0.0, 0.8), (0.0, 0.0))

    def test_noop_when_standoff_zero(self):
        self.assertEqual(gate.apply_standoff(3.0, 1.0, 0.0), (3.0, 1.0))


class GridGeometryTest(unittest.TestCase):
    def test_cell_bounds_and_center(self):
        self.assertEqual(gate.cell_pixel_bounds(0, 0, 3, 3, 600, 300), (0, 0, 200, 100))
        self.assertEqual(gate.cell_pixel_bounds(2, 2, 3, 3, 600, 300), (400, 200, 600, 300))
        self.assertEqual(gate.cell_center_pixel(1, 1, 3, 3, 600, 300), (300.0, 150.0))


class DepthScanTest(unittest.TestCase):
    def test_cell_median_depth_ignores_invalid(self):
        depth = np.full((30, 30), np.nan, dtype=np.float32)
        depth[0:10, 0:10] = 2.0
        depth[0, 0] = 0.0  # below min, must be ignored
        median = gate.cell_median_depth(depth, (0, 0, 10, 10), 0.25, 4.0)
        self.assertAlmostEqual(median, 2.0)

    def test_cell_median_depth_none_when_empty(self):
        depth = np.full((30, 30), np.nan, dtype=np.float32)
        self.assertIsNone(gate.cell_median_depth(depth, (0, 0, 10, 10), 0.25, 4.0))

    def test_most_open_cell_picks_max_clearance(self):
        depth = np.full((30, 30), np.nan, dtype=np.float32)
        depth[0:10, 0:10] = 1.0     # cell (0,0) near
        depth[0:10, 20:30] = 3.5    # cell (2,0) far -> most open
        depth[20:30, 0:10] = 0.5    # cell (0,2) below clearance
        depths = gate.scan_cells(depth, 3, 3, 0.25, 4.0)
        self.assertEqual(gate.most_open_cell(depths, 0.8), (2, 0))

    def test_most_open_cell_none_when_all_blocked(self):
        depth = np.full((30, 30), 0.3, dtype=np.float32)  # all below 0.8 clearance
        depths = gate.scan_cells(depth, 3, 3, 0.25, 4.0)
        self.assertIsNone(gate.most_open_cell(depths, 0.8))


if __name__ == "__main__":
    unittest.main()
