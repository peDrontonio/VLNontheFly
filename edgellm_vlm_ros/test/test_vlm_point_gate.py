import importlib.util
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "vlm_point_gate.py"
SPEC = importlib.util.spec_from_file_location("vlm_point_gate", SCRIPT_PATH)
gate = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = gate
SPEC.loader.exec_module(gate)


def result_with_text(text, width=640, height=480):
    return json.dumps({"text": text, "image_width": width, "image_height": height})


def camera_info(width=4, height=3, fx=100.0, fy=100.0, cx=2.0, cy=1.0):
    info = CameraInfo()
    info.width = width
    info.height = height
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return info


def depth_image_16uc1(values):
    array = np.asarray(values, dtype=np.uint16)
    msg = Image()
    msg.height = array.shape[0]
    msg.width = array.shape[1]
    msg.encoding = "16UC1"
    msg.is_bigendian = False
    msg.step = array.shape[1] * array.dtype.itemsize
    msg.data = array.tobytes()
    return msg


class TestVlmPointGate(unittest.TestCase):
    def test_parse_strict_nested_text_json(self):
        proposal = gate.parse_vlm_result(result_with_text('{"u":320,"v":240,"confidence":0.8}'))

        self.assertEqual(proposal.u, 320.0)
        self.assertEqual(proposal.v, 240.0)
        self.assertEqual(proposal.confidence, 0.8)
        self.assertEqual(proposal.image_width, 640)
        self.assertEqual(proposal.image_height, 480)

    def test_reject_natural_language_text(self):
        with self.assertRaises(json.JSONDecodeError):
            gate.parse_vlm_result(result_with_text("Move toward the open area."))

    def test_validate_rejects_low_confidence(self):
        proposal = gate.PointProposal(320.0, 240.0, 0.2, "", 640, 480)
        decision = gate.validate_proposal(proposal, 0.6, 640, 480)

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "confidence below threshold")

    def test_validate_rejects_pixel_outside_image(self):
        proposal = gate.PointProposal(700.0, 240.0, 0.9, "", 640, 480)
        decision = gate.validate_proposal(proposal, 0.6, 640, 480)

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "pixel outside source image")

    def test_depth_image_to_meters_and_median_sample(self):
        msg = depth_image_16uc1([
            [0, 1000, 0],
            [1100, 1200, 1300],
            [0, 1400, 0],
        ])
        depth_m = gate.depth_image_to_meters(msg, 0.001)

        sampled = gate.sample_depth_m(depth_m, 1.0, 1.0, 1, 0.25, 4.0)

        self.assertTrue(math.isclose(sampled, 1.2, abs_tol=1e-6))

    def test_scale_and_deproject_center_pixel(self):
        u_depth, v_depth = gate.scale_pixel(320.0, 240.0, 640, 480, 4, 3)
        point = gate.deproject_pixel_to_optical(
            u_depth,
            v_depth,
            2.0,
            camera_info(width=4, height=3, fx=100.0, fy=100.0, cx=2.0, cy=1.5),
        )

        self.assertTrue(math.isclose(point[0], 0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(point[1], 0.0, abs_tol=1e-6))
        self.assertEqual(point[2], 2.0)

    def test_optical_to_body_horizontal_ignores_vertical_for_z_plane(self):
        dx_body, dy_body = gate.optical_to_body_horizontal((0.25, -0.75, 2.0))

        self.assertEqual(dx_body, 2.0)
        self.assertEqual(dy_body, -0.25)

    def test_clamp_xy_distance(self):
        dx, dy = gate.clamp_xy_distance(3.0, 4.0, 2.0)

        self.assertTrue(math.isclose(math.hypot(dx, dy), 2.0, abs_tol=1e-6))

    def test_resolve_goal_z_keeps_current_or_fixed_plane(self):
        self.assertEqual(gate.resolve_goal_z("current_pose", 1.0, 0.7), 0.7)
        self.assertEqual(gate.resolve_goal_z("fixed", 1.0, 0.7), 1.0)

    def test_px4_local_position_to_pose_converts_ned_to_enu(self):
        msg = VehicleLocalPosition()
        msg.x = 2.0
        msg.y = 3.0
        msg.z = -1.5
        msg.heading = 0.0

        pose = gate.px4_local_position_to_pose(msg)
        yaw = gate.quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        self.assertEqual(pose.position.x, 3.0)
        self.assertEqual(pose.position.y, 2.0)
        self.assertEqual(pose.position.z, 1.5)
        self.assertTrue(math.isclose(yaw, math.pi / 2.0, abs_tol=1e-6))


if __name__ == "__main__":
    unittest.main()
