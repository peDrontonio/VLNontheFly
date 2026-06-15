import importlib.util
import json
import math
import sys
import unittest
from pathlib import Path

from px4_msgs.msg import VehicleLocalPosition


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "vlm_primitive_gate.py"
SPEC = importlib.util.spec_from_file_location("vlm_primitive_gate", SCRIPT_PATH)
gate = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = gate
SPEC.loader.exec_module(gate)


def result_with_text(text):
    return json.dumps({"text": text})


class TestVlmPrimitiveGate(unittest.TestCase):
    def test_parse_strict_nested_text_json(self):
        proposal = gate.parse_vlm_result(
            result_with_text(
                '{"primitive":"FORWARD","distance_m":0.5,"confidence":0.8,"reason":"clear"}'
            )
        )

        self.assertEqual(proposal.primitive, "FORWARD")
        self.assertEqual(proposal.distance_m, 0.5)
        self.assertEqual(proposal.confidence, 0.8)

    def test_reject_natural_language_text(self):
        with self.assertRaises((ValueError, json.JSONDecodeError)):
            gate.parse_vlm_result(result_with_text("Move forward because the path is clear."))

    def test_accepts_safe_truncated_hold_only(self):
        proposal = gate.parse_vlm_result(
            result_with_text('{"primitive":"HOLD","distance_m":0.0')
        )

        self.assertEqual(proposal.primitive, "HOLD")
        self.assertEqual(proposal.distance_m, 0.0)
        self.assertEqual(proposal.confidence, 1.0)

    def test_rejects_truncated_movement(self):
        with self.assertRaises(json.JSONDecodeError):
            gate.parse_vlm_result(result_with_text('{"primitive":"FORWARD","distance_m":0.5'))

    def test_validate_rejects_low_confidence(self):
        proposal = gate.PrimitiveProposal("FORWARD", 0.5, 0.2, "uncertain")
        decision = gate.validate_proposal(proposal, 0.65, 1.0, [0.5, 1.0])

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "confidence below threshold")

    def test_validate_rejects_unlisted_distance(self):
        proposal = gate.PrimitiveProposal("FORWARD", 0.75, 0.9, "clear")
        decision = gate.validate_proposal(proposal, 0.65, 1.0, [0.5, 1.0])

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "translation distance is not allowed")

    def test_primitive_to_body_offset_is_flu_without_yaw(self):
        # Offsets are in base_link FLU and never depend on heading.
        forward = gate.primitive_to_body_offset(gate.PrimitiveProposal("FORWARD", 0.5, 0.9, ""))
        left = gate.primitive_to_body_offset(gate.PrimitiveProposal("LEFT", 0.5, 0.9, ""))
        up = gate.primitive_to_body_offset(gate.PrimitiveProposal("UP", 0.5, 0.9, ""))

        self.assertEqual(forward, (0.5, 0.0, 0.0))
        self.assertEqual(left, (0.0, 0.5, 0.0))
        self.assertEqual(up, (0.0, 0.0, 0.5))

    def test_primitive_to_body_offset_negative_directions(self):
        back = gate.primitive_to_body_offset(gate.PrimitiveProposal("BACK", 1.0, 0.9, ""))
        right = gate.primitive_to_body_offset(gate.PrimitiveProposal("RIGHT", 1.0, 0.9, ""))
        down = gate.primitive_to_body_offset(gate.PrimitiveProposal("DOWN", 1.0, 0.9, ""))

        self.assertEqual(back, (-1.0, 0.0, 0.0))
        self.assertEqual(right, (0.0, -1.0, 0.0))
        self.assertEqual(down, (0.0, 0.0, -1.0))

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
