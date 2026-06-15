import importlib.util
import json
import sys
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "vlm_nav_supervisor.py"
SPEC = importlib.util.spec_from_file_location("vlm_nav_supervisor", SCRIPT_PATH)
supervisor = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = supervisor
SPEC.loader.exec_module(supervisor)


class TestVlmNavSupervisor(unittest.TestCase):
    def test_parse_accepted_gate_proposal(self):
        parsed = supervisor.parse_gate_proposal(json.dumps({
            "accepted": True,
            "gate_reason": "accepted",
            "u": 320,
            "v": 240,
            "confidence": 0.9,
        }))

        self.assertTrue(parsed.accepted)
        self.assertEqual(parsed.reason, "accepted")
        self.assertEqual(parsed.payload["u"], 320)

    def test_parse_rejected_gate_proposal(self):
        parsed = supervisor.parse_gate_proposal(json.dumps({
            "accepted": False,
            "gate_reason": "confidence below threshold",
        }))

        self.assertFalse(parsed.accepted)
        self.assertEqual(parsed.reason, "confidence below threshold")

    def test_rejects_non_object_proposal(self):
        with self.assertRaises(ValueError):
            supervisor.parse_gate_proposal(json.dumps(["bad"]))


if __name__ == "__main__":
    unittest.main()
