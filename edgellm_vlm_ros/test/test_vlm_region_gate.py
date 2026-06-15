import importlib.util
import json
import sys
import unittest
from pathlib import Path

import numpy as np


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
