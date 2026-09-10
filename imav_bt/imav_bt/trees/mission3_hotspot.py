"""
mission3_hotspot.py — Mission 3, drop a cone on the hot spot. PLACEHOLDER.

Rulebook 4.4.5 / Table 4. Three identical boxes sit side by side; one contains a
heat source at 70-100 degC. Identify it and drop a 7 g PLA cone into it.

    S3 = (Dr·A + Hb·A) · W3 + La3 + B3 + SF3

    Dr  4 pts cone dropped in the hot box, 2 pts in any other box
    Hb  3 pts hot box detected
    -> 4 + 3 = 7 raw, the mission cap.

*** Ordering is worth 3 points: blink BEFORE you drop ***
---------------------------------------------------------
Rulebook 4.4.6 defines how Hb is judged, and it is unusually specific:

    "the drone must be equipped with a red LED that flashes visibly when the
     drone is directly above the relevant box (before or during the release of
     the cone). Hb points will be awarded based SOLELY on this criterion."

So Hb is earned by the LED blink, not by the drop. Two consequences the tree
must encode:

  1. The blink step comes BEFORE the drop in the sequence. If the drop is
     attempted first and something goes wrong -- the drone drifts off the box,
     the servo jams, the battery guard trips -- the 3 detection points are lost
     along with it, even though the detection itself succeeded.

  2. The blink must not be conditional on the dropper being healthy. A drone
     with a failed dropper can still score 3 of the 7 raw points, which after
     the mass factor is worth roughly 6 points. So the blink is a plain
     sequence step, never nested under the drop.

Note also that dropping into the WRONG box still scores 2 of 4. So a low-
confidence thermal reading is not a reason to abort the drop -- guessing beats
not dropping. That belongs in the real ScanBoxes/SelectHotBox behaviours; the
tree just needs to not stand in the way.

STATUS: placeholder. No thermal sensor code exists in the repo yet, and the
payload services are mocked by scripts/payload_node.py.
See docs/ADDING_A_MISSION.md.
"""

from __future__ import annotations

from typing import Optional

import py_trees

from imav_bt.behaviors.bookkeeping import MarkObjective
from imav_bt.behaviors.stubs import step_factory

MISSION = "mission3"


def build(config: Optional[dict] = None, arena: Optional[dict] = None):
    """Build the Mission 3 subtree.

    Args:
        config: the ``missions.mission3`` section of config/missions.yaml.
        arena: anchor table from config/arena.yaml.

    Returns:
        A tickable behaviour.
    """
    step = step_factory(config)

    return py_trees.composites.Sequence(
        "Mission 3: Hot Spot Drop",
        memory=True,
        children=[
            step("ApproachBoxArea", "fly to the box-area anchor and hold"),
            step(
                "ScanBoxes",
                "sweep the 3 boxes with the thermal array and record a "
                "temperature per box",
            ),
            step(
                "SelectHotBox",
                "pick the hottest box. A wrong box still scores 2 of 4 for the "
                "drop, so ALWAYS commit to a choice rather than aborting",
            ),
            step("HoverOverBox", "centre and hold directly above the selected box"),
            # --- Hb: 3 points, awarded on the blink alone. Keep this first. ---
            step(
                "BlinkHotspotLed",
                "blink the red LED while directly above the box. This alone "
                "earns Hb=3, so it must run BEFORE the drop and must not "
                "depend on the dropper working",
            ),
            MarkObjective("Mark hot box detected", "m3.hotbox_detected"),
            # --- Dr: 4 points in the correct box, 2 in any other. ---
            step("DropCone", "release the cone into the box below"),
            MarkObjective("Mark cone dropped", "m3.cone_dropped"),
        ],
    )
