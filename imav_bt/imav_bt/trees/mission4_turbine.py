"""
mission4_turbine.py — Mission 4, wind turbine grounding inspection. PLACEHOLDER.

Rulebook 4.4.7 / Table 5. Lift an 8 cm copper ring off a 50 cm support (ArUco
markers id4/id5/id6 assist positioning), then touch it against copper discs on a
turbine blade to close a continuity circuit and sound a buzzer.

    S4 = ((grabRing·A + touchWind·A) · W4 + SF4)

    grabRing   4 pts
    touchWind  5 pts
    -> 9 raw, and 9 · 2 (max mass factor) + 2 self-made = 20, the mission cap.

Why this is a separate flight plan
-----------------------------------
Rulebook 4.4.8: mission 4 "is the only mission that cannot be combined with the
others and does not require landing on the landing pad." There is no landing
bonus and no multi-mission bonus term in S4 -- compare Tables 2-4, which all
carry La and B terms. So mission 4 gets its own flight (trees/flight_b.py), and
chaining logic does not apply to it.

*** The tether constraint ***
------------------------------
The ring trails a 4 m length of 0.1 mm enamelled copper wire connected to the
continuity test rig. From the moment the ring is grabbed, the drone is
physically tethered. Rulebook 4.4.7: "At the end of the mission, the drone must
land near the base of the wind turbine to avoid any breaking of the copper
wire."

That is a hard constraint the tree cannot express with a plain sequence: it must
hold for the whole remainder of the flight, not at one step. The real
implementation should run a tether-radius guard in PARALLEL with everything
after GrabRing -- structurally similar to the root's safety branch, but scoped
to this subtree. It is left as a stub here because its recovery action (release
the ring? descend in place?) is a hardware decision that has not been made.

Note also that the ring "may be manually placed on the drone before takeoff, but
no grabbing points will be awarded in this case" -- i.e. skipping the grab is a
legitimate 4-point-cheaper strategy that still leaves 5 points on the table for
the disc touch. Set ``steps.GrabRing.outcome: failure`` in missions.yaml to
rehearse that variant.

STATUS: placeholder. No ArUco detection and no grabber driver exist in the repo.
See docs/ADDING_A_MISSION.md.
"""

from __future__ import annotations

from typing import Optional

import py_trees

from imav_bt.behaviors.bookkeeping import MarkObjective
from imav_bt.behaviors.stubs import step_factory

MISSION = "mission4"


def build(config: Optional[dict] = None, arena: Optional[dict] = None):
    """Build the Mission 4 subtree.

    Args:
        config: the ``missions.mission4`` section of config/missions.yaml.
        arena: anchor table from config/arena.yaml.

    Returns:
        A tickable behaviour.
    """
    step = step_factory(config)

    return py_trees.composites.Sequence(
        "Mission 4: Turbine Inspection",
        memory=True,
        children=[
            step(
                "ApproachRingSupport",
                "fly to the ring support using ArUco id4/id5/id6 for pose",
            ),
            step("VisualServoToRing", "close the last few cm onto the ring by visual servo"),
            step("GrabRing", "engage the grabber and lift the ring off its support"),
            step(
                "VerifyGrab",
                "confirm the ring is actually held before committing to the "
                "blade approach -- a false positive here drags the wire",
            ),
            MarkObjective("Mark ring grabbed", "m4.ring_grabbed"),
            # --- Everything past this point is tethered by 4 m of 0.1 mm wire. ---
            step(
                "ApproachBlade",
                "fly to the blade's copper discs, staying inside the tether "
                "radius. TODO: run a tether guard in PARALLEL from here on, "
                "not as a sequence step",
            ),
            step(
                "TouchCopperDisc",
                "press the ring against a copper disc and watch for continuity "
                "/ buzzer confirmation",
            ),
            MarkObjective("Mark disc touched", "m4.disc_touched"),
            step("RetreatFromBlade", "back off slowly without snatching the wire"),
        ],
    )
