"""
mission2_darkroom.py — Mission 2, inspect the dark room. PLACEHOLDER.

Rulebook 4.4.3 / Table 3. A 2.5 m cube with two window openings (one red-bordered,
one blue). Fly in, determine how many baby dolls are inside, fly out, and display
the number on a screen.

    S2 = (Fin·A + Fout·A + Ba·A) · W2 + La2 + B2 + SF2

    Fin   2 pts entering via the red window, 1 via the blue
    Fout  2 pts exiting  via the red window, 1 via the blue
    Ba    3 · (Counted / Nba_max)
    -> 2 + 2 + 3 = 7 raw, the mission cap.

*** The asymmetry that must be encoded in behaviour, not left to a human ***
------------------------------------------------------------------------
Table 3 states: **if Counted > Nba_max then 0 points.** Not "0 for the count" --
the counting term collapses entirely.

So the two directions of error are wildly unequal:

    undercount by one   ->  lose 3/Nba_max points   (a fraction)
    overcount by one    ->  lose ALL 3 count points

The counting behaviour must therefore be deliberately conservative: require the
same doll to be seen from several viewpoints before it is counted, and when
uncertain, report the LOWER bound. A detector tuned for balanced precision and
recall is the wrong tuning for this mission; it should be tuned to avoid false
positives even at the cost of misses.

Entry and exit are scored separately and independently (Fin and Fout are
distinct terms), so each gets its own red-preferred ladder. Failing to enter,
though, means there is nothing to count -- unlike mission 1, the steps here are
genuinely sequential.

STATUS: placeholder. Note that counting dolls is the objective classical CV
handles worst; this is the most likely place to want a learned detector or the
VLM re-attached. See docs/ADDING_A_MISSION.md.
"""

from __future__ import annotations

from typing import Optional

import py_trees

from imav_bt.behaviors.bookkeeping import MarkObjective
from imav_bt.behaviors.stubs import step_factory

MISSION = "mission2"


def _window_ladder(name: str, direction: str, step):
    """Enter or exit through the red window (2 pts) if possible, else blue (1 pt).

    No skip branch here, unlike mission 1: if the drone cannot get through a
    window at all there is no way to continue the mission, so the ladder is
    allowed to fail and the mission is skipped by its SkipOnFailure wrapper.
    """
    return py_trees.composites.Selector(
        name,
        memory=True,
        children=[
            py_trees.composites.Sequence(
                f"{name} via red", memory=True,
                children=[
                    step(f"Align{direction}Red", "line up with the red-bordered window"),
                    step(f"Fly{direction}Red", "fly through the red window"),
                    MarkObjective(f"Mark {direction} red", f"m2.{direction.lower()}_red"),
                ],
            ),
            py_trees.composites.Sequence(
                f"{name} via blue", memory=True,
                children=[
                    step(f"Align{direction}Blue", "line up with the blue-bordered window"),
                    step(f"Fly{direction}Blue", "fly through the blue window"),
                    MarkObjective(f"Mark {direction} blue", f"m2.{direction.lower()}_blue"),
                ],
            ),
        ],
    )


def build(config: Optional[dict] = None, arena: Optional[dict] = None):
    """Build the Mission 2 subtree.

    Args:
        config: the ``missions.mission2`` section of config/missions.yaml.
        arena: anchor table from config/arena.yaml.

    Returns:
        A tickable behaviour.
    """
    step = step_factory(config)

    return py_trees.composites.Sequence(
        "Mission 2: Dark Room",
        memory=True,
        children=[
            step("ApproachRoom", "fly to the room face anchor and hold"),
            _window_ladder("Enter room", "Enter", step),
            step("Illuminate", "switch on the onboard light; the room is unlit"),
            step(
                "SweepAndCount",
                "sweep the room and count the dolls CONSERVATIVELY -- require "
                "N-of-M agreement across viewpoints and report the LOWER bound, "
                "because overcounting scores 0 for the whole counting term",
            ),
            MarkObjective("Mark counted", "m2.counted"),
            _window_ladder("Exit room", "Exit", step),
            step("PublishCount", "display the count on the ground-station screen"),
        ],
    )
