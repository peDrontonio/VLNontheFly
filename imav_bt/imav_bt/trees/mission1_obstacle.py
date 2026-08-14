"""
mission1_obstacle.py — Mission 1, the obstacle course. PLACEHOLDER.

Rulebook 4.4.1 / Table 2. The drone flies a lane containing, in order: a window
panel, a red bar to pass OVER, two blue bars to pass UNDER, a PVC tube passage,
and a second window panel.

    S1 = 7/4 · (Win·A + BarR·A + BarB·A + Tub·A) · W1 + B1 + La1 + SF1

with the raw objectives worth:

    Win     1.0 pt per red window, 0.5 pt per blue window (two panels)
    BarR    0.75 high / 0.5 middle / 0.25 low   (flying over it)
    BarB    0.75 low  / 0.5 middle / 0.25 high  (flying under it)
    Tub     0.5 pt
    -> 2.0 + 0.75 + 0.75 + 0.5 = 4.0 raw, and 7/4 · 4.0 = 7, the mission cap.

The structural consequence: EVERY OBJECTIVE IS INDEPENDENT
-----------------------------------------------------------
Missing one gate costs only that gate's points. So no gate may be allowed to
fail the mission -- each one degrades to "skip and move on". That is why every
gate below is a Selector ending in a skip branch, and why the mission Sequence
itself can only fail if something catastrophic happens.

The same logic drives the harder/easier choice within a gate: try the
higher-scoring option first, fall back to the lower-scoring one, and only then
skip. A red window is worth double a blue one, so red is attempted first; the
red bar scores most when crossed high, the blue bars when crossed low.

STATUS: placeholder. Every leaf is a Stub. Replace them with real detection and
traversal behaviours -- the tree shape should not need to change.
See docs/ADDING_A_MISSION.md.
"""

from __future__ import annotations

from typing import Optional

import py_trees

from imav_bt.behaviors.bookkeeping import MarkObjective
from imav_bt.behaviors.stubs import Stub, step_factory

MISSION = "mission1"


def _gate_option(label: str, step, objective: str, detect: str, traverse: str):
    """One rung of a degradation ladder: detect it, cross it, record it.

    memory=True so a multi-tick traversal is not restarted from the detection
    step on every tick.
    """
    return py_trees.composites.Sequence(
        label,
        memory=True,
        children=[
            step(detect, f"detect the {label.lower()}"),
            step(traverse, f"generate and fly the traversal waypoints for {label.lower()}"),
            MarkObjective(f"Mark {objective}", objective),
        ],
    )


def _ladder(name: str, options, skip_note: str):
    """Try each option in order of decreasing score, then give up gracefully.

    memory=True: once an option has failed, the ladder moves on and does not
    re-attempt it while it is still running. With memory=False it would restart
    from the highest-scoring option on every tick and never make progress down
    the ladder. See docs/TREE_SEMANTICS.md.
    """
    return py_trees.composites.Selector(
        name,
        memory=True,
        children=list(options)
        + [Stub(f"{name}: skip", outcome="success", note=skip_note)],
    )


def build(config: Optional[dict] = None, arena: Optional[dict] = None):
    """Build the Mission 1 subtree.

    Args:
        config: the ``missions.mission1`` section of config/missions.yaml.
        arena: anchor table from config/arena.yaml. Unused while every leaf is
            a stub; real traversal behaviours will need it.

    Returns:
        A tickable behaviour.
    """
    step = step_factory(config)

    def window_panel(panel: str):
        """A window panel: red opening (1.0 pt) preferred over blue (0.5 pt)."""
        return _ladder(
            f"Window {panel}",
            [
                _gate_option(
                    f"Window {panel} red", step,
                    objective=f"m1.window_{panel.lower()}_red",
                    detect=f"DetectWindow{panel}Red",
                    traverse=f"TraverseWindow{panel}Red",
                ),
                _gate_option(
                    f"Window {panel} blue", step,
                    objective=f"m1.window_{panel.lower()}_blue",
                    detect=f"DetectWindow{panel}Blue",
                    traverse=f"TraverseWindow{panel}Blue",
                ),
            ],
            skip_note=f"window panel {panel} not crossed; continue the course",
        )

    return py_trees.composites.Sequence(
        "Mission 1: Obstacle Course",
        memory=True,
        children=[
            window_panel("A"),
            # Red bar: flown OVER, so the HIGH setting scores most (0.75).
            _ladder(
                "Red bar",
                [
                    _gate_option(
                        "Red bar high", step, objective="m1.bar_red_high",
                        detect="DetectRedBar", traverse="CrossRedBarHigh",
                    ),
                    _gate_option(
                        "Red bar middle", step, objective="m1.bar_red_middle",
                        detect="DetectRedBar", traverse="CrossRedBarMiddle",
                    ),
                ],
                skip_note="red bar not crossed; continue the course",
            ),
            # Blue bars: flown UNDER, so the LOW setting scores most (0.75).
            _ladder(
                "Blue bars",
                [
                    _gate_option(
                        "Blue bars low", step, objective="m1.bar_blue_low",
                        detect="DetectBlueBars", traverse="CrossBlueBarsLow",
                    ),
                    _gate_option(
                        "Blue bars middle", step, objective="m1.bar_blue_middle",
                        detect="DetectBlueBars", traverse="CrossBlueBarsMiddle",
                    ),
                ],
                skip_note="blue bars not crossed; continue the course",
            ),
            _ladder(
                "Tube passage",
                [
                    _gate_option(
                        "Tubes", step, objective="m1.tubes",
                        detect="DetectTubes", traverse="CrossTubes",
                    )
                ],
                skip_note="tube passage not crossed; continue the course",
            ),
            window_panel("B"),
        ],
    )
