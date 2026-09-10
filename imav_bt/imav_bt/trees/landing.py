"""
landing.py — precision landing, and the plain landing fallback. PLACEHOLDER.

Rulebook 4.2.1 / 4.3.3. The landing platform is 80x80 cm, carries an ArUco 5x5
marker (id1) of exactly that size, and runs in one of three modes:

    1. horizontal, stationary
    2. horizontal, translating at 0.1 m/s
    3. pitched 10 deg, translating at 0.1 m/s

Scoring:

    2 pts  landing on the MOVING platform
    1 pt   landing on the static platform

Why this is worth more than it looks
-------------------------------------
Rulebook 4.3.3: "If multiple missions are completed during a single flight, the
landing bonus is applied to each corresponding mission score." So in FlightA,
which chains missions 1-3, a moving-platform landing is worth 2 points x 3
missions = 6 points, and the difference between a moving and a static landing is
3 points, not 1.

That justifies attempting the moving platform first and only degrading to the
static pad if the tracking will not converge -- but it also means a failed
landing must never mean *no* landing. Hence the three-rung ladder, ending in a
plain descent that scores nothing but puts the aircraft down safely.

Ladder order, highest score first:

    moving platform   2 pts x every mission in the flight
    static platform   1 pt  x every mission in the flight
    plain land        0 pts, but safe

STATUS: placeholder. No ArUco detection exists in the repo yet. The real
implementation needs marker tracking plus a descent controller that matches the
platform's 0.1 m/s translation -- and, for mode 3, tolerates a 10 deg tilt.
"""

from __future__ import annotations

from typing import Optional

import py_trees

from imav_bt.behaviors.bookkeeping import MarkObjective
from imav_bt.behaviors.flight import land
from imav_bt.behaviors.stubs import step_factory


def build(config: Optional[dict] = None, arena: Optional[dict] = None):
    """Build the precision-landing subtree.

    Args:
        config: the ``landing`` section of config/missions.yaml.
        arena: anchor table from config/arena.yaml.

    Returns:
        A tickable behaviour that always finishes with the drone on the ground.
    """
    step = step_factory(config)

    def approach(label: str, objective: str, note: str):
        return py_trees.composites.Sequence(
            f"Land on {label} platform",
            memory=True,
            children=[
                step(f"Find{label}Marker", "locate ArUco id1 on the landing platform"),
                step(f"Track{label}Platform", note),
                step(f"Descend{label}", "gated descent, aborting if the marker is lost"),
                step(f"Confirm{label}Touchdown", "confirm touchdown, then disarm"),
                MarkObjective(f"Mark {objective}", objective),
            ],
        )

    return py_trees.composites.Selector(
        "Precision Landing",
        memory=True,
        children=[
            approach(
                "Moving", "landing.moving",
                "track the platform and match its 0.1 m/s translation; "
                "tolerate the 10 deg pitched mode",
            ),
            approach(
                "Static", "landing.static",
                "hold station over the stationary platform",
            ),
            # Last resort: no points, but the drone is down and undamaged.
            land(),
        ],
    )
