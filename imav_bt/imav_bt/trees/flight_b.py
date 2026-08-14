"""
flight_b.py — the wind-turbine flight: mission 4 alone.

Mission 4 gets its own flight plan because the rulebook forbids combining it
(4.4.8): "is the only mission that cannot be combined with the others and does
not require landing on the landing pad." Table 5 confirms it structurally --
S4 has no La (landing) and no B (multi-mission) term, unlike Tables 2-4.

So none of FlightA's chaining logic applies here. There is nothing to chain to
and no bonus to protect.

Mission 4 is still wrapped in SkipOnFailure, but for a different reason
-----------------------------------------------------------------------
In FlightA the wrapper protects the bonuses of missions that already succeeded.
Here it protects the LANDING: if the grab or the disc touch fails, the drone
must still fly to the turbine base and put itself down, rather than returning
FAILURE and leaving the aircraft airborne with nothing driving it.

The same decorator, a different justification -- worth understanding before
copying the pattern into a new flight plan.

The tether
----------
Once the ring is grabbed the drone is physically tethered by 4 m of 0.1 mm
enamelled wire, and the rulebook requires landing near the turbine base to avoid
snapping it. That is why the final step here is a base-relative landing rather
than the shared precision-landing ladder in trees/landing.py -- there are no
landing points to win, and flying to the landing platform would drag the wire
across the arena.

STATUS: the flight structure is complete. The mission subtree is a placeholder.
"""

from __future__ import annotations

from typing import Optional

import py_trees

from imav_bt.behaviors.bookkeeping import StopRunning
from imav_bt.behaviors.flight import arm_and_takeoff
from imav_bt.behaviors.stubs import Stub, step_factory
from imav_bt.decorators.policy import guarded
from imav_bt.trees import mission4_turbine

MISSION = "mission4"


def build(config: Optional[dict] = None, arena: Optional[dict] = None):
    """Build the FlightB subtree: takeoff, mission 4, land near the turbine base.

    Args:
        config: the whole missions config (``missions`` key).
        arena: anchor table from config/arena.yaml.

    Returns:
        A tickable behaviour.
    """
    config = config or {}
    cfg = (config.get("missions") or {}).get(MISSION) or {}
    step = step_factory(cfg)

    if not cfg.get("enabled", True):
        mission = Stub(
            f"{MISSION} (disabled)",
            outcome="success",
            note=f"enable with missions.{MISSION}.enabled: true in config/missions.yaml",
        )
    else:
        mission = guarded(
            name=MISSION,
            child=mission4_turbine.build(cfg, arena),
            deadline=float(cfg.get("deadline_s", 240.0)),
            attempts=int(cfg.get("attempts", 1)),
            mission=MISSION,
        )

    return py_trees.composites.Sequence(
        "FlightB: M4 Turbine",
        memory=True,
        children=[
            arm_and_takeoff(),
            mission,
            step(
                "LandNearTurbineBase",
                "descend and land close to the turbine base. Required by "
                "rulebook 4.4.7 so the 4 m copper wire is not snapped -- do "
                "NOT return to the landing platform",
            ),
            # Must be last: see behaviors/bookkeeping.py:StopRunning.
            StopRunning(),
        ],
    )
