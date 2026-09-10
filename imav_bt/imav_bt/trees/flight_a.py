"""
flight_a.py — the high-value flight: missions 1, 2 and 3 chained, then land.

*** This is where most of the available points are, and the reason is
    structural rather than technical. ***

Rulebook 4.3.3, 4.3.4 and 4.3.6 together:

    landing bonus        2 pts (moving platform), "applied to EACH corresponding
                         mission score" when several missions share a flight
    multi-mission bonus  2 pts for 3 missions in one flight, "counted for EACH
                         mission involved"
    4.3.6                "If 3 missions are completed in one flight and the
                         drone successfully lands on the platform, BOTH the
                         multi-mission bonus and the landing bonus are awarded
                         to each of the three mission scores"

So chaining missions 1-3 into a single flight ending on the moving platform adds
(2 + 2) x 3 = 12 points that simply do not exist if the same three missions are
flown separately. The rulebook says the design is "intentional, to encourage
efficient mission chaining".

What that means for this tree
------------------------------
Finishing the flight is worth more than any individual mission. A mission that
fails must NOT abort the flight, because doing so would forfeit the landing and
multi-mission bonuses belonging to the missions that already succeeded -- one
failure would cost points on work that went fine.

Hence every mission slot is wrapped by `guarded()`, whose outermost layer is
SkipOnFailure (see decorators/policy.py). The mission's own points are lost
either way; the bonuses are not.

The exceptions are deliberate:

    ArmAndTakeoff   NOT wrapped. If takeoff fails there is nothing to continue
                    to, and masking that failure would let the tree "fly" the
                    whole mission list on the ground.
    Landing         NOT wrapped. It is a ladder that already degrades internally
                    (moving -> static -> plain descent) and its last rung cannot
                    fail. See trees/landing.py.

STATUS: the flight structure is complete and tested. The mission subtrees it
calls are placeholders.
"""

from __future__ import annotations

from typing import Optional

import py_trees

from imav_bt.behaviors.bookkeeping import StopRunning
from imav_bt.behaviors.flight import arm_and_takeoff
from imav_bt.behaviors.stubs import Stub
from imav_bt.decorators.policy import guarded
from imav_bt.trees import (
    landing,
    mission1_obstacle,
    mission2_darkroom,
    mission3_hotspot,
)

#: Mission slots flown by FlightA, in order. Order matters: it should follow the
#: physical layout of the arena, not the mission numbering, once the anchors are
#: surveyed.
CHAIN = (
    ("mission1", mission1_obstacle),
    ("mission2", mission2_darkroom),
    ("mission3", mission3_hotspot),
)


def mission_slot(key: str, module, config: dict, arena: Optional[dict]):
    """Build one mission and wrap it in the policy decorators.

    A disabled mission is replaced by a stub that succeeds immediately, rather
    than being dropped from the tree, so the tree diagram still shows the whole
    flight and re-enabling it is a config edit.

    Args:
        key: mission key, e.g. ``"mission1"``.
        module: the mission module, exposing ``build(config, arena)``.
        config: the ``missions`` section of config/missions.yaml.
        arena: anchor table from config/arena.yaml.

    Returns:
        The decorated mission subtree.
    """
    cfg = (config or {}).get(key) or {}

    if not cfg.get("enabled", True):
        return Stub(
            f"{key} (disabled)",
            outcome="success",
            note=f"enable with missions.{key}.enabled: true in config/missions.yaml",
        )

    return guarded(
        name=key,
        child=module.build(cfg, arena),
        deadline=float(cfg.get("deadline_s", 180.0)),
        attempts=int(cfg.get("attempts", 1)),
        mission=key,
    )


def build(config: Optional[dict] = None, arena: Optional[dict] = None):
    """Build the FlightA subtree: takeoff, missions 1-3, precision landing.

    Args:
        config: the whole missions config (``missions`` and ``landing`` keys).
        arena: anchor table from config/arena.yaml.

    Returns:
        A tickable behaviour.
    """
    config = config or {}
    missions = config.get("missions") or {}

    children = [arm_and_takeoff()]
    children += [mission_slot(key, mod, missions, arena) for key, mod in CHAIN]
    children.append(landing.build(config.get("landing"), arena))
    # Must be last: without it the completed flight restarts and takes off
    # again on the next tick. See behaviors/bookkeeping.py:StopRunning.
    children.append(StopRunning())

    # memory=True: once a mission is done the flight moves on and never
    # re-enters it. With memory=False every tick would restart from takeoff.
    return py_trees.composites.Sequence("FlightA: M1 -> M2 -> M3 -> Land", memory=True,
                                        children=children)
