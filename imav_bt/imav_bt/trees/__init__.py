"""
Tree and subtree assembly.

    root.py               the whole tree: safety / flight plans / idle
    flight_a.py           missions 1-3 chained, then precision landing
    flight_b.py           mission 4 alone (rulebook forbids combining it)
    mission1_obstacle.py  ┐
    mission2_darkroom.py  │ PLACEHOLDERS -- structure and scoring rationale are
    mission3_hotspot.py   │ documented, every leaf is a Stub
    mission4_turbine.py   ┘
    landing.py            precision-landing ladder, also a placeholder

Every builder has the same signature::

    build(config: dict | None, arena: dict | None) -> py_trees.behaviour.Behaviour

so a new mission drops in without any special handling. See
docs/ADDING_A_MISSION.md.
"""

from imav_bt.trees.root import build_root

__all__ = ["build_root"]
