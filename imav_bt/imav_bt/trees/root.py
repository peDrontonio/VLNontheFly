"""
root.py — the top of the tree: safety, flight-plan selection, idle.

    Root (Selector, memory=False)
    ├── SafetyOverride      Sequence: is something wrong? -> recover
    ├── FlightPlanRunner    Selector: run whichever plan is selected
    │   ├── FlightA         guarded by FlightPlanIs("flight_a")
    │   └── FlightB         guarded by FlightPlanIs("flight_b")
    └── Idle                nothing selected, or nothing running

Read it as a priority list: *if something is wrong, deal with that; otherwise
fly the selected plan; otherwise sit still.*


Why the root Selector has memory=False
---------------------------------------
This is the single most important structural choice in the package.

With ``memory=False`` a Selector re-ticks its children from the top on every
tick. So the safety branch is re-evaluated ~10 times a second, no matter how
deep in a mission the tree currently is. The instant SafetyTripped returns
SUCCESS, the selector takes the safety branch and py_trees invalidates
everything at lower priority -- the running mission subtree is stopped, its
leaves get terminate() called, and GotoAnchor releases its goal.

With ``memory=True`` the selector would resume straight into the running
mission branch and never look at the safety check again until that branch
finished. A low battery mid-mission would go unnoticed. That is a crash, not a
style preference.

The cost of memory=False is that lower-priority children must tolerate being
re-entered and interrupted; that is why the flight plans below sit behind cheap
condition guards, and why the flight subtrees themselves use memory=True
internally so they resume where they left off rather than restarting.

See docs/TREE_SEMANTICS.md for the full set of memory rules used here.


Why the safety Sequence also has memory=False
----------------------------------------------
Same reason, one level down: the SafetyTripped condition must be re-evaluated
every tick. With memory=True, once the recovery action started running the
sequence would stop re-checking whether the emergency was still real.
"""

from __future__ import annotations

from typing import Optional

import py_trees

from imav_bt import blackboard as bb
from imav_bt.behaviors.guards import FlightPlanIs, SafetyLimits, SafetyTripped
from imav_bt.behaviors.stubs import Stub, step_factory
from imav_bt.trees import flight_a, flight_b


def safety_limits_from(config: Optional[dict]) -> SafetyLimits:
    """Build SafetyLimits from the ``safety`` section of config/bt.yaml.

    Args:
        config: the top-level config dict; its ``safety`` key is read.

    Returns:
        Populated SafetyLimits, with library defaults for anything absent.
    """
    section = (config or {}).get("safety") or {}
    geofence = section.get("geofence")
    if geofence is not None:
        if len(geofence) != 6:
            raise ValueError(
                "safety.geofence must be [xmin, xmax, ymin, ymax, zmin, zmax], "
                f"got {geofence!r}"
            )
        geofence = tuple(float(v) for v in geofence)

    return SafetyLimits(
        min_battery=float(section.get("min_battery", 0.25)),
        max_odom_age_s=float(section.get("max_odom_age_s", 1.0)),
        geofence=geofence,
        require_odom=bool(section.get("require_odom", True)),
    )


def safety_branch(config: Optional[dict] = None):
    """Highest-priority branch: detect an emergency and act on it.

    Returns SUCCESS while an emergency is being handled, which keeps the root
    Selector parked here and prevents the mission from resuming.
    """
    step = step_factory((config or {}).get("safety"))

    return py_trees.composites.Sequence(
        "SafetyOverride",
        memory=False,  # re-evaluate the condition every tick; see module docstring
        children=[
            SafetyTripped("Emergency?", limits=safety_limits_from(config)),
            step(
                "EmergencyLand",
                "controlled descent and disarm at the current position. The "
                "reason is on the blackboard key `safety_reason`. NOTE the RC "
                "kill switch remains the real last resort -- this is a "
                "software-level response, not a substitute for it",
            ),
        ],
    )


def flight_plan_runner(config: Optional[dict] = None, arena: Optional[dict] = None):
    """Select and run whichever flight plan the operator has chosen.

    Each plan sits behind a cheap FlightPlanIs condition, and the Selector uses
    memory=False so switching plans mid-flight takes effect on the next tick.
    """
    return py_trees.composites.Selector(
        "FlightPlanRunner",
        memory=False,
        children=[
            py_trees.composites.Sequence(
                "Run FlightA",
                memory=False,
                children=[
                    FlightPlanIs("Plan is FlightA?", bb.FlightPlan.FLIGHT_A),
                    flight_a.build(config, arena),
                ],
            ),
            py_trees.composites.Sequence(
                "Run FlightB",
                memory=False,
                children=[
                    FlightPlanIs("Plan is FlightB?", bb.FlightPlan.FLIGHT_B),
                    flight_b.build(config, arena),
                ],
            ),
        ],
    )


def idle():
    """Do nothing, forever. Reached when no plan is selected or nothing is running.

    Returns RUNNING rather than SUCCESS so the tree reads as "idling" rather
    than "finished" in the tree watcher, and so the root never reports a
    completed status that a supervisor might act on.
    """
    return Stub(
        "Idle",
        outcome="running",
        note=(
            "waiting for an operator to select a flight plan and call "
            "~/set_running. The drone is not commanded here -- whatever the "
            "planner was last told still stands"
        ),
    )


def build_root(config: Optional[dict] = None, arena: Optional[dict] = None):
    """Assemble the complete behaviour tree.

    Args:
        config: merged config -- the ``safety`` section from config/bt.yaml plus
            the ``missions`` and ``landing`` sections from config/missions.yaml.
        arena: anchor table from config/arena.yaml.

    Returns:
        The root behaviour, ready to hand to a py_trees BehaviourTree.

    Example:
        >>> root = build_root(config, arena)
        >>> tree = py_trees.trees.BehaviourTree(root)
        >>> py_trees.trees.setup(root=root, node=ros_node)   # NOT root.setup()
    """
    return py_trees.composites.Selector(
        "Root",
        memory=False,  # reactive: safety is re-checked on every tick
        children=[
            safety_branch(config),
            flight_plan_runner(config, arena),
            idle(),
        ],
    )
