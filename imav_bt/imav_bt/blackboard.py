"""
blackboard.py — the typed key schema shared by every behaviour.

Why a schema at all
-------------------
py_trees' blackboard is a process-global key/value store. That is convenient
and, left unmanaged, becomes the usual mess: keys invented at the call site,
typos that silently create new keys, and no way to know what a subtree actually
needs. Every key this package uses is therefore declared once, here, with a
type and a default.

Two properties follow, and both are load-bearing for this project:

  * A subtree can be unit-tested with no ROS graph at all. Call reset(), write
    the handful of keys the subtree reads, tick it, assert on the result. That
    is exactly what test/test_trees_smoke.py does.

  * Behaviours declare their key access up front (py_trees enforces READ vs
    WRITE), so `py-trees-tree-watcher` and the failure logs show which
    behaviour touched which key.

Who writes what
---------------
Keys divide cleanly by writer, and mixing this up is the most likely source of
confusion when adding behaviours:

    SENSED     written ONLY by bt_engine's ROS subscriptions, read by anyone.
               A behaviour writing these is a bug -- it would be fabricating
               sensor data.
    COMMANDED  written by operator-facing services (set_running, abort).
    DERIVED    written by behaviours, read by other behaviours. Mission
               outcomes and objective bookkeeping live here.

Global-state warning
--------------------
py_trees.blackboard.Blackboard.storage is class-level, i.e. shared across every
tree in the process. Tests MUST call reset() in setUp() or state leaks between
cases. reset() is also called once by bt_engine at startup.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable, Sequence

import py_trees

#: All imav_bt keys live under this namespace, so they cannot collide with keys
#: created by py_trees' own demo behaviours or by anything added later.
NAMESPACE = "/imav"


class FlightPlan:
    """Which top-level plan the flight-plan selector should run.

    IMAV rulebook 4.4.8 forbids combining mission 4 with the others, which is
    why there are two flight plans rather than one ordering of four missions.
    """

    IDLE = "idle"
    #: Missions 1 -> 2 -> 3 chained, ending in a precision landing. This is the
    #: high-value plan: the multi-mission and landing bonuses are each awarded
    #: to EVERY mission in the flight, so chaining is worth +12 points over
    #: flying the three missions separately.
    FLIGHT_A = "flight_a"
    #: Mission 4 (wind turbine) alone, landing near the turbine base.
    FLIGHT_B = "flight_b"

    ALL = (IDLE, FLIGHT_A, FLIGHT_B)


class Outcome:
    """Per-mission result recorded by the bookkeeping behaviours."""

    PENDING = "pending"
    SUCCESS = "success"
    #: The mission failed but the flight continued. This is a deliberate,
    #: score-maximising outcome, not an error -- see decorators/policy.py.
    SKIPPED = "skipped"
    FAILED = "failed"


@dataclass(frozen=True)
class Key:
    """One blackboard key: its name, type, default and who is allowed to write it."""

    name: str
    type: type
    factory: Callable[[], Any]
    writer: str  # "SENSED" | "COMMANDED" | "DERIVED"
    doc: str
    optional: bool = False  # if True, None is an accepted value


SCHEMA: tuple[Key, ...] = (
    # ---------------- SENSED: written only by bt_engine's subscriptions -----
    Key(
        "odom_valid", bool, lambda: False, "SENSED",
        "True once a recent /odometry message has been received. Gates the "
        "whole tree via guards.LocalizationHealthy.",
    ),
    Key(
        "position", tuple, lambda: (0.0, 0.0, 0.0), "SENSED",
        "Latest (x, y, z) in the ENU `map` frame, from /odometry.",
    ),
    Key(
        "yaw", float, lambda: 0.0, "SENSED",
        "Latest yaw in the ENU `map` frame [rad], from /odometry.",
    ),
    Key(
        "odom_stamp", float, lambda: 0.0, "SENSED",
        "Monotonic receipt time of the latest /odometry message [s]. Staleness "
        "is measured against this, not against the message header, so a frozen "
        "publisher is detected even if it keeps stamping fresh headers.",
    ),
    Key(
        "goal_status", str, lambda: "", "SENSED",
        "Last string seen on planning/goal_status. See interfaces.GoalStatus. "
        "Empty until the planner says something.",
    ),
    Key(
        "goal_status_stamp", float, lambda: 0.0, "SENSED",
        "Monotonic receipt time of the last goal_status message [s].",
    ),
    Key(
        "battery_fraction", float, lambda: 1.0, "SENSED",
        "Battery state of charge in [0, 1].",
    ),
    Key(
        "battery_valid", bool, lambda: False, "SENSED",
        "True once a battery message has arrived. While False the battery "
        "guard does NOT trip -- an absent sensor must not ground the drone.",
    ),

    # ---------------- COMMANDED: written by operator-facing services --------
    Key(
        "flight_plan", str, lambda: FlightPlan.IDLE, "COMMANDED",
        "Which plan the flight-plan selector runs. One of FlightPlan.ALL.",
    ),
    Key(
        "running", bool, lambda: False, "COMMANDED",
        "False holds the tree in idle regardless of flight_plan. This is the "
        "'go' switch; the tree ticks continuously but does nothing until set.",
    ),
    Key(
        "abort_requested", bool, lambda: False, "COMMANDED",
        "Latched operator abort. Forces the safety branch to win on the next "
        "tick. Cleared only by set_running(false).",
    ),

    # ---------------- DERIVED: written by behaviours -------------------------
    Key(
        "mission_outcomes", dict, dict, "DERIVED",
        "Mission name -> Outcome. Written by bookkeeping.RecordOutcome and by "
        "the SkipOnFailure decorator.",
    ),
    Key(
        "objectives", dict, dict, "DERIVED",
        "Objective key -> bool achieved. The granular record used to work out "
        "what was actually scored (e.g. 'm1.red_window_a': True).",
    ),
    Key(
        "active_goal", tuple, lambda: None, "DERIVED",
        "The (x, y, z) goal currently owned by a GotoAnchor leaf, or None. "
        "Used to detect two behaviours fighting over the goal topic.",
        optional=True,
    ),
    Key(
        "safety_reason", str, lambda: "", "DERIVED",
        "Why the safety branch triggered, for logging. Empty when nominal.",
    ),
)

_BY_NAME = {k.name: k for k in SCHEMA}


def key_names() -> tuple[str, ...]:
    """Every declared key name, unqualified (no namespace prefix)."""
    return tuple(k.name for k in SCHEMA)


def spec(name: str) -> Key:
    """Look up one key's declaration. Raises KeyError with a helpful message."""
    try:
        return _BY_NAME[name]
    except KeyError:
        raise KeyError(
            f"'{name}' is not a declared imav_bt blackboard key. "
            f"Declare it in imav_bt/blackboard.py:SCHEMA rather than writing "
            f"it ad hoc. Known keys: {', '.join(sorted(_BY_NAME))}"
        ) from None


def reset() -> None:
    """Clear the global blackboard and write every key back to its default.

    Call this once at engine startup, and in every test's setUp(). py_trees
    stores blackboard state on a class attribute, so without this, state leaks
    between trees and between test cases.
    """
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="imav_bt_defaults", namespace=NAMESPACE)
    for key in SCHEMA:
        writer.register_key(key=key.name, access=py_trees.common.Access.WRITE)
        setattr(writer, key.name, key.factory())


def client(
    name: str,
    read: Iterable[str] = (),
    write: Iterable[str] = (),
) -> py_trees.blackboard.Client:
    """Build a namespaced blackboard client with the given keys registered.

    Every key is validated against SCHEMA, so a typo fails loudly at
    construction time instead of silently creating a new key that nothing
    ever writes.

    Args:
        name: client name, shown in blackboard introspection output.
        read: key names to register for READ access.
        write: key names to register for WRITE access.

    Returns:
        A py_trees blackboard Client bound to the imav_bt namespace.
    """
    bb = py_trees.blackboard.Client(name=name, namespace=NAMESPACE)
    for key_name in read:
        spec(key_name)
        bb.register_key(key=key_name, access=py_trees.common.Access.READ)
    for key_name in write:
        spec(key_name)
        bb.register_key(key=key_name, access=py_trees.common.Access.WRITE)
    return bb


def validate() -> list[str]:
    """Type-check every key's current value against SCHEMA.

    Returns a list of human-readable problems; empty means the blackboard is
    consistent. Used by test_blackboard.py and by bt_engine's startup check --
    a mistyped key (say, a str written where a float is expected) otherwise
    surfaces much later as a confusing comparison error inside a guard.
    """
    reader = py_trees.blackboard.Client(name="imav_bt_validate", namespace=NAMESPACE)
    problems: list[str] = []
    for key in SCHEMA:
        reader.register_key(key=key.name, access=py_trees.common.Access.READ)
        try:
            value = getattr(reader, key.name)
        except KeyError:
            problems.append(f"{key.name}: not set (did you call blackboard.reset()?)")
            continue
        if value is None:
            if not key.optional:
                problems.append(f"{key.name}: is None but the key is not optional")
            continue
        if not isinstance(value, key.type):
            problems.append(
                f"{key.name}: expected {key.type.__name__}, "
                f"got {type(value).__name__} ({value!r})"
            )
    return problems


def snapshot() -> dict[str, Any]:
    """Return every key's current value as a plain dict, for logging/tests."""
    reader = py_trees.blackboard.Client(name="imav_bt_snapshot", namespace=NAMESPACE)
    out: dict[str, Any] = {}
    for key in SCHEMA:
        reader.register_key(key=key.name, access=py_trees.common.Access.READ)
        try:
            out[key.name] = getattr(reader, key.name)
        except KeyError:
            out[key.name] = None
    return out


def keys_by_writer(writer: str) -> Sequence[Key]:
    """All keys owned by one writer class: SENSED, COMMANDED or DERIVED."""
    return tuple(k for k in SCHEMA if k.writer == writer)
