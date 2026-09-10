"""
stubs.py — placeholder leaves, the vocabulary the mission subtrees are built
from until the real logic replaces them.

The whole point of this package's current phase is that the tree's CONTROL FLOW
is finished and tested while the mission logic is not yet written. Stubs are
what make that possible: every mission subtree is assembled from real
composites and real decorators, with Stub leaves standing in for the steps that
will eventually detect a window, drop a cone or grab a ring.

So the thing being validated today is exactly the thing that is hard to change
later -- ordering, retry policy, what happens when a mission fails -- while the
leaf bodies, which are easy to swap, are deferred.

Replacing a stub
----------------
Swap the Stub for a real behaviour with the same name. Nothing else in the tree
changes. See docs/ADDING_A_BEHAVIOR.md.

    # before
    Stub("DetectRedWindow", outcome="success")
    # after
    DetectRedWindow(name="DetectRedWindow", min_confidence=0.6)

Making a stub fail on purpose
-----------------------------
Every stub reads its outcome from config (config/missions.yaml), so fault
injection needs no code change -- flip one value and confirm the tree degrades
the way it should. This is how the "a failed mission must not abort the flight"
behaviour is exercised end-to-end on the bench. See docs/TESTING.md.
"""

from __future__ import annotations

from typing import Optional

from py_trees.common import Status

from imav_bt.behaviors.base import RosBehaviour

#: Accepted `outcome` values for Stub.
OUTCOMES = {
    "success": Status.SUCCESS,
    "failure": Status.FAILURE,
    "running": Status.RUNNING,  # never finishes; use to test Deadline
}


class Stub(RosBehaviour):
    """A placeholder leaf that pretends to do work, then returns a fixed result.

    Args:
        name: behaviour name. Use the name the REAL behaviour will have, so the
            tree diagram already reads correctly and swapping it in is a
            one-line change.
        outcome: one of ``"success"``, ``"failure"``, ``"running"``.
            ``"running"`` never finishes, which is how the silent-stall case is
            simulated for Deadline.
        ticks: how many ticks to spend RUNNING before returning ``outcome``.
            1 means finish immediately. Use a larger number to make bench runs
            legible in the tree watcher instead of flashing past.
        note: what the real implementation will do. Shown as the feedback
            message, so `py-trees-tree-watcher` doubles as a to-do list.

    Raises:
        ValueError: if ``outcome`` is not a known value or ``ticks`` < 1.
    """

    requires_node = False

    def __init__(
        self,
        name: str,
        outcome: str = "success",
        ticks: int = 1,
        note: str = "",
    ):
        super().__init__(name)
        key = (outcome or "").strip().lower()
        if key not in OUTCOMES:
            raise ValueError(
                f"{name}: outcome must be one of {sorted(OUTCOMES)}, got {outcome!r}"
            )
        if ticks < 1:
            raise ValueError(f"{name}: ticks must be >= 1, got {ticks}")
        self.outcome = OUTCOMES[key]
        self.outcome_name = key
        self.ticks = ticks
        self.note = note
        self._elapsed = 0

    def initialise(self) -> None:
        self._elapsed = 0

    def update(self) -> Status:
        self._elapsed += 1
        suffix = f" — TODO: {self.note}" if self.note else ""

        if self.outcome_name == "running":
            self.feedback_message = f"stub: never finishes{suffix}"
            return Status.RUNNING

        if self._elapsed < self.ticks:
            self.feedback_message = (
                f"stub: working {self._elapsed}/{self.ticks}{suffix}"
            )
            return Status.RUNNING

        self.feedback_message = f"stub: {self.outcome_name}{suffix}"
        return self.outcome


def stub_from_config(
    name: str,
    config: Optional[dict] = None,
    note: str = "",
) -> Stub:
    """Build a Stub from a config dict, so fault injection needs no code edit.

    Reads ``outcome`` and ``ticks`` from ``config`` -- typically one entry of
    ``config/missions.yaml`` -- falling back to a plain success.

    Args:
        name: behaviour name.
        config: mapping that may contain ``outcome`` and ``ticks``.
        note: description of the real implementation, for the feedback message.

    Returns:
        A configured Stub.

    Example:
        >>> cfg = {"steps": {"DetectRedWindow": {"outcome": "failure"}}}
        >>> stub_from_config("DetectRedWindow", cfg["steps"]["DetectRedWindow"])
    """
    config = config or {}
    return Stub(
        name=name,
        outcome=str(config.get("outcome", "success")),
        ticks=int(config.get("ticks", 1)),
        note=note,
    )


def step_factory(mission_config: Optional[dict] = None):
    """Return a ``step(name, note)`` helper bound to one mission's config.

    Every mission subtree opens with this, so each of its placeholder steps can
    be individually configured (and individually made to fail) from
    config/missions.yaml without touching the tree code::

        step = step_factory(cfg)
        Sequence("Mission 3", memory=True, children=[
            step("ScanBoxes", "sweep the 3 boxes with the thermal array"),
            step("BlinkHotspotLed", "blink the red LED above the hot box"),
        ])

    Args:
        mission_config: one mission's config dict; its ``steps`` mapping is
            consulted for per-step ``outcome`` and ``ticks`` overrides.

    Returns:
        A callable ``(name, note="") -> Stub``.
    """
    steps = (mission_config or {}).get("steps") or {}

    def step(name: str, note: str = "") -> Stub:
        return stub_from_config(name, steps.get(name), note)

    return step
