"""
bookkeeping.py — leaves that record what happened, for scoring and debriefing.

None of these affect flight. They exist because after a 30-minute competition
slot (rulebook 2.2) you need to know exactly which objectives were achieved, and
reconstructing that from a rosbag under time pressure is miserable.

The distinction that matters
----------------------------
``mission_outcomes`` records whether a MISSION ran to completion; the
SkipOnFailure decorator writes it automatically (decorators/policy.py).
``objectives`` records the individual scorable items INSIDE a mission, which is
what the rulebook's scoring tables actually multiply out. For example mission 1
(Table 2) scores each window, each bar and the tube passage separately:

    m1.window_a_red     1.0 pt   (0.5 if the blue window was used instead)
    m1.bar_red_high     0.75 pt
    m1.bar_blue_low     0.75 pt
    m1.tubes            0.5 pt

Only the mission subtree knows which of these it managed, so it marks them as it
goes. Nothing in this package converts objectives into points yet -- the score
calculator is deliberately out of scope for this phase -- but recording them now
means the data exists when it is written.
"""

from __future__ import annotations

from typing import Optional

from py_trees.common import Status

from imav_bt import blackboard as bb
from imav_bt.behaviors.base import RosBehaviour


class MarkObjective(RosBehaviour):
    """Record one scorable objective, then return SUCCESS.

    Drop this into a sequence immediately after the step that earns the
    objective, so the record reflects what actually happened rather than what
    the tree intended.

    Args:
        name: behaviour name.
        objective: dotted key, conventionally ``"<mission>.<item>"``
            (e.g. ``"m1.window_a_red"``).
        achieved: value to record.

    Example:
        >>> Sequence("CrossRedWindow", memory=True, children=[
        ...     DetectRedWindow(...),
        ...     TraverseWindow(...),
        ...     MarkObjective("Mark", "m1.window_a_red"),
        ... ])
    """

    requires_node = False

    def __init__(self, name: str, objective: str, achieved: bool = True):
        super().__init__(name)
        self.objective = objective
        self.achieved = achieved
        self._bb = bb.client(f"{name}:objectives", write=["objectives"])

    def update(self) -> Status:
        objectives = dict(self._bb.objectives)
        objectives[self.objective] = self.achieved
        self._bb.objectives = objectives
        self.feedback_message = f"{self.objective} = {self.achieved}"
        return Status.SUCCESS


class RecordOutcome(RosBehaviour):
    """Record a mission outcome explicitly, then return SUCCESS.

    Usually unnecessary -- SkipOnFailure records outcomes on its own. Use this
    for missions that are NOT wrapped in SkipOnFailure, such as mission 4 in
    FlightB, where there is nothing to continue to if it fails.

    Args:
        name: behaviour name.
        mission: mission key (e.g. ``"mission4"``).
        outcome: one of :class:`imav_bt.blackboard.Outcome`.

    Raises:
        ValueError: if ``outcome`` is not a known outcome.
    """

    requires_node = False

    _VALID = (bb.Outcome.PENDING, bb.Outcome.SUCCESS, bb.Outcome.SKIPPED, bb.Outcome.FAILED)

    def __init__(self, name: str, mission: str, outcome: str = bb.Outcome.SUCCESS):
        super().__init__(name)
        if outcome not in self._VALID:
            raise ValueError(
                f"{name}: unknown outcome {outcome!r}; expected one of {self._VALID}"
            )
        self.mission = mission
        self.outcome = outcome
        self._bb = bb.client(f"{name}:outcomes", write=["mission_outcomes"])

    def update(self) -> Status:
        outcomes = dict(self._bb.mission_outcomes)
        outcomes[self.mission] = self.outcome
        self._bb.mission_outcomes = outcomes
        self.feedback_message = f"{self.mission} = {self.outcome}"
        return Status.SUCCESS


class StopRunning(RosBehaviour):
    """Disarm the tree at the end of a flight plan, then return SUCCESS.

    *** Without this, a completed flight plan takes off again. ***

    The tree ticks forever, and py_trees restarts a subtree whose previous
    status was not RUNNING (see docs/TREE_SEMANTICS.md). So a FlightA that
    reaches its landing returns SUCCESS, and on the very next tick the root
    Selector re-enters it from the top -- straight back into ArmAndTakeoff.
    On the bench that just looks like a loop. On the aircraft it is an
    unattended second takeoff moments after landing.

    Clearing ``running`` makes the FlightPlanIs guards fail, so the root falls
    through to Idle and stays there. The operator re-arms with
    ``~/set_running`` for another attempt -- which is the right workflow anyway,
    since the 30-minute competition slot (rulebook 2.2) allows several.

    Put it last in every flight plan.
    """

    requires_node = False

    def __init__(self, name: str = "StopRunning"):
        super().__init__(name)
        self._bb = bb.client(f"{name}:stop", write=["running"])

    def update(self) -> Status:
        self._bb.running = False
        self.feedback_message = "flight complete; disarmed, waiting to be re-armed"
        self.info("flight plan complete — " + summary())
        return Status.SUCCESS


def summary(prefix: Optional[str] = None) -> str:
    """Render the current outcomes and objectives as one log-friendly line.

    Args:
        prefix: filter objectives to those starting with this string
            (e.g. ``"m1."`` for mission 1 only). None returns everything.

    Returns:
        A single-line summary, e.g.
        ``"missions: mission1=success mission2=skipped | objectives: m1.tubes=True"``.
    """
    snap = bb.snapshot()
    outcomes = snap.get("mission_outcomes") or {}
    objectives = snap.get("objectives") or {}
    if prefix:
        objectives = {k: v for k, v in objectives.items() if k.startswith(prefix)}

    missions_text = " ".join(f"{k}={v}" for k, v in sorted(outcomes.items())) or "none"
    objectives_text = " ".join(f"{k}={v}" for k, v in sorted(objectives.items())) or "none"
    return f"missions: {missions_text} | objectives: {objectives_text}"
