"""
guards.py — conditions that gate the tree.

Two families:

    Safety    Is something wrong right now? Drives the highest-priority branch
              of the root, which preempts whatever the mission was doing.
    Selection Which flight plan should be running? Cheap blackboard reads used
              by the flight-plan selector.

Design decisions worth knowing before you edit these
----------------------------------------------------

*A missing sensor must not ground the drone.* The battery guard does nothing
until a battery message has actually arrived (``battery_valid``). If the topic
is unmapped on a given airframe, the guard stays quiet rather than tripping on
a default value of 0.0 and refusing to fly. The localization guard takes the
opposite stance -- no pose means the planner cannot work at all, so absence IS
the emergency. This asymmetry is deliberate; see docs/ARCHITECTURE.md.

*Staleness is measured on receipt time, not on message headers.* A publisher
that wedges while still stamping fresh headers is a real failure mode, and
comparing header stamps to ROS time would miss it. bt_engine records
``odom_stamp`` from the monotonic clock at receipt.

*One reason string, not five booleans.* SafetyTripped evaluates every check and
reports the first thing it finds, writing it to the ``safety_reason``
blackboard key. That string is what gets logged and shown in the tree watcher,
so make it specific enough to act on: "battery 0.18 below floor 0.25" rather
than "battery".
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from imav_bt import blackboard as bb
from imav_bt.behaviors.base import Condition


@dataclass(frozen=True)
class SafetyLimits:
    """Thresholds for :class:`SafetyTripped`. Loaded from config/bt.yaml.

    Attributes:
        min_battery: state of charge below which the flight is aborted, in
            [0, 1]. Only enforced once a battery message has been seen.
        max_odom_age_s: how stale /odometry may get before pose is considered
            lost. The planner itself refuses goals without odometry
            (``rejected:not_ready``), so this mainly catches the case where
            pose dies mid-trajectory.
        geofence: (xmin, xmax, ymin, ymax, zmin, zmax) in the ENU `map` frame,
            or None to disable. The IMAV indoor arena is 14 x 7 x 4 m
            (rulebook 4.2), but the useful bound is your own flight volume
            inside it, not the cage.
        require_odom: when True, never having received odometry is an emergency.
    """

    min_battery: float = 0.25
    max_odom_age_s: float = 1.0
    geofence: Optional[Tuple[float, float, float, float, float, float]] = None
    require_odom: bool = True


class SafetyTripped(Condition):
    """SUCCESS when something is wrong, FAILURE when everything is nominal.

    The inverted-looking polarity is intentional: this sits at the head of the
    root's safety Sequence, so SUCCESS means "yes, there is an emergency,
    proceed to the recovery action".

    Also writes the ``safety_reason`` blackboard key -- empty when nominal --
    so the recovery behaviours and the status topic can report *why* without
    re-running the checks.

    Args:
        name: behaviour name.
        limits: thresholds to enforce.
        time_source: injectable monotonic clock, for tests.
    """

    def __init__(
        self,
        name: str = "SafetyTripped",
        limits: Optional[SafetyLimits] = None,
        time_source: Callable[[], float] = time.monotonic,
    ):
        super().__init__(name)
        self.limits = limits or SafetyLimits()
        self._now = time_source
        self._bb = bb.client(
            f"{name}:safety",
            read=[
                "abort_requested",
                "battery_valid",
                "battery_fraction",
                "odom_valid",
                "odom_stamp",
                "position",
                "running",
            ],
            write=["safety_reason"],
        )

    def _first_problem(self) -> str:
        """Return the first problem found, or "" when nominal."""
        if self._bb.abort_requested:
            return "operator abort requested"

        # Battery: silent until a real reading exists, then hard.
        if self._bb.battery_valid and self._bb.battery_fraction < self.limits.min_battery:
            return (
                f"battery {self._bb.battery_fraction:.2f} below floor "
                f"{self.limits.min_battery:.2f}"
            )

        # Localization: absence IS the emergency, unlike the battery.
        if self.limits.require_odom:
            if not self._bb.odom_valid:
                return "no odometry received"
            age = self._now() - self._bb.odom_stamp
            if age > self.limits.max_odom_age_s:
                return (
                    f"odometry stale by {age:.2f}s "
                    f"(limit {self.limits.max_odom_age_s:.2f}s)"
                )

        if self.limits.geofence is not None and self._bb.odom_valid:
            x, y, z = self._bb.position
            xmin, xmax, ymin, ymax, zmin, zmax = self.limits.geofence
            if not (xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax):
                return (
                    f"outside geofence at ({x:.2f}, {y:.2f}, {z:.2f}); "
                    f"bounds x[{xmin}, {xmax}] y[{ymin}, {ymax}] z[{zmin}, {zmax}]"
                )

        return ""

    def check(self) -> Tuple[bool, str]:
        # Safety checks only apply once the operator has armed the tree. While
        # idle the drone is on the ground with no odometry, which would
        # otherwise read as a permanent emergency.
        if not self._bb.running:
            self._bb.safety_reason = ""
            return False, "idle: safety checks not armed"

        problem = self._first_problem()
        self._bb.safety_reason = problem
        if problem:
            return True, f"EMERGENCY: {problem}"
        return False, "nominal"


class IsRunning(Condition):
    """SUCCESS while the operator has set the tree running."""

    def __init__(self, name: str = "IsRunning"):
        super().__init__(name)
        self._bb = bb.client(f"{name}:running", read=["running"])

    def check(self) -> Tuple[bool, str]:
        running = self._bb.running
        return running, "running" if running else "idle (waiting for set_running)"


class FlightPlanIs(Condition):
    """SUCCESS when the selected flight plan matches ``plan``.

    Args:
        name: behaviour name.
        plan: one of :data:`imav_bt.blackboard.FlightPlan.ALL`.

    Raises:
        ValueError: if ``plan`` is not a known flight plan.
    """

    def __init__(self, name: str, plan: str):
        super().__init__(name)
        if plan not in bb.FlightPlan.ALL:
            raise ValueError(
                f"{name}: unknown flight plan {plan!r}; "
                f"expected one of {bb.FlightPlan.ALL}"
            )
        self.plan = plan
        self._bb = bb.client(f"{name}:plan", read=["flight_plan", "running"])

    def check(self) -> Tuple[bool, str]:
        if not self._bb.running:
            return False, "idle"
        selected = self._bb.flight_plan
        if selected == self.plan:
            return True, f"flight plan is {self.plan}"
        return False, f"flight plan is {selected}, not {self.plan}"
