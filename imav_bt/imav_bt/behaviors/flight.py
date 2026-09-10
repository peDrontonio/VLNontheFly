"""
flight.py — behaviours that move the drone.

GotoAnchor is the only one implemented for real. Everything else here
(ArmAndTakeoff, Hover, Land) is a stub, because those depend on airframe
decisions that are still open. GotoAnchor is different: it encodes the
ego-planner goal handshake, which is subtle enough that getting it wrong would
be discovered on the flight line rather than on the bench.


The goal handshake, and the trap in it
--------------------------------------
Publishing a goal to ego-planner is fire-and-forget; feedback comes back
asynchronously on `planning/goal_status` (see interfaces.py). The obvious
implementation -- publish, then wait for "reached" -- has two bugs.

*Bug 1: stale status.* The status topic still holds the result of the PREVIOUS
goal at the moment you publish the next one. A naive leaf reads "reached" left
over from the last waypoint and instantly reports success without having moved.
GotoAnchor defends against this by recording the publish time and ignoring
every status whose receipt timestamp predates it.

*Bug 2: silence.* ego-planner publishes no replan-failure and no collision
status. If it accepts a goal and then gets stuck, nothing further is ever
published -- "reached" never arrives and no failure appears either. A leaf that
simply waits will return RUNNING forever and the flight ends with the drone
hovering until the battery runs out.

Bug 2 cannot be fixed inside this leaf, because the information genuinely does
not exist. It is handled one level up, by wrapping every GotoAnchor in a
Deadline decorator. **A bare GotoAnchor is always a mistake.** Use the helper
:func:`goto` rather than constructing one directly, and it is wrapped for you.

Optionally, ``arrival_tolerance_m`` gives a second, independent way to conclude
the goal was met: the drone is physically within tolerance of the target. This
is off by default because the planner is the authority on whether it considers
a goal complete, but it is useful on a stack where "reached" is known to be
unreliable.
"""

from __future__ import annotations

import math
import time
from typing import Callable, Dict, Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped
from py_trees.common import Status
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from imav_bt import blackboard as bb
from imav_bt import interfaces
from imav_bt.behaviors.base import RosBehaviour
from imav_bt.behaviors.stubs import Stub
from imav_bt.decorators.policy import AttemptBudget, Deadline

Vec3 = Tuple[float, float, float]


def resolve_anchor(name: str, anchors: Dict[str, Sequence[float]]) -> Vec3:
    """Look up a named arena anchor and return it as an (x, y, z) tuple.

    Anchors are named positions in the ENU `map` frame, defined in
    config/arena.yaml. Behaviours refer to anchors rather than raw coordinates
    so that re-surveying the arena -- or swapping the localization backend
    entirely -- is a config change and never a code change.

    Args:
        name: anchor name, e.g. ``"takeoff_pad"``.
        anchors: mapping of anchor name to a 3-element sequence.

    Returns:
        The anchor position as a tuple of floats.

    Raises:
        KeyError: if the anchor is not defined.
        ValueError: if the anchor is not 3 numbers.
    """
    try:
        raw = anchors[name]
    except KeyError:
        known = ", ".join(sorted(anchors)) or "<none defined>"
        raise KeyError(
            f"anchor {name!r} is not defined in config/arena.yaml. Known anchors: {known}"
        ) from None
    if len(raw) != 3:
        raise ValueError(f"anchor {name!r} must be [x, y, z], got {raw!r}")
    return (float(raw[0]), float(raw[1]), float(raw[2]))


class GotoAnchor(RosBehaviour):
    """Send the drone to a position and wait for ego-planner to report arrival.

    Publishes one ``geometry_msgs/PoseStamped`` on
    :data:`~imav_bt.interfaces.TOPIC_GOAL` when the behaviour starts, then
    interprets ``planning/goal_status``:

    ==========================  ======================================
    goal_status                 result
    ==========================  ======================================
    ``reached``                 SUCCESS
    ``rejected:*``              FAILURE
    ``failed:*``                FAILURE
    ``accepted``/``accepted:*`` RUNNING (planner is working on it)
    nothing new yet             RUNNING
    ==========================  ======================================

    Statuses received before this behaviour published are ignored -- they
    describe the previous goal.

    Args:
        name: behaviour name.
        position: explicit (x, y, z) in the ENU `map` frame. Mutually exclusive
            with ``anchor``.
        anchor: name of an anchor in ``anchors``. Mutually exclusive with
            ``position``.
        anchors: anchor table, normally loaded from config/arena.yaml.
        frame_id: goal frame. Must match the planner world frame.
        arrival_tolerance_m: if set, also succeed when the drone is within this
            distance of the target, without waiting for ``reached``. None
            (default) trusts the planner alone.
        time_source: injectable monotonic clock, for tests.

    Raises:
        ValueError: if neither or both of ``position`` and ``anchor`` are given.

    Warning:
        Always wrap this in a Deadline. Use :func:`goto`.
    """

    def __init__(
        self,
        name: str,
        position: Optional[Vec3] = None,
        anchor: Optional[str] = None,
        anchors: Optional[Dict[str, Sequence[float]]] = None,
        frame_id: str = "map",
        arrival_tolerance_m: Optional[float] = None,
        time_source: Callable[[], float] = time.monotonic,
    ):
        super().__init__(name)
        if (position is None) == (anchor is None):
            raise ValueError(
                f"{name}: pass exactly one of position=(x, y, z) or anchor='name'"
            )
        self.anchor = anchor
        self._anchors = dict(anchors or {})
        self._explicit_position = (
            (float(position[0]), float(position[1]), float(position[2]))
            if position is not None
            else None
        )
        self.frame_id = frame_id
        self.arrival_tolerance_m = arrival_tolerance_m
        self._now = time_source

        self.target: Optional[Vec3] = None
        self._published_at = 0.0
        self._publisher = None

        self._bb = bb.client(
            f"{name}:goto",
            read=["goal_status", "goal_status_stamp", "position", "odom_valid"],
            write=["active_goal"],
        )

    # -- lifecycle -------------------------------------------------------

    def setup_ros(self, node) -> None:
        # Depth 1, reliable, volatile: the planner wants the newest goal and
        # nothing else. TRANSIENT_LOCAL would replay an old goal to a planner
        # that restarts mid-flight, which is the opposite of what we want.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = node.create_publisher(PoseStamped, interfaces.TOPIC_GOAL, qos)

    def initialise(self) -> None:
        """Resolve the target and publish it exactly once."""
        self.target = (
            self._explicit_position
            if self._explicit_position is not None
            else resolve_anchor(self.anchor, self._anchors)
        )

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        if self.node is not None:
            msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.target
        # ego-planner ignores goal orientation (ego_replan_fsm.cpp:447); send a
        # valid identity quaternion anyway so the message is well-formed for
        # anything else listening (RViz, rosbag analysis).
        msg.pose.orientation.w = 1.0

        # Record the publish time BEFORE publishing. Any status stamped earlier
        # belongs to a previous goal and must be ignored (see module docstring).
        self._published_at = self._now()
        if self._publisher is not None:
            self._publisher.publish(msg)

        self._bb.active_goal = self.target
        label = self.anchor or "position"
        self.info(
            f"goal -> {label} "
            f"({self.target[0]:.2f}, {self.target[1]:.2f}, {self.target[2]:.2f}) "
            f"[{self.frame_id}]"
        )

    def update(self) -> Status:
        if self.target is None:  # pragma: no cover - initialise always sets it
            self.feedback_message = "no target resolved"
            return Status.FAILURE

        # Independent arrival check, when enabled.
        if self.arrival_tolerance_m is not None and self._bb.odom_valid:
            distance = self._distance_to_target()
            if distance <= self.arrival_tolerance_m:
                self.feedback_message = f"arrived within {distance:.2f}m of target"
                return Status.SUCCESS

        status = self._fresh_status()
        if status is None:
            self.feedback_message = "goal sent, awaiting planner"
            return Status.RUNNING

        if interfaces.GoalStatus.is_reached(status):
            self.feedback_message = "planner reports reached"
            return Status.SUCCESS

        if interfaces.GoalStatus.is_failure(status):
            self.feedback_message = f"planner rejected the goal: {status}"
            return Status.FAILURE

        if interfaces.GoalStatus.is_accepted(status):
            self.feedback_message = f"planner working ({status})"
            return Status.RUNNING

        # Unknown string: ego-planner grew a new status. Keep waiting rather
        # than guessing, and make the surprise visible in the logs.
        self.warn(f"unrecognised goal_status {status!r}; continuing to wait")
        self.feedback_message = f"unrecognised status {status!r}"
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        """Release goal ownership so a stale target cannot mislead the next leaf."""
        if new_status != Status.RUNNING:
            self._bb.active_goal = None

    # -- helpers ---------------------------------------------------------

    def _fresh_status(self) -> Optional[str]:
        """The latest goal_status, but only if it arrived after we published."""
        if self._bb.goal_status_stamp < self._published_at:
            return None
        status = self._bb.goal_status
        return status or None

    def _distance_to_target(self) -> float:
        x, y, z = self._bb.position
        tx, ty, tz = self.target
        return math.sqrt((x - tx) ** 2 + (y - ty) ** 2 + (z - tz) ** 2)


def goto(
    name: str,
    *,
    deadline: float,
    attempts: int = 1,
    time_source: Callable[[], float] = time.monotonic,
    **kwargs,
):
    """Build a GotoAnchor already wrapped in the decorators it requires.

    Prefer this over constructing :class:`GotoAnchor` directly. ego-planner can
    accept a goal and then go silent forever (see the module docstring), so an
    unbounded GotoAnchor is a hang waiting to happen.

    Args:
        name: behaviour name.
        deadline: seconds to allow per attempt before giving up.
        attempts: how many times to re-send the goal before failing.
        time_source: injectable clock, forwarded to both layers.
        **kwargs: forwarded to :class:`GotoAnchor` (``anchor``/``position``,
            ``anchors``, ``frame_id``, ``arrival_tolerance_m``).

    Returns:
        The decorated behaviour, ready to add to a composite.

    Example:
        >>> goto("Go to room", anchor="room_face", anchors=arena,
        ...      deadline=25.0, attempts=2)
    """
    leaf = GotoAnchor(name=name, time_source=time_source, **kwargs)
    node = Deadline(
        name=f"{name} Deadline", child=leaf, duration=deadline, time_source=time_source
    )
    if attempts > 1:
        node = AttemptBudget(name=f"{name} Attempts", child=node, attempts=attempts)
    return node


# ---------------------------------------------------------------------------
# Stubs: implement these when the airframe interface is settled.
# ---------------------------------------------------------------------------
#
# ArmAndTakeoff is deliberately NOT implemented. On this stack, takeoff is a
# manual step in the documented flight protocol (see
# planner_wrapper/launch/ego_raptor.launch.py): arm, take off by hand, enable
# Raptor via the RC switch, and only then start the planner. Automating it
# means committing to an arming path -- PX4 VehicleCommand, or mobile_msgs'
# Takeoff service -- and that decision has safety consequences beyond this
# package. Until then the tree assumes it is started in the air.


def arm_and_takeoff(altitude_m: float = 1.2, ticks: int = 3) -> Stub:
    """Placeholder for the takeoff sequence.

    Args:
        altitude_m: intended takeoff altitude, recorded in the note only.
        ticks: how long the stub pretends to work, to keep bench runs legible.
    """
    return Stub(
        "ArmAndTakeoff",
        outcome="success",
        ticks=ticks,
        note=(
            f"arm, take off to {altitude_m:.1f}m and hold. Currently manual: "
            f"arm and take off by hand, enable Raptor on the RC switch, then "
            f"set the tree running."
        ),
    )


def hover(seconds: float = 2.0, ticks: int = 2) -> Stub:
    """Placeholder for holding position for a fixed time."""
    return Stub(
        "Hover",
        outcome="success",
        ticks=ticks,
        note=f"hold position for {seconds:.1f}s (stop sending goals and let the planner settle)",
    )


def land(ticks: int = 3) -> Stub:
    """Placeholder for a plain descent-and-disarm, distinct from precision landing."""
    return Stub(
        "Land",
        outcome="success",
        ticks=ticks,
        note="descend and disarm at the current position (no marker tracking)",
    )
