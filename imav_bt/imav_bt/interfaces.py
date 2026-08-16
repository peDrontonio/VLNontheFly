"""
interfaces.py — the external ROS contract, in one place.

Every topic name, service name and status string the tree exchanges with the
rest of the stack is declared here rather than sprinkled through behaviours.
Two reasons:

  1. There is exactly one file to edit when a topic gets renamed or namespaced.
  2. docs/INTERFACES.md is a prose rendering of this file, so the two can be
     checked against each other.

The goal_status contract
------------------------
ego-planner's FSM publishes std_msgs/String on `planning/goal_status`
(ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp:154,313).
This is a first-party addition to ego-planner, not upstream, and it is only
published when the planner runs in MANUAL_TARGET mode -- which is the mode the
tree drives it in. The full set of strings it can emit:

    accepted:safety_bubble   goal passed the goal-gate/odom/FSM-state checks
    accepted                 global trajectory successfully planned
    rejected:<reason>        goal gate or z-bound violation
    rejected:not_ready       no odometry, or FSM not in WAIT_TARGET/EXEC_TRAJ
    failed:plan              planGlobalTraj failed
    reached                  local target == global target and duration elapsed

*** IMPORTANT CAVEAT ***
There is NO replan-failure and NO collision status. If the planner accepts a
goal and then gets stuck -- trapped by newly-observed occupancy, replanning
forever, or simply not moving -- it publishes NOTHING further. `reached` may
never arrive and no failure ever appears.

Consequently a behaviour that waits on goal_status can wait forever. Every
goal-following leaf MUST be wrapped in a Deadline decorator. This is not
defensive style, it is the only way to observe that class of failure. See
behaviors/flight.py:GotoAnchor and decorators/policy.py:Deadline.
"""

from __future__ import annotations

# --------------------------------------------------------------------------
# Topics the tree publishes
# --------------------------------------------------------------------------

#: Absolute goal in the planner world frame (ENU, `map`). Consumed by
#: ego_replan_fsm's MANUAL_TARGET waypoint callback. Only position is used --
#: the planner ignores orientation (ego_replan_fsm.cpp:447).
TOPIC_GOAL = "/move_base_simple/goal"

#: Body-relative goal (FLU: +x forward, +y left, +z up), converted to an
#: absolute map goal by planner/src/relative_goal_to_map.py. Useful for
#: behaviours that reason in the drone frame instead of arena coordinates.
TOPIC_RELATIVE_GOAL = "/relative_goal"

#: Human-readable one-line tree state, for logging and the ground station.
TOPIC_BT_STATUS = "~/status"

# --------------------------------------------------------------------------
# Topics the tree subscribes to
# --------------------------------------------------------------------------

#: nav_msgs/Odometry in the ENU `map` frame, published by
#: planner/src/odometry_converter.py from /fmu/out/vehicle_odometry.
#: This is the ONLY pose source the tree reads, which is what makes the
#: localization backend (OptiTrack / OpenVINS / beacons) pluggable.
TOPIC_ODOM = "/odometry"

#: std_msgs/String goal feedback from ego-planner. See the module docstring.
TOPIC_GOAL_STATUS = "/planning/goal_status"

#: sensor_msgs/BatteryState, when available. The battery guard degrades to
#: "unknown -> do not trip" when nothing publishes here (see guards.py).
TOPIC_BATTERY = "/battery_status"

# --------------------------------------------------------------------------
# Services the tree offers
# --------------------------------------------------------------------------

#: std_srvs/SetBool -- true arms the selected flight plan, false returns to idle.
SRV_SET_RUNNING = "~/set_running"

#: std_srvs/Trigger -- operator abort. Latches; clears only on set_running(false).
SRV_ABORT = "~/abort"

# --------------------------------------------------------------------------
# Services the tree calls (payload hardware abstraction, scripts/payload_node.py)
# --------------------------------------------------------------------------

SRV_PAYLOAD_DROP_CONE = "/payload/drop_cone"
SRV_PAYLOAD_BLINK_LED = "/payload/blink_hotspot_led"
SRV_PAYLOAD_GRAB_RING = "/payload/grab_ring"
SRV_PAYLOAD_RELEASE_RING = "/payload/release_ring"
SRV_PAYLOAD_CONTINUITY = "/payload/read_continuity"


class GoalStatus:
    """The exact strings ego-planner emits on :data:`TOPIC_GOAL_STATUS`."""

    ACCEPTED = "accepted"
    ACCEPTED_SAFETY_BUBBLE = "accepted:safety_bubble"
    REACHED = "reached"
    REJECTED_PREFIX = "rejected:"
    FAILED_PREFIX = "failed:"

    @staticmethod
    def is_accepted(status: str) -> bool:
        """True while the planner is working on the goal (keep waiting)."""
        s = (status or "").strip().lower()
        return s == GoalStatus.ACCEPTED or s == GoalStatus.ACCEPTED_SAFETY_BUBBLE

    @staticmethod
    def is_reached(status: str) -> bool:
        """True when the planner reports the goal was reached."""
        return (status or "").strip().lower() == GoalStatus.REACHED

    @staticmethod
    def is_failure(status: str) -> bool:
        """True for any terminal rejection or planning failure."""
        s = (status or "").strip().lower()
        return s.startswith(GoalStatus.REJECTED_PREFIX) or s.startswith(
            GoalStatus.FAILED_PREFIX
        )

    @staticmethod
    def is_terminal(status: str) -> bool:
        """True when the planner will publish nothing further for this goal."""
        return GoalStatus.is_reached(status) or GoalStatus.is_failure(status)
