#!/usr/bin/env python3
"""Gate strict VLM navigation primitives into body-relative waypoint goals.

Goals are published as geometry_msgs/PoseStamped on `/relative_goal` whose
position is a base_link FLU offset (+x forward, +y left, +z up), not an absolute
world pose. `relative_goal_to_map` composes that offset with the current
odometry into the map frame for ego-planner, so the world-frame math lives in
exactly one place instead of being duplicated (and hard-coded) here.
"""

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


TRANSLATION_PRIMITIVES = {
    "FORWARD",
    "BACK",
    "LEFT",
    "RIGHT",
    "UP",
    "DOWN",
}
ALL_PRIMITIVES = TRANSLATION_PRIMITIVES | {"HOLD"}


@dataclass(frozen=True)
class PrimitiveProposal:
    primitive: str
    distance_m: float
    confidence: float
    reason: str


@dataclass(frozen=True)
class GateDecision:
    accepted: bool
    reason: str
    proposal: Optional[PrimitiveProposal] = None


def parse_primitive_object(candidate: Dict[str, Any]) -> PrimitiveProposal:
    primitive = str(candidate.get("primitive", "")).strip().upper()
    if primitive not in ALL_PRIMITIVES:
        raise ValueError(f"invalid primitive: {primitive}")

    # HOLD never translates. Small models sometimes contradict themselves and
    # emit HOLD with a non-zero distance_m; normalize it so a valid HOLD is not
    # rejected downstream for "HOLD distance must be zero".
    if primitive == "HOLD":
        distance = 0.0
    else:
        distance = float(candidate.get("distance_m", 0.0))
    confidence = float(candidate.get("confidence", 0.0))
    reason = str(candidate.get("reason", "")).strip()
    return PrimitiveProposal(
        primitive=primitive,
        distance_m=distance,
        confidence=confidence,
        reason=reason,
    )


def parse_safe_truncated_hold(text: str) -> Optional[PrimitiveProposal]:
    compact = "".join(text.split())
    # A truncated HOLD is always safe to recover (HOLD ignores distance), but a
    # truncated translation command is not, so only accept when HOLD is present
    # and no translation primitive appears.
    if '"primitive":"HOLD"' not in compact:
        return None
    if any(f'"primitive":"{primitive}"' in compact for primitive in TRANSLATION_PRIMITIVES):
        return None
    return PrimitiveProposal(
        primitive="HOLD",
        distance_m=0.0,
        confidence=1.0,
        reason="truncated hold",
    )


def parse_vlm_result(payload: str) -> PrimitiveProposal:
    """Parse the VLM node result and require strict JSON in the result text."""
    outer = json.loads(payload)
    candidate: Any
    if isinstance(outer, dict) and "text" in outer:
        text = outer["text"]
        try:
            candidate = json.loads(text)
        except json.JSONDecodeError:
            safe_hold = parse_safe_truncated_hold(text)
            if safe_hold is not None:
                return safe_hold
            raise
    else:
        candidate = outer

    if not isinstance(candidate, dict):
        raise ValueError("primitive payload must be a JSON object")

    return parse_primitive_object(candidate)


def validate_proposal(
    proposal: PrimitiveProposal,
    min_confidence: float,
    max_translation_m: float,
    allowed_distances: Iterable[float],
) -> GateDecision:
    if not 0.0 <= proposal.confidence <= 1.0:
        return GateDecision(False, "confidence outside 0..1", proposal)
    if proposal.confidence < min_confidence:
        return GateDecision(False, "confidence below threshold", proposal)

    if proposal.primitive == "HOLD":
        if abs(proposal.distance_m) > 1e-6:
            return GateDecision(False, "HOLD distance must be zero", proposal)
        return GateDecision(True, "hold", proposal)

    if proposal.distance_m <= 0.0:
        return GateDecision(False, "translation distance must be positive", proposal)
    if proposal.distance_m > max_translation_m:
        return GateDecision(False, "translation distance exceeds limit", proposal)

    allowed = list(allowed_distances)
    if allowed and not any(math.isclose(proposal.distance_m, value, abs_tol=1e-3) for value in allowed):
        return GateDecision(False, "translation distance is not allowed", proposal)

    return GateDecision(True, "accepted", proposal)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw_rad
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def px4_local_position_to_pose(msg: VehicleLocalPosition) -> Pose:
    pose = Pose()
    pose.position.x = float(msg.y)
    pose.position.y = float(msg.x)
    pose.position.z = float(-msg.z)
    yaw_enu = math.pi / 2.0 - float(msg.heading)
    qx, qy, qz, qw = yaw_to_quaternion(yaw_enu)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def primitive_to_body_offset(proposal: PrimitiveProposal) -> Tuple[float, float, float]:
    """Map a primitive to a base_link FLU offset (forward, left, up) in meters.

    No yaw is applied: the offset is expressed in the robot body frame and the
    downstream relative-goal adapter rotates it into the map frame.
    """
    distance = proposal.distance_m
    if proposal.primitive == "FORWARD":
        return distance, 0.0, 0.0
    if proposal.primitive == "BACK":
        return -distance, 0.0, 0.0
    if proposal.primitive == "LEFT":
        return 0.0, distance, 0.0
    if proposal.primitive == "RIGHT":
        return 0.0, -distance, 0.0
    if proposal.primitive == "UP":
        return 0.0, 0.0, distance
    if proposal.primitive == "DOWN":
        return 0.0, 0.0, -distance
    return 0.0, 0.0, 0.0


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


class VlmPrimitiveGate(Node):
    def __init__(self) -> None:
        super().__init__("vlm_primitive_gate")

        self.vlm_result_topic = self.declare_parameter(
            "vlm_result_topic", "/edgellm_vlm_node/result").value
        self.odometry_topic = self.declare_parameter("odometry_topic", "").value
        self.pose_topic = self.declare_parameter("pose_topic", "").value
        self.vehicle_local_position_topic = self.declare_parameter(
            "vehicle_local_position_topic", "/fmu/out/vehicle_local_position").value
        self.goal_topic = self.declare_parameter("goal_topic", "/relative_goal").value
        self.goal_frame_id = self.declare_parameter("goal_frame_id", "base_link").value
        self.min_confidence = float(self.declare_parameter("min_confidence", 0.65).value)
        self.max_translation_m = float(self.declare_parameter("max_translation_m", 1.0).value)
        self.allowed_distances = [
            float(value) for value in self.declare_parameter("allowed_distances_m", [0.5, 1.0]).value
        ]
        self.cooldown_s = float(self.declare_parameter("cooldown_s", 3.0).value)
        self.max_result_age_s = float(self.declare_parameter("max_result_age_s", 2.5).value)
        self.max_odom_age_s = float(self.declare_parameter("max_odom_age_s", 1.0).value)
        self.min_goal_z_m = float(self.declare_parameter("min_goal_z_m", 0.0).value)
        self.max_goal_z_m = float(self.declare_parameter("max_goal_z_m", 3.0).value)

        qos = QoSProfile(depth=10)
        self.result_sub = self.create_subscription(
            String, self.vlm_result_topic, self._on_result, qos)
        self.odom_sub = None
        if self.odometry_topic:
            self.odom_sub = self.create_subscription(
                Odometry, self.odometry_topic, self._on_odom, qos)
        self.pose_sub = None
        if self.pose_topic:
            self.pose_sub = self.create_subscription(
                PoseStamped, self.pose_topic, self._on_pose, qos)
        self.local_position_sub = None
        if self.vehicle_local_position_topic:
            px4_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.local_position_sub = self.create_subscription(
                VehicleLocalPosition,
                self.vehicle_local_position_topic,
                self._on_vehicle_local_position,
                px4_qos,
            )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, qos)
        self.proposal_pub = self.create_publisher(String, "~/proposal", qos)
        self.status_pub = self.create_publisher(String, "~/status", qos)
        self.execute_srv = self.create_service(Trigger, "~/execute_next", self._on_execute_next)

        self.latest_pose: Optional[Pose] = None
        self.latest_pose_rx: Optional[rclpy.time.Time] = None
        self.latest_decision: Optional[GateDecision] = None
        self.latest_result_rx: Optional[rclpy.time.Time] = None
        self.last_goal_pub: Optional[rclpy.time.Time] = None

        self._publish_status(
            f"ready result_topic={self.vlm_result_topic} odom_topic={self.odometry_topic} "
            f"pose_topic={self.pose_topic} "
            f"vehicle_local_position_topic={self.vehicle_local_position_topic} "
            f"goal_topic={self.goal_topic}")

    def _now(self) -> rclpy.time.Time:
        return self.get_clock().now()

    def _seconds_since(self, stamp: Optional[rclpy.time.Time]) -> Optional[float]:
        if stamp is None:
            return None
        return (self._now() - stamp).nanoseconds / 1e9

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _publish_proposal(self, decision: GateDecision) -> None:
        payload: Dict[str, Any] = {
            "accepted": decision.accepted,
            "gate_reason": decision.reason,
        }
        if decision.proposal is not None:
            payload.update({
                "primitive": decision.proposal.primitive,
                "distance_m": decision.proposal.distance_m,
                "confidence": decision.proposal.confidence,
                "reason": decision.proposal.reason,
            })
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.proposal_pub.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_pose = msg.pose.pose
        self.latest_pose_rx = self._now()

    def _on_pose(self, msg: PoseStamped) -> None:
        self.latest_pose = msg.pose
        self.latest_pose_rx = self._now()

    def _on_vehicle_local_position(self, msg: VehicleLocalPosition) -> None:
        if not (msg.xy_valid and msg.z_valid and math.isfinite(msg.heading)):
            return
        self.latest_pose = px4_local_position_to_pose(msg)
        self.latest_pose_rx = self._now()

    def _on_result(self, msg: String) -> None:
        self.latest_result_rx = self._now()
        try:
            proposal = parse_vlm_result(msg.data)
            decision = validate_proposal(
                proposal,
                self.min_confidence,
                self.max_translation_m,
                self.allowed_distances,
            )
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            decision = GateDecision(False, f"parse_error:{exc}")

        self.latest_decision = decision
        self._publish_proposal(decision)
        status = "proposal:accepted" if decision.accepted else f"proposal:rejected:{decision.reason}"
        self._publish_status(status)

    def _on_execute_next(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        ok, reason = self._can_publish_goal()
        if not ok:
            response.success = False
            response.message = reason
            self._publish_status(f"execute:rejected:{reason}")
            return response

        assert self.latest_decision is not None
        assert self.latest_decision.proposal is not None
        proposal = self.latest_decision.proposal

        if proposal.primitive == "HOLD":
            response.success = True
            response.message = "HOLD accepted; no goal published"
            self._publish_status("execute:hold")
            return response

        assert self.latest_pose is not None
        goal = self._build_goal(proposal, self.latest_pose)
        self.goal_pub.publish(goal)
        self.last_goal_pub = self._now()

        response.success = True
        response.message = (
            f"published {proposal.primitive} {proposal.distance_m:.2f}m {self.goal_frame_id} "
            f"offset (fwd={goal.pose.position.x:.2f}, left={goal.pose.position.y:.2f}, "
            f"up={goal.pose.position.z:.2f})")
        self._publish_status(f"execute:published:{proposal.primitive}")
        return response

    def _can_publish_goal(self) -> Tuple[bool, str]:
        if self.latest_decision is None:
            return False, "no VLM proposal received"
        if not self.latest_decision.accepted:
            return False, self.latest_decision.reason
        if self.latest_decision.proposal is None:
            return False, "accepted decision has no proposal"
        if self.latest_pose is None:
            return False, "no pose received"

        result_age = self._seconds_since(self.latest_result_rx)
        if result_age is None or result_age > self.max_result_age_s:
            return False, "VLM proposal is stale"

        pose_age = self._seconds_since(self.latest_pose_rx)
        if pose_age is None or pose_age > self.max_odom_age_s:
            return False, "pose is stale"

        cooldown_age = self._seconds_since(self.last_goal_pub)
        if cooldown_age is not None and cooldown_age < self.cooldown_s:
            return False, "cooldown active"

        return True, "accepted"

    def _build_goal(self, proposal: PrimitiveProposal, pose: Pose) -> PoseStamped:
        forward, left, up = primitive_to_body_offset(proposal)

        goal = PoseStamped()
        goal.header.stamp = self._now().to_msg()
        goal.header.frame_id = self.goal_frame_id
        goal.pose.position.x = forward
        goal.pose.position.y = left
        # Enforce the absolute z safety envelope against the current altitude,
        # then re-express the result as a base_link-relative up offset so the
        # relative-goal adapter still composes it correctly.
        target_abs_z = clamp(
            pose.position.z + up,
            self.min_goal_z_m,
            self.max_goal_z_m,
        )
        goal.pose.position.z = target_abs_z - pose.position.z
        goal.pose.orientation.w = 1.0
        return goal


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VlmPrimitiveGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
