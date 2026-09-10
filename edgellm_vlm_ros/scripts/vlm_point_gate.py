#!/usr/bin/env python3
"""Gate strict VLM image-point proposals into body-relative waypoint goals.

The selected pixel is deprojected with depth into a base_link FLU offset
(+x forward, +y left) and published as a geometry_msgs/PoseStamped on
`/relative_goal`. `relative_goal_to_map` composes that offset with the current
odometry into the map frame for ego-planner, so this node never needs the
world-frame yaw rotation or pose addition it used to hard-code.
"""

import json
import math
import re
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class PointProposal:
    u: float
    v: float
    confidence: float
    reason: str
    image_width: Optional[int] = None
    image_height: Optional[int] = None


@dataclass(frozen=True)
class GateDecision:
    accepted: bool
    reason: str
    proposal: Optional[PointProposal] = None


def _scalar_pixel(value: Any, index: int) -> float:
    """Coerce a pixel field to a scalar.

    Some VLM outputs place the full [x, y] pair into both the u and v keys
    instead of separate scalars (e.g. {"u": [500, 370], "v": [500, 370]}).
    Recover the intended axis by index: u -> index 0, v -> index 1.
    """
    if isinstance(value, (list, tuple)):
        if not value:
            raise ValueError("empty pixel list")
        if len(value) == 1:
            return float(value[0])
        return float(value[index])
    return float(value)


def parse_point_object(candidate: Dict[str, Any], outer: Optional[Dict[str, Any]] = None) -> PointProposal:
    if "u" in candidate:
        u = _scalar_pixel(candidate["u"], 0)
    elif "x" in candidate:
        u = _scalar_pixel(candidate["x"], 0)
    elif "pixel_x" in candidate:
        u = _scalar_pixel(candidate["pixel_x"], 0)
    else:
        raise ValueError("missing u/x/pixel_x")

    if "v" in candidate:
        v = _scalar_pixel(candidate["v"], 1)
    elif "y" in candidate:
        v = _scalar_pixel(candidate["y"], 1)
    elif "pixel_y" in candidate:
        v = _scalar_pixel(candidate["pixel_y"], 1)
    else:
        raise ValueError("missing v/y/pixel_y")

    confidence = float(candidate.get("confidence", 0.0))
    reason = str(candidate.get("reason", "")).strip()
    image_width = candidate.get("image_width")
    image_height = candidate.get("image_height")
    if outer is not None:
        image_width = image_width if image_width is not None else outer.get("image_width")
        image_height = image_height if image_height is not None else outer.get("image_height")

    return PointProposal(
        u=u,
        v=v,
        confidence=confidence,
        reason=reason,
        image_width=int(image_width) if image_width is not None else None,
        image_height=int(image_height) if image_height is not None else None,
    )


def _loads_lenient(text: str) -> Any:
    """Parse model JSON, repairing truncated/unterminated output.

    Small VLMs sometimes emit invalid JSON, e.g. a runaway confidence like
    0.999999999999999 that overruns the token budget so the closing brace is
    never produced. Fall back to extracting the numeric fields by regex.
    """
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        def find(keys: Tuple[str, ...]) -> Optional[str]:
            for key in keys:
                match = re.search(r'"%s"\s*:\s*\[?\s*(-?\d+(?:\.\d+)?)' % key, text)
                if match is not None:
                    return match.group(1)
            return None

        u = find(("u", "x", "pixel_x"))
        v = find(("v", "y", "pixel_y"))
        if u is None or v is None:
            raise
        recovered: Dict[str, Any] = {"u": float(u), "v": float(v)}
        confidence = find(("confidence", "conf", "score"))
        if confidence is not None:
            recovered["confidence"] = min(1.0, float(confidence))
        return recovered


def parse_vlm_result(payload: str) -> PointProposal:
    """Parse the VLM node result, tolerating truncated JSON in the result text."""
    outer = json.loads(payload)
    candidate: Any
    if isinstance(outer, dict) and "text" in outer:
        candidate = _loads_lenient(outer["text"])
    else:
        candidate = outer
        outer = None

    if not isinstance(candidate, dict):
        raise ValueError("point payload must be a JSON object")

    return parse_point_object(candidate, outer)


def validate_proposal(
    proposal: PointProposal,
    min_confidence: float,
    source_width: int,
    source_height: int,
) -> GateDecision:
    if not 0.0 <= proposal.confidence <= 1.0:
        return GateDecision(False, "confidence outside 0..1", proposal)
    if proposal.confidence < min_confidence:
        return GateDecision(False, "confidence below threshold", proposal)
    if source_width <= 0 or source_height <= 0:
        return GateDecision(False, "source image size is unknown", proposal)
    if not 0.0 <= proposal.u < source_width or not 0.0 <= proposal.v < source_height:
        return GateDecision(False, "pixel outside source image", proposal)
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


def depth_image_to_meters(msg: Image, depth_scale: float) -> np.ndarray:
    if msg.encoding in ("16UC1", "mono16"):
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        values = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=dtype,
            buffer=msg.data,
            strides=(msg.step, dtype.itemsize),
        ).astype(np.float32)
        return values * depth_scale
    if msg.encoding == "32FC1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        return np.ndarray(
            shape=(msg.height, msg.width),
            dtype=dtype,
            buffer=msg.data,
            strides=(msg.step, dtype.itemsize),
        ).astype(np.float32)
    raise ValueError(f"unsupported depth encoding: {msg.encoding}")


def scale_pixel(
    u: float,
    v: float,
    source_width: int,
    source_height: int,
    target_width: int,
    target_height: int,
) -> Tuple[float, float]:
    if source_width <= 0 or source_height <= 0:
        raise ValueError("source image size must be positive")
    return (
        u * float(target_width) / float(source_width),
        v * float(target_height) / float(source_height),
    )


def sample_depth_m(
    depth_m: np.ndarray,
    u: float,
    v: float,
    radius_px: int,
    min_depth_m: float,
    max_depth_m: float,
) -> float:
    height, width = depth_m.shape
    center_u = int(round(u))
    center_v = int(round(v))
    u0 = max(0, center_u - radius_px)
    u1 = min(width, center_u + radius_px + 1)
    v0 = max(0, center_v - radius_px)
    v1 = min(height, center_v + radius_px + 1)
    if u0 >= u1 or v0 >= v1:
        raise ValueError("scaled pixel outside depth image")

    window = depth_m[v0:v1, u0:u1]
    valid = window[np.isfinite(window) & (window >= min_depth_m) & (window <= max_depth_m)]
    if valid.size == 0:
        raise ValueError("no valid depth near selected pixel")
    return float(np.median(valid))


def deproject_pixel_to_optical(
    u: float,
    v: float,
    depth_m: float,
    camera_info: CameraInfo,
) -> Tuple[float, float, float]:
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("camera intrinsics are invalid")
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return x, y, depth_m


def optical_to_body_horizontal(point_optical: Tuple[float, float, float]) -> Tuple[float, float]:
    optical_x, _optical_y, optical_z = point_optical
    body_x = optical_z
    body_y = -optical_x
    return body_x, body_y


def clamp_xy_distance(dx: float, dy: float, max_distance_m: float) -> Tuple[float, float]:
    distance = math.hypot(dx, dy)
    if max_distance_m <= 0.0 or distance <= max_distance_m:
        return dx, dy
    scale = max_distance_m / distance
    return dx * scale, dy * scale


def resolve_goal_z(mode: str, fixed_goal_z_m: float, pose_z_m: float) -> float:
    return fixed_goal_z_m if mode == "fixed" else pose_z_m


class VlmPointGate(Node):
    def __init__(self) -> None:
        super().__init__("vlm_point_gate")

        self.vlm_result_topic = self.declare_parameter(
            "vlm_result_topic", "/edgellm_vlm_node/result").value
        self.depth_topic = self.declare_parameter(
            "depth_topic", "/camera/camera/depth/image_rect_raw").value
        self.depth_camera_info_topic = self.declare_parameter(
            "depth_camera_info_topic", "/camera/camera/depth/camera_info").value
        self.color_camera_info_topic = self.declare_parameter(
            "color_camera_info_topic", "/camera/camera/color/camera_info").value
        self.odometry_topic = self.declare_parameter("odometry_topic", "").value
        self.pose_topic = self.declare_parameter("pose_topic", "").value
        self.vehicle_local_position_topic = self.declare_parameter(
            "vehicle_local_position_topic", "/fmu/out/vehicle_local_position").value
        self.goal_topic = self.declare_parameter("goal_topic", "/relative_goal").value
        self.goal_frame_id = self.declare_parameter("goal_frame_id", "base_link").value
        self.min_confidence = float(self.declare_parameter("min_confidence", 0.60).value)
        self.depth_scale_m = float(self.declare_parameter("depth_scale_m", 0.001).value)
        self.depth_sample_radius_px = int(self.declare_parameter("depth_sample_radius_px", 4).value)
        self.min_depth_m = float(self.declare_parameter("min_depth_m", 0.25).value)
        self.max_depth_m = float(self.declare_parameter("max_depth_m", 4.0).value)
        self.max_goal_distance_m = float(self.declare_parameter("max_goal_distance_m", 1.5).value)
        self.goal_z_mode = self.declare_parameter("goal_z_mode", "current_pose").value
        self.fixed_goal_z_m = float(self.declare_parameter("fixed_goal_z_m", 1.0).value)
        self.cooldown_s = float(self.declare_parameter("cooldown_s", 2.0).value)
        self.max_result_age_s = float(self.declare_parameter("max_result_age_s", 2.5).value)
        self.max_depth_age_s = float(self.declare_parameter("max_depth_age_s", 0.5).value)
        self.max_pose_age_s = float(self.declare_parameter("max_pose_age_s", 1.0).value)

        qos = QoSProfile(depth=10)
        self.result_sub = self.create_subscription(
            String, self.vlm_result_topic, self._on_result, qos)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._on_depth, qos)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, self.depth_camera_info_topic, self._on_depth_info, qos)
        self.color_info_sub = self.create_subscription(
            CameraInfo, self.color_camera_info_topic, self._on_color_info, qos)
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
        self.latest_depth_msg: Optional[Image] = None
        self.latest_depth_rx: Optional[rclpy.time.Time] = None
        self.latest_depth_info: Optional[CameraInfo] = None
        self.latest_color_info: Optional[CameraInfo] = None
        self.latest_decision: Optional[GateDecision] = None
        self.latest_result_rx: Optional[rclpy.time.Time] = None
        self.last_goal_pub: Optional[rclpy.time.Time] = None

        self._publish_status(
            f"ready result_topic={self.vlm_result_topic} depth_topic={self.depth_topic} "
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
                "u": decision.proposal.u,
                "v": decision.proposal.v,
                "confidence": decision.proposal.confidence,
                "reason": decision.proposal.reason,
                "image_width": decision.proposal.image_width,
                "image_height": decision.proposal.image_height,
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

    def _on_depth(self, msg: Image) -> None:
        self.latest_depth_msg = msg
        self.latest_depth_rx = self._now()

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self.latest_depth_info = msg

    def _on_color_info(self, msg: CameraInfo) -> None:
        self.latest_color_info = msg

    def _source_size_for(self, proposal: PointProposal) -> Tuple[int, int]:
        if proposal.image_width is not None and proposal.image_height is not None:
            return proposal.image_width, proposal.image_height
        if self.latest_color_info is not None:
            return int(self.latest_color_info.width), int(self.latest_color_info.height)
        return 0, 0

    def _on_result(self, msg: String) -> None:
        self.latest_result_rx = self._now()
        try:
            proposal = parse_vlm_result(msg.data)
            source_width, source_height = self._source_size_for(proposal)
            decision = validate_proposal(
                proposal,
                self.min_confidence,
                source_width,
                source_height,
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
        assert self.latest_pose is not None
        assert self.latest_depth_msg is not None
        assert self.latest_depth_info is not None

        try:
            goal, details = self._build_goal(
                self.latest_decision.proposal,
                self.latest_pose,
                self.latest_depth_msg,
                self.latest_depth_info,
            )
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            self._publish_status(f"execute:rejected:{exc}")
            return response

        self.goal_pub.publish(goal)
        self.last_goal_pub = self._now()

        response.success = True
        response.message = (
            f"published point {self.goal_frame_id} offset "
            f"(fwd={goal.pose.position.x:.2f}, left={goal.pose.position.y:.2f}, "
            f"up={goal.pose.position.z:.2f}) depth={details['depth_m']:.2f}m")
        self._publish_status("execute:published:POINT")
        return response

    def _can_publish_goal(self) -> Tuple[bool, str]:
        if self.latest_decision is None:
            return False, "no VLM point proposal received"
        if not self.latest_decision.accepted:
            return False, self.latest_decision.reason
        if self.latest_decision.proposal is None:
            return False, "accepted decision has no proposal"
        if self.latest_pose is None:
            return False, "no pose received"
        if self.latest_depth_msg is None:
            return False, "no depth image received"
        if self.latest_depth_info is None:
            return False, "no depth camera info received"

        result_age = self._seconds_since(self.latest_result_rx)
        if result_age is None or result_age > self.max_result_age_s:
            return False, "VLM point proposal is stale"

        depth_age = self._seconds_since(self.latest_depth_rx)
        if depth_age is None or depth_age > self.max_depth_age_s:
            return False, "depth image is stale"

        pose_age = self._seconds_since(self.latest_pose_rx)
        if pose_age is None or pose_age > self.max_pose_age_s:
            return False, "pose is stale"

        cooldown_age = self._seconds_since(self.last_goal_pub)
        if cooldown_age is not None and cooldown_age < self.cooldown_s:
            return False, "cooldown active"

        return True, "accepted"

    def _build_goal(
        self,
        proposal: PointProposal,
        pose: Pose,
        depth_msg: Image,
        depth_info: CameraInfo,
    ) -> Tuple[PoseStamped, Dict[str, float]]:
        source_width, source_height = self._source_size_for(proposal)
        u_depth, v_depth = scale_pixel(
            proposal.u,
            proposal.v,
            source_width,
            source_height,
            depth_msg.width,
            depth_msg.height,
        )
        depth_m = sample_depth_m(
            depth_image_to_meters(depth_msg, self.depth_scale_m),
            u_depth,
            v_depth,
            self.depth_sample_radius_px,
            self.min_depth_m,
            self.max_depth_m,
        )
        point_optical = deproject_pixel_to_optical(u_depth, v_depth, depth_m, depth_info)
        dx_body, dy_body = optical_to_body_horizontal(point_optical)
        dx_body, dy_body = clamp_xy_distance(dx_body, dy_body, self.max_goal_distance_m)

        # Resolve the desired absolute target altitude, then re-express it as a
        # base_link-relative up offset so the relative-goal adapter composes it
        # with the live pose instead of this node baking in a world z.
        target_abs_z = resolve_goal_z(self.goal_z_mode, self.fixed_goal_z_m, pose.position.z)

        goal = PoseStamped()
        goal.header.stamp = self._now().to_msg()
        goal.header.frame_id = self.goal_frame_id
        goal.pose.position.x = dx_body
        goal.pose.position.y = dy_body
        goal.pose.position.z = target_abs_z - pose.position.z
        goal.pose.orientation.w = 1.0
        return goal, {
            "u_depth": u_depth,
            "v_depth": v_depth,
            "depth_m": depth_m,
            "dx_body": dx_body,
            "dy_body": dy_body,
        }


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VlmPointGate()
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
