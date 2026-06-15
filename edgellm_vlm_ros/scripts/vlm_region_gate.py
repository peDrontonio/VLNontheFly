#!/usr/bin/env python3
"""Gate coarse VLM region selections into depth-validated waypoint goals.

The VLM picks one region of a coarse image grid (e.g. CENTER, TOP-LEFT). This
node maps that region to a deterministic cell-center pixel, validates it with
the D435i depth image, and publishes a base_link-relative PoseStamped on
`/relative_goal` (same contract as vlm_point_gate). If the VLM region is
unusable (low confidence, blocked, no valid depth, or unparseable), the node
falls back to the most-open valid region measured purely from depth, so the
planner only ever receives a reachable goal. If no region is open it HOLDs.

The geometry (depth decoding, deprojection, body transform, clamping, PX4 pose
conversion) is reused from vlm_point_gate.
"""

import json
import os
import re
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

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

# Reuse the proven geometry from the point gate. At runtime the sibling module
# is on sys.path[0] (both install to lib/<pkg>); fall back to loading by path
# for tools/tests that import this file directly.
try:
    import vlm_point_gate as pg
except ImportError:  # pragma: no cover - exercised only outside the install tree
    import importlib.util

    _here = os.path.dirname(os.path.abspath(__file__))
    _spec = importlib.util.spec_from_file_location(
        "vlm_point_gate", os.path.join(_here, "vlm_point_gate.py"))
    pg = importlib.util.module_from_spec(_spec)
    sys.modules["vlm_point_gate"] = pg
    _spec.loader.exec_module(pg)


@dataclass(frozen=True)
class RegionProposal:
    region: str
    confidence: float
    reason: str = ""


@dataclass(frozen=True)
class CellDecision:
    accepted: bool
    reason: str
    col: Optional[int] = None
    row: Optional[int] = None
    region: Optional[str] = None
    depth_m: Optional[float] = None
    used_fallback: bool = False
    confidence: float = 0.0


def _axis_labels(n: int, horizontal: bool) -> List[str]:
    if horizontal:
        presets = {1: ["CENTER"], 2: ["LEFT", "RIGHT"], 3: ["LEFT", "CENTER", "RIGHT"]}
        prefix = "COL"
    else:
        presets = {1: ["MIDDLE"], 2: ["TOP", "BOTTOM"], 3: ["TOP", "MIDDLE", "BOTTOM"]}
        prefix = "ROW"
    if n in presets:
        return presets[n]
    return [f"{prefix}{i}" for i in range(n)]


def build_region_table(cols: int, rows: int) -> Dict[str, Tuple[int, int]]:
    """Map each region name to its (col, row) grid index."""
    if cols < 1 or rows < 1:
        raise ValueError("grid must have at least one column and row")
    horizontal = _axis_labels(cols, horizontal=True)
    vertical = _axis_labels(rows, horizontal=False)
    table: Dict[str, Tuple[int, int]] = {}
    for r in range(rows):
        for c in range(cols):
            if vertical[r] == "MIDDLE" and horizontal[c] == "CENTER":
                name = "CENTER"
            else:
                name = f"{vertical[r]}-{horizontal[c]}"
            table[name] = (c, r)
    return table


NONE_SENTINELS = {"NONE", "NULL", "NOTFOUND", "NOT-FOUND", "NA", "N-A", "NOTHING"}


def _normalize_region(value: Any) -> str:
    text = str(value).strip().upper().replace("_", "-").replace(" ", "-")
    return re.sub(r"-+", "-", text).strip("-")


def _match_region_keyword(text: str, known: List[str]) -> Optional[str]:
    norm = re.sub(r"[^A-Z-]", "", str(text).upper().replace("_", "-").replace(" ", "-"))
    for name in sorted(known, key=len, reverse=True):
        if name in norm:
            return name
    return None


def parse_region_result(payload: str, known: List[str]) -> RegionProposal:
    """Parse the VLM result into a region, tolerating broken/truncated JSON."""
    outer = json.loads(payload)
    if isinstance(outer, dict) and "text" in outer:
        text = str(outer["text"])
    else:
        text = payload

    region: Any = None
    confidence: Any = 0.0
    try:
        obj = json.loads(text)
    except json.JSONDecodeError:
        obj = None
    if isinstance(obj, dict):
        region = obj.get("region", obj.get("cell", obj.get("name")))
        confidence = obj.get("confidence", 0.0)

    if region is None:
        region = _match_region_keyword(text, known)
        match = re.search(r'"confidence"\s*:\s*([01](?:\.\d+)?)', text)
        if match is not None:
            confidence = match.group(1)

    if region is None:
        raise ValueError("no region in result")
    region = _normalize_region(region)
    if region in NONE_SENTINELS:
        region = "NONE"  # target-not-visible sentinel; the gate decides what to do
    elif region not in known:
        raise ValueError(f"unknown region: {region}")
    try:
        confidence = float(confidence)
    except (TypeError, ValueError):
        confidence = 0.0
    return RegionProposal(region=region, confidence=min(1.0, max(0.0, confidence)))


def cell_pixel_bounds(
    col: int, row: int, cols: int, rows: int, width: int, height: int
) -> Tuple[int, int, int, int]:
    x0 = int(round(col * width / cols))
    x1 = int(round((col + 1) * width / cols))
    y0 = int(round(row * height / rows))
    y1 = int(round((row + 1) * height / rows))
    return x0, y0, x1, y1


def cell_center_pixel(
    col: int, row: int, cols: int, rows: int, width: int, height: int
) -> Tuple[float, float]:
    return (col + 0.5) * width / cols, (row + 0.5) * height / rows


def cell_median_depth(
    depth_m: np.ndarray, bounds: Tuple[int, int, int, int], min_depth_m: float, max_depth_m: float
) -> Optional[float]:
    x0, y0, x1, y1 = bounds
    window = depth_m[y0:y1, x0:x1]
    if window.size == 0:
        return None
    valid = window[np.isfinite(window) & (window >= min_depth_m) & (window <= max_depth_m)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def scan_cells(
    depth_m: np.ndarray, cols: int, rows: int, min_depth_m: float, max_depth_m: float
) -> Dict[Tuple[int, int], Optional[float]]:
    """Median depth per grid cell (None when no valid depth)."""
    height, width = depth_m.shape
    result: Dict[Tuple[int, int], Optional[float]] = {}
    for r in range(rows):
        for c in range(cols):
            bounds = cell_pixel_bounds(c, r, cols, rows, width, height)
            result[(c, r)] = cell_median_depth(depth_m, bounds, min_depth_m, max_depth_m)
    return result


def apply_standoff(dx: float, dy: float, standoff_m: float) -> Tuple[float, float]:
    """Shorten an approach offset so the drone stops standoff_m before the target.

    Returns (0, 0) when the target is already within the standoff distance.
    """
    distance = float(np.hypot(dx, dy))
    if standoff_m <= 0.0 or distance == 0.0:
        return dx, dy
    if distance <= standoff_m:
        return 0.0, 0.0
    scale = (distance - standoff_m) / distance
    return dx * scale, dy * scale


def most_open_cell(
    depths: Dict[Tuple[int, int], Optional[float]], min_clearance_m: float
) -> Optional[Tuple[int, int]]:
    best: Optional[Tuple[int, int]] = None
    best_depth = -1.0
    for cell, depth in depths.items():
        if depth is None or depth < min_clearance_m:
            continue
        if depth > best_depth:
            best_depth = depth
            best = cell
    return best


class VlmRegionGate(Node):
    def __init__(self) -> None:
        super().__init__("vlm_region_gate")

        self.vlm_result_topic = self.declare_parameter(
            "vlm_result_topic", "/edgellm_vlm_node/result").value
        self.depth_topic = self.declare_parameter(
            "depth_topic", "/camera/camera/depth/image_rect_raw").value
        self.depth_camera_info_topic = self.declare_parameter(
            "depth_camera_info_topic", "/camera/camera/depth/camera_info").value
        self.odometry_topic = self.declare_parameter("odometry_topic", "").value
        self.pose_topic = self.declare_parameter("pose_topic", "").value
        self.vehicle_local_position_topic = self.declare_parameter(
            "vehicle_local_position_topic", "/fmu/out/vehicle_local_position").value
        self.goal_topic = self.declare_parameter("goal_topic", "/relative_goal").value
        self.goal_frame_id = self.declare_parameter("goal_frame_id", "base_link").value

        self.grid_cols = int(self.declare_parameter("grid_cols", 3).value)
        self.grid_rows = int(self.declare_parameter("grid_rows", 3).value)
        # "open_space": go to the most-open region (depth fallback when VLM is unsure).
        # "target": approach the region the VLM reports the target object in; HOLD if not found.
        self.selection_mode = str(self.declare_parameter("selection_mode", "open_space").value)
        self.use_vlm_region = bool(self.declare_parameter("use_vlm_region", True).value)
        self.min_confidence = float(self.declare_parameter("min_confidence", 0.50).value)
        self.min_clearance_m = float(self.declare_parameter("min_clearance_m", 0.8).value)
        # Stop this far short of the target surface in "target" mode.
        self.standoff_m = float(self.declare_parameter("standoff_m", 0.8).value)
        self.auto_execute = bool(self.declare_parameter("auto_execute", False).value)

        self.depth_scale_m = float(self.declare_parameter("depth_scale_m", 0.001).value)
        self.min_depth_m = float(self.declare_parameter("min_depth_m", 0.25).value)
        self.max_depth_m = float(self.declare_parameter("max_depth_m", 4.0).value)
        self.max_goal_distance_m = float(self.declare_parameter("max_goal_distance_m", 1.5).value)
        self.goal_z_mode = self.declare_parameter("goal_z_mode", "current_pose").value
        self.fixed_goal_z_m = float(self.declare_parameter("fixed_goal_z_m", 1.0).value)
        self.cooldown_s = float(self.declare_parameter("cooldown_s", 2.0).value)
        self.max_result_age_s = float(self.declare_parameter("max_result_age_s", 2.5).value)
        self.max_depth_age_s = float(self.declare_parameter("max_depth_age_s", 0.5).value)
        self.max_pose_age_s = float(self.declare_parameter("max_pose_age_s", 1.0).value)

        self.region_table = build_region_table(self.grid_cols, self.grid_rows)
        self.region_names = list(self.region_table.keys())

        qos = QoSProfile(depth=10)
        self.result_sub = self.create_subscription(
            String, self.vlm_result_topic, self._on_result, qos)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._on_depth, qos)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, self.depth_camera_info_topic, self._on_depth_info, qos)
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
        self.latest_proposal: Optional[RegionProposal] = None
        self.latest_decision: Optional[CellDecision] = None
        self.latest_result_rx: Optional[rclpy.time.Time] = None
        self.last_goal_pub: Optional[rclpy.time.Time] = None

        self._publish_status(
            f"ready grid={self.grid_cols}x{self.grid_rows} regions={len(self.region_names)} "
            f"use_vlm_region={self.use_vlm_region} auto_execute={self.auto_execute} "
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

    def _publish_proposal(self, decision: CellDecision) -> None:
        payload: Dict[str, Any] = {
            "accepted": decision.accepted,
            "gate_reason": decision.reason,
            "region": decision.region,
            "col": decision.col,
            "row": decision.row,
            "depth_m": decision.depth_m,
            "used_fallback": decision.used_fallback,
            "confidence": decision.confidence,
        }
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
        if not (msg.xy_valid and msg.z_valid and np.isfinite(msg.heading)):
            return
        self.latest_pose = pg.px4_local_position_to_pose(msg)
        self.latest_pose_rx = self._now()

    def _on_depth(self, msg: Image) -> None:
        self.latest_depth_msg = msg
        self.latest_depth_rx = self._now()

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self.latest_depth_info = msg

    def _decide_cell(self) -> CellDecision:
        if self.latest_depth_msg is None:
            return CellDecision(False, "no depth image received")
        depth_m = pg.depth_image_to_meters(self.latest_depth_msg, self.depth_scale_m)
        depths = scan_cells(
            depth_m, self.grid_cols, self.grid_rows, self.min_depth_m, self.max_depth_m)
        if self.selection_mode == "target":
            return self._decide_target(depths)
        return self._decide_open_space(depths)

    def _decide_target(self, depths: Dict[Tuple[int, int], Optional[float]]) -> CellDecision:
        """Approach the region the VLM reports the target in; HOLD if not found."""
        proposal = self.latest_proposal
        if proposal is None or proposal.region == "NONE":
            return CellDecision(False, "target_not_found")
        if proposal.confidence < self.min_confidence:
            return CellDecision(False, "target_low_confidence", confidence=proposal.confidence)
        cell = self.region_table.get(proposal.region)
        if cell is None:
            return CellDecision(False, "target_region_invalid", confidence=proposal.confidence)
        depth = depths.get(cell)
        if depth is None:
            return CellDecision(False, "target_no_depth", region=proposal.region,
                                col=cell[0], row=cell[1], confidence=proposal.confidence)
        return CellDecision(True, "vlm_target", cell[0], cell[1], proposal.region,
                            depth, used_fallback=False, confidence=proposal.confidence)

    def _decide_open_space(self, depths: Dict[Tuple[int, int], Optional[float]]) -> CellDecision:
        """Pick a grid cell: VLM region if usable, else most-open by depth."""
        proposal = self.latest_proposal
        if self.use_vlm_region and proposal is not None and proposal.confidence >= self.min_confidence:
            cell = self.region_table.get(proposal.region)
            if cell is not None:
                depth = depths.get(cell)
                if depth is not None and depth >= self.min_clearance_m:
                    return CellDecision(
                        True, "vlm_region", cell[0], cell[1], proposal.region,
                        depth, used_fallback=False, confidence=proposal.confidence)

        # Fallback: most-open valid cell measured from depth alone.
        cell = most_open_cell(depths, self.min_clearance_m)
        if cell is None:
            return CellDecision(False, "all_regions_blocked")
        region = next((n for n, idx in self.region_table.items() if idx == cell), None)
        conf = proposal.confidence if proposal is not None else 0.0
        return CellDecision(
            True, "depth_fallback", cell[0], cell[1], region,
            depths[cell], used_fallback=True, confidence=conf)

    def _on_result(self, msg: String) -> None:
        self.latest_result_rx = self._now()
        try:
            self.latest_proposal = parse_region_result(msg.data, self.region_names)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.latest_proposal = None
            self._publish_status(f"region:unparsed:{exc}")

        decision = self._decide_cell()
        self.latest_decision = decision
        self._publish_proposal(decision)
        status = "proposal:accepted:" + decision.reason if decision.accepted \
            else "proposal:rejected:" + decision.reason
        self._publish_status(status)

        if self.auto_execute and decision.accepted:
            self._try_publish_goal()

    def _on_execute_next(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        ok, message = self._try_publish_goal()
        response.success = ok
        response.message = message
        return response

    def _try_publish_goal(self) -> Tuple[bool, str]:
        ok, reason = self._can_publish_goal()
        if not ok:
            self._publish_status(f"execute:rejected:{reason}")
            return False, reason

        assert self.latest_decision is not None
        assert self.latest_pose is not None
        assert self.latest_depth_msg is not None
        assert self.latest_depth_info is not None
        try:
            goal, depth_m = self._build_goal(
                self.latest_decision, self.latest_pose,
                self.latest_depth_msg, self.latest_depth_info)
        except ValueError as exc:
            self._publish_status(f"execute:rejected:{exc}")
            return False, str(exc)

        self.goal_pub.publish(goal)
        self.last_goal_pub = self._now()
        message = (
            f"published region {self.latest_decision.region} "
            f"({'fallback' if self.latest_decision.used_fallback else 'vlm'}) offset "
            f"(fwd={goal.pose.position.x:.2f}, left={goal.pose.position.y:.2f}, "
            f"up={goal.pose.position.z:.2f}) depth={depth_m:.2f}m")
        self._publish_status(f"execute:published:{self.latest_decision.region}")
        return True, message

    def _can_publish_goal(self) -> Tuple[bool, str]:
        if self.latest_decision is None or not self.latest_decision.accepted:
            return False, "no usable region decision"
        if self.latest_pose is None:
            return False, "no pose received"
        if self.latest_depth_msg is None:
            return False, "no depth image received"
        if self.latest_depth_info is None:
            return False, "no depth camera info received"

        result_age = self._seconds_since(self.latest_result_rx)
        if result_age is None or result_age > self.max_result_age_s:
            return False, "VLM region result is stale"
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
        self, decision: CellDecision, pose: Pose, depth_msg: Image, depth_info: CameraInfo
    ) -> Tuple[PoseStamped, float]:
        depth_m_img = pg.depth_image_to_meters(depth_msg, self.depth_scale_m)
        bounds = cell_pixel_bounds(
            decision.col, decision.row, self.grid_cols, self.grid_rows,
            depth_msg.width, depth_msg.height)
        depth_m = cell_median_depth(depth_m_img, bounds, self.min_depth_m, self.max_depth_m)
        if depth_m is None:
            raise ValueError("no valid depth in selected region")
        u, v = cell_center_pixel(
            decision.col, decision.row, self.grid_cols, self.grid_rows,
            depth_msg.width, depth_msg.height)
        point_optical = pg.deproject_pixel_to_optical(u, v, depth_m, depth_info)
        dx_body, dy_body = pg.optical_to_body_horizontal(point_optical)
        if self.selection_mode == "target":
            # Stop short of the target surface, then clamp the hop length.
            dx_body, dy_body = apply_standoff(dx_body, dy_body, self.standoff_m)
        dx_body, dy_body = pg.clamp_xy_distance(dx_body, dy_body, self.max_goal_distance_m)
        target_abs_z = pg.resolve_goal_z(self.goal_z_mode, self.fixed_goal_z_m, pose.position.z)

        goal = PoseStamped()
        goal.header.stamp = self._now().to_msg()
        goal.header.frame_id = self.goal_frame_id
        goal.pose.position.x = dx_body
        goal.pose.position.y = dy_body
        goal.pose.position.z = target_abs_z - pose.position.z
        goal.pose.orientation.w = 1.0
        return goal, depth_m


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VlmRegionGate()
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
