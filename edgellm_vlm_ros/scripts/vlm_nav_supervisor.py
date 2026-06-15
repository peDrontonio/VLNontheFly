#!/usr/bin/env python3
"""Switch one VLM runtime between point goals and altitude primitives."""

import json
from dataclasses import dataclass
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger


POINT_NAV = "POINT_NAV"
ALTITUDE_ADJUST = "ALTITUDE_ADJUST"
SETTLE = "SETTLE"
HOLD = "HOLD"


@dataclass
class ProposalState:
    accepted: bool = False
    reason: str = ""
    payload: Optional[Dict[str, Any]] = None
    stamp: Optional[rclpy.time.Time] = None
    executed: bool = False


def parse_gate_proposal(payload: str) -> ProposalState:
    data = json.loads(payload)
    if not isinstance(data, dict):
        raise ValueError("proposal must be a JSON object")
    return ProposalState(
        accepted=bool(data.get("accepted", False)),
        reason=str(data.get("gate_reason", "")),
        payload=data,
    )


class VlmNavSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("vlm_nav_supervisor")

        self.auto_execute = bool(self.declare_parameter("auto_execute", False).value)
        self.tick_hz = float(self.declare_parameter("tick_hz", 1.0).value)
        self.point_rejection_limit = int(self.declare_parameter("point_rejection_limit", 3).value)
        self.primitive_rejection_limit = int(
            self.declare_parameter("primitive_rejection_limit", 2).value)
        self.max_proposal_age_s = float(self.declare_parameter("max_proposal_age_s", 3.0).value)
        self.settle_s = float(self.declare_parameter("settle_s", 2.0).value)

        self.point_mode_service = self.declare_parameter(
            "point_mode_service", "/edgellm_vlm_node/set_point_mode").value
        self.point_execute_service = self.declare_parameter(
            "point_execute_service", "/vlm_point_gate/execute_next").value
        self.primitive_execute_service = self.declare_parameter(
            "primitive_execute_service", "/vlm_primitive_gate/execute_next").value
        self.point_proposal_topic = self.declare_parameter(
            "point_proposal_topic", "/vlm_point_gate/proposal").value
        self.primitive_proposal_topic = self.declare_parameter(
            "primitive_proposal_topic", "/vlm_primitive_gate/proposal").value

        self.point_mode_client = self.create_client(SetBool, self.point_mode_service)
        self.point_execute_client = self.create_client(Trigger, self.point_execute_service)
        self.primitive_execute_client = self.create_client(Trigger, self.primitive_execute_service)

        self.point_proposal_sub = self.create_subscription(
            String, self.point_proposal_topic, self._on_point_proposal, 10)
        self.primitive_proposal_sub = self.create_subscription(
            String, self.primitive_proposal_topic, self._on_primitive_proposal, 10)
        self.state_pub = self.create_publisher(String, "~/state", 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.step_srv = self.create_service(Trigger, "~/step", self._on_step)
        self.enable_srv = self.create_service(SetBool, "~/set_auto_execute", self._on_set_auto_execute)

        self.state = POINT_NAV
        self.requested_point_mode: Optional[bool] = None
        self.pending_mode_future = None
        self.pending_execute_future = None
        self.pending_execute_kind: Optional[str] = None
        self.state_entered = self._now()

        self.latest_point = ProposalState()
        self.latest_primitive = ProposalState()
        self.point_rejections = 0
        self.primitive_rejections = 0

        period = 1.0 / self.tick_hz if self.tick_hz > 0.0 else 1.0
        self.timer = self.create_timer(period, self._on_timer)
        self._publish_status("ready")
        self._publish_state()

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

    def _publish_state(self) -> None:
        msg = String()
        msg.data = json.dumps({
            "state": self.state,
            "auto_execute": self.auto_execute,
            "requested_prompt_mode": "point" if self.requested_point_mode else "primitive",
            "point_rejections": self.point_rejections,
            "primitive_rejections": self.primitive_rejections,
        }, separators=(",", ":"))
        self.state_pub.publish(msg)

    def _transition(self, state: str, reason: str) -> None:
        if self.state == state:
            return
        self.state = state
        self.state_entered = self._now()
        self._publish_status(f"state:{state}:{reason}")
        self._publish_state()

    def _on_point_proposal(self, msg: String) -> None:
        try:
            proposal = parse_gate_proposal(msg.data)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            proposal = ProposalState(False, f"parse_error:{exc}")
        proposal.stamp = self._now()
        self.latest_point = proposal
        if self.state == POINT_NAV:
            if proposal.accepted:
                self.point_rejections = 0
            else:
                self.point_rejections += 1
        self._publish_state()

    def _on_primitive_proposal(self, msg: String) -> None:
        try:
            proposal = parse_gate_proposal(msg.data)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            proposal = ProposalState(False, f"parse_error:{exc}")
        proposal.stamp = self._now()
        self.latest_primitive = proposal
        if self.state == ALTITUDE_ADJUST:
            if proposal.accepted:
                self.primitive_rejections = 0
            else:
                self.primitive_rejections += 1
        self._publish_state()

    def _on_set_auto_execute(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        self.auto_execute = bool(request.data)
        response.success = True
        response.message = f"auto_execute={self.auto_execute}"
        self._publish_status(response.message)
        self._publish_state()
        return response

    def _on_step(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        acted, reason = self._step(force_execute=True)
        response.success = acted
        response.message = reason
        return response

    def _on_timer(self) -> None:
        self._check_futures()
        self._step(force_execute=self.auto_execute)

    def _check_futures(self) -> None:
        if self.pending_mode_future is not None and self.pending_mode_future.done():
            try:
                response = self.pending_mode_future.result()
                if response.success:
                    self._publish_status(response.message)
                else:
                    self._publish_status(f"mode_switch_failed:{response.message}")
            except Exception as exc:  # noqa: BLE001
                self._publish_status(f"mode_switch_exception:{exc}")
            self.pending_mode_future = None

        if self.pending_execute_future is not None and self.pending_execute_future.done():
            kind = self.pending_execute_kind or "unknown"
            try:
                response = self.pending_execute_future.result()
                if response.success:
                    self._publish_status(f"execute:{kind}:success:{response.message}")
                    if kind == "primitive":
                        self._transition(SETTLE, "primitive executed")
                else:
                    self._publish_status(f"execute:{kind}:failed:{response.message}")
                    if kind == "point":
                        self.point_rejections += 1
                    if kind == "primitive":
                        self.primitive_rejections += 1
            except Exception as exc:  # noqa: BLE001
                self._publish_status(f"execute:{kind}:exception:{exc}")
            self.pending_execute_future = None
            self.pending_execute_kind = None
            self._publish_state()

    def _step(self, force_execute: bool) -> tuple[bool, str]:
        if self.state == POINT_NAV:
            self._request_point_mode(True)
            if self._point_is_stale():
                self.point_rejections += 1
                self._publish_state()
            if self.point_rejections >= self.point_rejection_limit:
                self.primitive_rejections = 0
                self.latest_primitive = ProposalState()
                self._transition(ALTITUDE_ADJUST, "point rejected repeatedly")
                self._request_point_mode(False)
                return True, "switched to altitude primitive mode"
            if force_execute and self._can_execute(self.latest_point):
                return self._execute_point()
            return False, "waiting for accepted point proposal"

        if self.state == ALTITUDE_ADJUST:
            self._request_point_mode(False)
            if self.primitive_rejections >= self.primitive_rejection_limit:
                self._transition(HOLD, "primitive rejected repeatedly")
                return True, "switched to hold"
            if force_execute and self._can_execute(self.latest_primitive):
                return self._execute_primitive()
            return False, "waiting for accepted primitive proposal"

        if self.state == SETTLE:
            age = self._seconds_since(self.state_entered)
            if age is not None and age >= self.settle_s:
                self.point_rejections = 0
                self.latest_point = ProposalState()
                self._transition(POINT_NAV, "settle complete")
                self._request_point_mode(True)
                return True, "returned to point mode"
            return False, "settling"

        if self.state == HOLD:
            self._request_point_mode(True)
            if self.latest_point.accepted:
                self.point_rejections = 0
                self._transition(POINT_NAV, "new point accepted")
                return True, "returned to point mode"
            return False, "holding"

        return False, f"unknown state {self.state}"

    def _point_is_stale(self) -> bool:
        age = self._seconds_since(self.latest_point.stamp)
        return age is not None and age > self.max_proposal_age_s and not self.latest_point.executed

    def _can_execute(self, proposal: ProposalState) -> bool:
        if self.pending_execute_future is not None:
            return False
        if proposal.executed or not proposal.accepted:
            return False
        age = self._seconds_since(proposal.stamp)
        return age is not None and age <= self.max_proposal_age_s

    def _request_point_mode(self, point_mode: bool) -> None:
        if self.requested_point_mode == point_mode:
            return
        if self.pending_mode_future is not None:
            return
        if not self.point_mode_client.service_is_ready():
            self.point_mode_client.wait_for_service(timeout_sec=0.0)
            return
        request = SetBool.Request()
        request.data = point_mode
        self.pending_mode_future = self.point_mode_client.call_async(request)
        self.requested_point_mode = point_mode
        mode = "point" if point_mode else "primitive"
        self._publish_status(f"request_prompt_mode:{mode}")
        self._publish_state()

    def _execute_point(self) -> tuple[bool, str]:
        if not self.point_execute_client.service_is_ready():
            self.point_execute_client.wait_for_service(timeout_sec=0.0)
            return False, "point execute service unavailable"
        self.latest_point.executed = True
        self.pending_execute_kind = "point"
        self.pending_execute_future = self.point_execute_client.call_async(Trigger.Request())
        self._publish_status("execute:point:requested")
        return True, "point execute requested"

    def _execute_primitive(self) -> tuple[bool, str]:
        if not self.primitive_execute_client.service_is_ready():
            self.primitive_execute_client.wait_for_service(timeout_sec=0.0)
            return False, "primitive execute service unavailable"
        primitive = None
        if self.latest_primitive.payload is not None:
            primitive = str(self.latest_primitive.payload.get("primitive", "")).upper()
        if primitive not in {"UP", "DOWN", "HOLD"}:
            self.latest_primitive.executed = True
            self.primitive_rejections += 1
            self._publish_status(f"execute:primitive:rejected:non_altitude:{primitive}")
            return False, "primitive is not an altitude primitive"
        self.latest_primitive.executed = True
        self.pending_execute_kind = "primitive"
        self.pending_execute_future = self.primitive_execute_client.call_async(Trigger.Request())
        self._publish_status("execute:primitive:requested")
        return True, "primitive execute requested"


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VlmNavSupervisor()
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
