#!/usr/bin/env python3
"""
mock_arena.py — a fake drone and a fake ego-planner, so the tree can be flown
on a desk.

Publishes everything the tree consumes and impersonates the half of the
handshake ego-planner would normally provide:

    publishes  /odometry              nav_msgs/Odometry     a drone that moves
    publishes  /planning/goal_status  std_msgs/String       the planner's replies
    publishes  /battery_status        sensor_msgs/BatteryState
    subscribes /move_base_simple/goal geometry_msgs/PoseStamped

The point is not realism. It is that every failure mode the tree is supposed to
survive can be produced on demand, in a second, with no aircraft:

    planner_mode:=normal    accept the goal, fly there, report `reached`
    planner_mode:=silent    accept the goal and then say NOTHING, ever.
                            *** This is the real ego-planner failure that has
                            no status of its own -- no replan-failure and no
                            collision message exists. It is the reason every
                            goal-following leaf must sit under a Deadline. ***
    planner_mode:=reject    refuse every goal with `rejected:<reason>`

    battery_drain_per_min:=20.0   watch the safety branch preempt a mission
    odom_stops_after_s:=10.0      kill pose mid-flight and watch it preempt too

Example::

    ros2 run imav_bt mock_arena.py --ros-args -p planner_mode:=silent
    ros2 run imav_bt bt_node.py --ros-args -p autostart:=true

See docs/TESTING.md for the full fault-injection matrix.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

PLANNER_MODES = ("normal", "silent", "reject")


class MockArena(Node):
    """A fake drone plus a fake ego-planner."""

    def __init__(self):
        super().__init__("mock_arena")

        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("speed_mps", 1.5)
        self.declare_parameter("start_position", [0.0, 0.0, 1.2])
        self.declare_parameter("planner_mode", "normal")
        self.declare_parameter("reject_reason", "rejected:not_ready")
        self.declare_parameter("arrival_tolerance_m", 0.15)
        self.declare_parameter("battery_start", 1.0)
        self.declare_parameter("battery_drain_per_min", 0.0)
        # 0 disables. Set >0 to stop publishing odometry after N seconds and
        # confirm the localization guard preempts whatever is running.
        self.declare_parameter("odom_stops_after_s", 0.0)

        self.mode = self.get_parameter("planner_mode").value
        if self.mode not in PLANNER_MODES:
            raise ValueError(
                f"planner_mode must be one of {PLANNER_MODES}, got {self.mode!r}"
            )

        self.speed = float(self.get_parameter("speed_mps").value)
        self.tolerance = float(self.get_parameter("arrival_tolerance_m").value)
        self.reject_reason = str(self.get_parameter("reject_reason").value)
        self.battery = float(self.get_parameter("battery_start").value)
        self.drain_per_s = float(self.get_parameter("battery_drain_per_min").value) / 60.0
        self.odom_stops_after = float(self.get_parameter("odom_stops_after_s").value)

        self.position = [float(v) for v in self.get_parameter("start_position").value]
        self.target = None
        self.announced_arrival = True

        # BEST_EFFORT to match planner/src/odometry_converter.py, so the real
        # subscriber QoS is exercised too.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.odom_pub = self.create_publisher(Odometry, "/odometry", sensor_qos)
        self.battery_pub = self.create_publisher(
            BatteryState, "/battery_status", sensor_qos
        )
        self.status_pub = self.create_publisher(String, "/planning/goal_status", 10)
        self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self._on_goal, 10
        )

        rate = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / rate
        self.started = time.monotonic()
        self.create_timer(self.dt, self._step)

        self.get_logger().info(
            f"mock_arena | planner_mode={self.mode} | speed={self.speed} m/s | "
            f"drain={self.drain_per_s * 60:.1f} %/min"
        )

    # -- fake planner ------------------------------------------------------

    def _on_goal(self, msg: PoseStamped):
        p = msg.pose.position
        target = [float(p.x), float(p.y), float(p.z)]

        if self.mode == "reject":
            self._say(self.reject_reason)
            self.get_logger().info(f"goal rejected: {target}")
            return

        self.target = target
        self.announced_arrival = False
        self._say("accepted")
        note = " (and will now go silent forever)" if self.mode == "silent" else ""
        self.get_logger().info(f"goal accepted: {target}{note}")

    def _say(self, status: str):
        message = String()
        message.data = status
        self.status_pub.publish(message)

    # -- fake drone --------------------------------------------------------

    def _step(self):
        elapsed = time.monotonic() - self.started
        self._advance_toward_target()

        if self.odom_stops_after > 0.0 and elapsed > self.odom_stops_after:
            # Deliberate silence: the localization guard should notice.
            return

        self._publish_odom()
        self._publish_battery()

    def _advance_toward_target(self):
        if self.target is None:
            return

        delta = [t - p for t, p in zip(self.target, self.position)]
        distance = math.sqrt(sum(d * d for d in delta))

        if distance <= self.tolerance:
            self.position = list(self.target)
            if not self.announced_arrival:
                self.announced_arrival = True
                if self.mode == "silent":
                    # Arrived, but the planner never admits it. This is exactly
                    # the case only a Deadline can catch.
                    self.get_logger().info("arrived, staying silent (by design)")
                else:
                    self._say("reached")
                    self.get_logger().info("reached")
            return

        step = min(self.speed * self.dt, distance)
        self.position = [
            p + step * d / distance for p, d in zip(self.position, delta)
        ]

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.position[0]
        msg.pose.pose.position.y = self.position[1]
        msg.pose.pose.position.z = self.position[2]
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def _publish_battery(self):
        if self.drain_per_s:
            self.battery = max(0.0, self.battery - self.drain_per_s * self.dt)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(self.battery)
        msg.present = True
        self.battery_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockArena()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
