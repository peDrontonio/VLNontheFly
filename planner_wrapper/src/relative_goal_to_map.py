#!/usr/bin/env python3
"""
Convert body-relative goals into absolute map goals for ego-planner.

Input `/relative_goal` is geometry_msgs/PoseStamped where position is an offset
in the current base_link FLU convention:
  x: forward, y: left, z: up.

Output `/move_base_simple/goal` is an absolute PoseStamped in the map frame,
which is what ego-planner's manual target callback consumes.
"""

import math
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RelativeGoalToMap(Node):
    def __init__(self):
        super().__init__("relative_goal_to_map")

        self.declare_parameter("odom_topic", "/odometry")
        self.declare_parameter("input_topic", "/relative_goal")
        self.declare_parameter("output_topic", "/move_base_simple/goal")
        self.declare_parameter("map_frame", "map")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.map_frame = self.get_parameter("map_frame").value

        self._lock = threading.Lock()
        self._odom = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos)
        self.create_subscription(PoseStamped, self.input_topic, self.on_goal, 10)
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        self.get_logger().info(
            f"relative_goal_to_map: {self.input_topic} base_link offsets "
            f"-> {self.output_topic} absolute {self.map_frame} goals, using {self.odom_topic}"
        )

    def on_odom(self, msg: Odometry):
        with self._lock:
            self._odom = msg

    def on_goal(self, msg: PoseStamped):
        with self._lock:
            odom = self._odom

        if odom is None:
            self.get_logger().warn("Ignoring relative goal: no odometry yet.")
            return

        rel = msg.pose.position
        pos = odom.pose.pose.position
        yaw = yaw_from_quaternion(odom.pose.pose.orientation)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame
        out.pose.position.x = float(pos.x + cy * rel.x - sy * rel.y)
        out.pose.position.y = float(pos.y + sy * rel.x + cy * rel.y)
        out.pose.position.z = float(pos.z + rel.z)
        out.pose.orientation.w = 1.0

        self.pub.publish(out)
        self.get_logger().info(
            "relative goal "
            f"forward={rel.x:.2f} left={rel.y:.2f} up={rel.z:.2f} -> "
            f"{self.map_frame} ({out.pose.position.x:.2f}, "
            f"{out.pose.position.y:.2f}, {out.pose.position.z:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RelativeGoalToMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
