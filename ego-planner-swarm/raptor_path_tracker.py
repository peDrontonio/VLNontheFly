#!/usr/bin/env python3
"""
Raptor path tracker ("carrot" sampler).

Bridges an ego-planner path into the Raptor EXTERNAL-mode setpoint stream:

    ego-planner path (nav_msgs/Path, ENU)
      -> walk a "carrot" along the path at a fixed speed (default 0.8 m/s)
      -> px4_msgs/TrajectorySetpoint (NED, position + velocity feedforward + optional yaw)
      -> /fmu/in/trajectory_setpoint_raptor  (uXRCE-DDS) -> uORB trajectory_setpoint_raptor
      -> rl_tools_commander (EXTERNAL mode) -> rl_tools_policy

This node has NO authority over activation. Whether Raptor is engaged is decided entirely by the
operator's RC switch (commander -> policy.active -> multiplexer SWITCH_BACK). If this node stops or
spams, the commander freezes the target (EXTERNAL_SETPOINT_TIMEOUT) and the operator's switch is still
the master override.

Frames: ego-planner publishes ENU (ROS REP-103); PX4 is NED. We convert once on path receipt:
    x_ned = y_enu,  y_ned = x_enu,  z_ned = -z_enu
Both ego-planner and the PX4 EKF must share the same local origin (same odometry source).

Run (on the companion, with the MicroXRCE Agent already bridging to the FC):
    ros2 run <your_pkg> raptor_path_tracker
or standalone:
    python3 raptor_path_tracker.py --ros-args -p path_topic:=/ego_planner/path -p speed:=0.8
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition

NAN = float("nan")


def enu_to_ned(p):
    """(x,y,z) ENU -> NED."""
    return np.array([p[1], p[0], -p[2]], dtype=float)


class RaptorPathTracker(Node):
    def __init__(self):
        super().__init__("raptor_path_tracker")

        # --- params
        self.declare_parameter("path_topic", "/ego_planner/path")
        self.declare_parameter("speed", 0.8)            # m/s along the path
        self.declare_parameter("rate_hz", 50.0)         # setpoint publish rate
        self.declare_parameter("face_travel", False)    # True: yaw faces travel dir; False: hold (NaN)
        self.declare_parameter("out_topic", "/fmu/in/trajectory_setpoint_raptor")
        self.declare_parameter("odom_topic", "/fmu/out/vehicle_local_position")

        path_topic = self.get_parameter("path_topic").value
        out_topic = self.get_parameter("out_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.speed = float(self.get_parameter("speed").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.face_travel = bool(self.get_parameter("face_travel").value)

        # --- state (everything internal is NED)
        self.pts = None          # Nx3 NED waypoints
        self.cum = None          # cumulative arc-length per waypoint
        self.s = 0.0             # current arc-length progress
        self.drone_ned = None    # latest vehicle position (NED) for re-anchoring on replan

        # PX4 uXRCE QoS: best-effort, transient-local, depth 1
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(TrajectorySetpoint, out_topic, px4_qos)
        self.create_subscription(Path, path_topic, self.on_path, 10)
        self.create_subscription(VehicleLocalPosition, odom_topic, self.on_odom, px4_qos)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(
            f"raptor_path_tracker: path={path_topic} -> {out_topic} @ {self.rate_hz:.0f} Hz, "
            f"speed={self.speed:.2f} m/s, face_travel={self.face_travel}"
        )

    def on_odom(self, msg: VehicleLocalPosition):
        if msg.xy_valid and msg.z_valid:
            self.drone_ned = np.array([msg.x, msg.y, msg.z], dtype=float)

    def on_path(self, msg: Path):
        """New path => full overwrite. Convert ENU->NED and re-anchor the carrot to the path point
        nearest the drone's current position (smooth continuation across replans)."""
        if len(msg.poses) < 2:
            self.get_logger().warn("path has < 2 poses; ignoring")
            return
        pts = np.array(
            [enu_to_ned((p.pose.position.x, p.pose.position.y, p.pose.position.z)) for p in msg.poses]
        )
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])

        self.pts = pts
        self.cum = cum

        # re-anchor: nearest waypoint to the drone (fall back to start)
        if self.drone_ned is not None:
            d = np.linalg.norm(pts - self.drone_ned[None, :], axis=1)
            self.s = float(cum[int(np.argmin(d))])
        else:
            self.s = 0.0

    def _sample(self, s):
        """Return (position_ned, tangent_unit_ned) at arc-length s along the polyline."""
        cum = self.cum
        total = cum[-1]
        s = max(0.0, min(s, total))
        # find segment containing s
        i = int(np.searchsorted(cum, s, side="right") - 1)
        i = max(0, min(i, len(self.pts) - 2))
        seg_len = cum[i + 1] - cum[i]
        t = 0.0 if seg_len < 1e-6 else (s - cum[i]) / seg_len
        p = self.pts[i] * (1 - t) + self.pts[i + 1] * t
        tang = self.pts[i + 1] - self.pts[i]
        n = np.linalg.norm(tang)
        tang = tang / n if n > 1e-6 else np.zeros(3)
        return p, tang

    def on_timer(self):
        # No path yet => publish nothing; the commander stays frozen (safe).
        if self.pts is None:
            return

        total = self.cum[-1]
        at_end = self.s >= total - 1e-3

        # advance the carrot
        if not at_end:
            self.s = min(self.s + self.speed / self.rate_hz, total)

        pos, tang = self._sample(self.s)
        vel = np.zeros(3) if at_end else self.speed * tang

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
        msg.position = [float(pos[0]), float(pos[1]), float(pos[2])]
        msg.velocity = [float(vel[0]), float(vel[1]), float(vel[2])]
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        if self.face_travel and not at_end and np.linalg.norm(tang) > 1e-6:
            msg.yaw = float(math.atan2(tang[1], tang[0]))  # NED yaw: atan2(east, north)
        else:
            msg.yaw = NAN  # commander holds the activation yaw
        msg.yawspeed = NAN
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RaptorPathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
