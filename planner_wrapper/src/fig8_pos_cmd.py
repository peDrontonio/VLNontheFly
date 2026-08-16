#!/usr/bin/env python3
"""
fig8_pos_cmd.py — fixed figure-eight PositionCommand streamer (pipeline isolation test).

Publishes a figure-8 (Gerono lemniscate) as quadrotor_msgs/PositionCommand on
`/drone_0_planning/pos_cmd` at 100 Hz, in ENU, exactly like ego-planner's
traj_server would. Feeding it to pos_cmd_to_raptor exercises the full
production chain (bridge -> uXRCE-DDS -> rl_tools_commander EXTERNAL) with
ego-planner being the only piece swapped out.

The 8 is anchored at the drone's current /odometry position and yaw when the
node starts, so it never commands a jump:

    x(tau) = x0 + size_x * sin(tau)        (ENU East)
    y(tau) = y0 + size_y * sin(2 tau)      (ENU North)
    z      = z0                            (constant altitude)

tau advances with a smooth speed ramp: 0 -> nominal over `ramp` seconds, and
back to 0 after `laps` laps — so commanded velocity starts and ends at zero.
After the ramp-down the node keeps publishing the final point with zero
velocity (hover hold), like traj_server does at a goal.

Run (with `ego_raptor.launch.py start_planner:=false` already up):
    ros2 run planner fig8_pos_cmd.py
    ros2 run planner fig8_pos_cmd.py --ros-args -p size_x:=1.0 -p size_y:=0.5 -p period:=30.0 -p laps:=2
"""
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


class Fig8PosCmd(Node):
    def __init__(self):
        super().__init__("fig8_pos_cmd")

        self.declare_parameter("size_x", 1.0)     # m, East half-width of the 8
        self.declare_parameter("size_y", 0.5)     # m, North half-height of the 8
        self.declare_parameter("period", 30.0)    # s per lap
        self.declare_parameter("laps", 2.0)       # laps before ramp-down (0 = endless)
        self.declare_parameter("ramp", 3.0)       # s, speed ramp up/down
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("max_speed", 1.0)  # m/s, refuse to run above this
        self.declare_parameter("odom_topic", "/odometry")
        self.declare_parameter("out_topic", "/drone_0_planning/pos_cmd")

        self.ax = float(self.get_parameter("size_x").value)
        self.ay = float(self.get_parameter("size_y").value)
        self.period = float(self.get_parameter("period").value)
        self.laps = float(self.get_parameter("laps").value)
        self.ramp = max(0.5, float(self.get_parameter("ramp").value))
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.omega = 2.0 * math.pi / self.period

        # peak commanded speed along the curve (numeric sweep of one lap)
        peak = max(
            self.omega * math.hypot(self.ax * math.cos(t), 2.0 * self.ay * math.cos(2.0 * t))
            for t in (i * 2.0 * math.pi / 1000.0 for i in range(1000))
        )
        max_speed = float(self.get_parameter("max_speed").value)
        if peak > max_speed:
            raise SystemExit(
                f"fig8_pos_cmd: peak speed {peak:.2f} m/s > max_speed {max_speed:.2f} — "
                f"increase period (need >= {self.period * peak / max_speed:.1f} s) or shrink the 8."
            )

        # anchored on first odometry; nothing is published before that (safe)
        self.center = None        # (x0, y0, z0) ENU
        self.yaw0 = 0.0           # ENU yaw held during the whole 8
        self.tau = 0.0            # curve phase
        self.t = 0.0              # time since anchor
        self.holding = False

        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10)
        self.pub = self.create_publisher(
            PositionCommand, self.get_parameter("out_topic").value, 10)
        self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(
            f"fig8_pos_cmd: {2*self.ax:.1f} x {2*self.ay:.1f} m, {self.period:.0f} s/lap, "
            f"{self.laps:g} laps, peak speed {peak:.2f} m/s — waiting for odometry to anchor..."
        )

    def on_odom(self, msg: Odometry):
        if self.center is not None:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.center = (p.x, p.y, p.z)
        self.yaw0 = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(
            f"anchored at ENU ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), yaw {math.degrees(self.yaw0):.0f} deg — starting"
        )

    def _speed_factor(self) -> float:
        """Smooth 0->1 ramp at start, 1->0 after `laps` laps (then hold)."""
        f = min(1.0, self.t / self.ramp)
        if self.laps > 0.0 and self.tau >= self.laps * 2.0 * math.pi:
            if not self.holding:
                self.holding = True
                self.t_down = self.t
                self.get_logger().info("laps done — ramping down to hover hold")
            f = max(0.0, 1.0 - (self.t - self.t_down) / self.ramp)
        return f

    def on_timer(self):
        if self.center is None:
            return
        dt = 1.0 / self.rate_hz
        self.t += dt
        f = self._speed_factor()
        self.tau += self.omega * f * dt

        x0, y0, z0 = self.center
        s1, c1 = math.sin(self.tau), math.cos(self.tau)
        s2, c2 = math.sin(2.0 * self.tau), math.cos(2.0 * self.tau)
        dtau = self.omega * f

        cmd = PositionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "world"
        cmd.position.x = x0 + self.ax * s1
        cmd.position.y = y0 + self.ay * s2
        cmd.position.z = z0
        cmd.velocity.x = self.ax * c1 * dtau
        cmd.velocity.y = 2.0 * self.ay * c2 * dtau
        cmd.velocity.z = 0.0
        cmd.yaw = self.yaw0
        cmd.yaw_dot = 0.0
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Fig8PosCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
