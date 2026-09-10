#!/usr/bin/env python3
"""
ego_planner_bridge.py — ego-planner PositionCommand -> PX4 velocity bridge.

ego-planner's traj_server publishes a quadrotor_msgs/PositionCommand on
`/drone_0_planning/pos_cmd` at ~100 Hz. PositionCommand carries position,
velocity, acceleration, yaw and yaw_dot, all expressed in the planner WORLD
frame, which is ENU (X-East, Y-North, Z-Up) — the same frame the grid map,
the /odometry topic and RViz use.

offboard_velocity_control exposes a `set_velocity` service that, when called
with frame_id="ned", forwards (vx, vy, vz, yaw) straight to PX4 as a NED
velocity setpoint (it skips the body->NED yaw rotation it would otherwise
apply to /cmd_vel). So this node:

  * subscribes  <pos_cmd_topic>   (quadrotor_msgs/PositionCommand, ENU world)
  * converts the ENU velocity + yaw to NED:
        vx_ned  =  vel.y           (North =  ENU Y)
        vy_ned  =  vel.x           (East  =  ENU X)
        vz_ned  = -vel.z           (Down  = -ENU Z)
        yaw_ned =  pi/2 - yaw_enu  (NED yaw: 0=North, +clockwise)
  * calls set_velocity(frame_id="ned") asynchronously at <= forward_rate_hz.

Each call carries a short `command_duration` so PX4 hovers in place if the
planner stream stops — defence in depth on top of offboard_velocity_control's
own 5 s companion-heartbeat failsafe.

Frame note: this bridge does NOT touch TF. The planner world is ENU; PX4 wants
NED. The only conversion is the velocity/yaw swap above. The pose side of that
ENU<->NED relationship lives in optitrack_bridge_node (pose -> PX4) and
odometry_converter.py (PX4 -> /odometry). Keep all three consistent.
"""

import math

import rclpy
from rclpy.node import Node

from quadrotor_msgs.msg import PositionCommand
from mobile_msgs.srv import SetVelocity


def wrap_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class EgoPlannerBridge(Node):
    def __init__(self):
        super().__init__('ego_planner_bridge')

        # ---- parameters ----
        self.pos_cmd_topic = self.declare_parameter(
            'pos_cmd_topic', '/drone_0_planning/pos_cmd').value
        # NOTE: offboard_velocity_control creates the service with a RELATIVE
        # name, so under the default namespace it is `/set_velocity`. If you
        # launch the offboard node under a namespace, override this. Verify with
        #   ros2 service list | grep set_velocity
        self.service_name = self.declare_parameter(
            'set_velocity_service', '/set_velocity').value
        self.forward_rate_hz = float(self.declare_parameter(
            'forward_rate_hz', 50.0).value)
        self.command_duration = float(self.declare_parameter(
            'command_duration', 0.15).value)
        self.auto_arm = bool(self.declare_parameter('auto_arm', False).value)
        # Stop forwarding if no fresh pos_cmd for this long -> PX4 falls back to
        # hover via the offboard node's own failsafe.
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.5).value)

        # ---- state ----
        self._last_cmd = None          # (vx_ned, vy_ned, vz_ned, yaw_ned)
        self._last_cmd_time = None     # rclpy.time.Time
        self._pending = None           # in-flight service future

        # ---- ROS I/O ----
        self.cli = self.create_client(SetVelocity, self.service_name)
        self.sub = self.create_subscription(
            PositionCommand, self.pos_cmd_topic, self.on_pos_cmd, 50)
        self.timer = self.create_timer(1.0 / self.forward_rate_hz, self.on_timer)

        self.get_logger().info(
            f"ego_planner_bridge | in='{self.pos_cmd_topic}' "
            f"-> service='{self.service_name}' (frame=ned) "
            f"@ {self.forward_rate_hz:.0f} Hz | auto_arm={self.auto_arm}")

    def on_pos_cmd(self, msg: PositionCommand):
        # ENU world velocity / yaw -> NED.
        vx_ned = msg.velocity.y
        vy_ned = msg.velocity.x
        vz_ned = -msg.velocity.z
        yaw_ned = wrap_pi(math.pi / 2.0 - msg.yaw)
        self._last_cmd = (vx_ned, vy_ned, vz_ned, yaw_ned)
        self._last_cmd_time = self.get_clock().now()

    def on_timer(self):
        if self._last_cmd is None or self._last_cmd_time is None:
            return

        # Stale command -> stop forwarding, let offboard failsafe hover.
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            return

        if not self.cli.service_is_ready():
            self.get_logger().warn(
                f"set_velocity service '{self.service_name}' not available yet.",
                throttle_duration_sec=2.0)
            return

        # Don't pile up requests: skip if the previous call hasn't returned.
        if self._pending is not None and not self._pending.done():
            return

        vx, vy, vz, yaw = self._last_cmd
        req = SetVelocity.Request()
        req.vx = float(vx)
        req.vy = float(vy)
        req.vz = float(vz)
        req.yaw = float(yaw)
        req.duration = float(self.command_duration)
        req.frame_id = 'ned'          # bypass the body->NED rotation in offboard
        req.auto_arm = bool(self.auto_arm)
        self._pending = self.cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = EgoPlannerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
