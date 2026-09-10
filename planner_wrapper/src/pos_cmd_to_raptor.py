#!/usr/bin/env python3
"""
pos_cmd_to_raptor.py — ego-planner PositionCommand -> Raptor EXTERNAL setpoint bridge.

ego-planner's traj_server publishes quadrotor_msgs/PositionCommand on
`/drone_0_planning/pos_cmd` at ~100 Hz, in the planner WORLD frame (ENU).
This node forwards each command 1:1 (no timer, no resampling — output rate is
traj_server's native 100 Hz) as a px4_msgs/TrajectorySetpoint in NED on
`/fmu/in/trajectory_setpoint_raptor`, which the rl_tools_commander EXTERNAL
mode consumes over uXRCE-DDS.

    pos_cmd (ENU)                      TrajectorySetpoint (NED)
      position (x,y,z)        ->         (y, x, -z)
      velocity (x,y,z)        ->         (y, x, -z)
      yaw (ENU, 0=East,CCW)   ->         wrap_pi(pi/2 - yaw)  (NED, 0=North,CW)

This node has NO authority over activation. Raptor is engaged/disengaged only
by the operator's RC switch. Being purely event-driven, it publishes nothing
before the first pos_cmd and stops the instant the planner stops — the
firmware's 300 ms staleness freeze then holds the last target.

Frame note: ego-planner's /odometry must come from the PX4 EKF
(odometry_converter.py on /fmu/out/vehicle_odometry) so both sides share the
same local origin. Keep the conversions here consistent with
odometry_converter.py and ego_planner_bridge.py.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from quadrotor_msgs.msg import PositionCommand
from px4_msgs.msg import TrajectorySetpoint

NAN = float("nan")


def wrap_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class PosCmdToRaptor(Node):
    def __init__(self):
        super().__init__("pos_cmd_to_raptor")

        self.declare_parameter("pos_cmd_topic", "/drone_0_planning/pos_cmd")
        self.declare_parameter("out_topic", "/fmu/in/trajectory_setpoint_raptor")
        pos_cmd_topic = self.get_parameter("pos_cmd_topic").value
        out_topic = self.get_parameter("out_topic").value

        # PX4 uXRCE QoS: best-effort, transient-local, depth 1
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(TrajectorySetpoint, out_topic, px4_qos)
        self.create_subscription(PositionCommand, pos_cmd_topic, self.on_pos_cmd, 50)

        self.get_logger().info(
            f"pos_cmd_to_raptor: {pos_cmd_topic} (ENU) -> {out_topic} (NED), "
            "1:1 forwarding at the pos_cmd rate (~100 Hz)"
        )

    def on_pos_cmd(self, cmd: PositionCommand):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
        msg.position = [
            float(cmd.position.y),
            float(cmd.position.x),
            float(-cmd.position.z),
        ]
        msg.velocity = [
            float(cmd.velocity.y),
            float(cmd.velocity.x),
            float(-cmd.velocity.z),
        ]
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        msg.yaw = float(wrap_pi(math.pi / 2.0 - cmd.yaw))
        msg.yawspeed = NAN
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PosCmdToRaptor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
