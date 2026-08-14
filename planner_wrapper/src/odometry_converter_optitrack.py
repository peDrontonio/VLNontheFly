#!/usr/bin/env python3
#
# OptiTrack odometry converter (PX4-fused source).
#
# Reads the PX4 EKF output (px4_msgs/VehicleOdometry on /fmu/out/vehicle_odometry,
# NED world / FRD body) — which already fuses the OptiTrack visual odometry coming
# in through the optitrack_bridge_node — and republishes it as nav_msgs/Odometry in
# an ENU world frame / FLU body so the planner can consume it on /odometry.
#
# Optionally broadcasts the map -> base_link TF so the planner's goal lookup (which
# transforms waypoints into the "map" frame) and RViz stay consistent.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

import numpy as np
from scipy.spatial.transform import Rotation as R


class OptitrackOdomConverter(Node):

    def __init__(self):
        super().__init__('optitrack_odom_converter')

        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.body_frame = self.get_parameter('body_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        # QoS compatível com PX4 (tópicos /fmu/out são BEST_EFFORT)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # NED -> ENU (mesma troca aplicada à posição, velocidade e orientação)
        self.T = np.array([[0, 1, 0],
                           [1, 0, 0],
                           [0, 0, -1]])

        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.callback,
            qos
        )

        self.pub = self.create_publisher(
            Odometry,
            '/odometry',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            "OptiTrack VehicleOdometry (/fmu/out) -> Odometry (/odometry) converter started "
            f"| world_frame={self.world_frame} | body_frame={self.body_frame} "
            f"| publish_tf={self.publish_tf}"
        )

    def callback(self, msg: VehicleOdometry):
        odom = Odometry()

        # =========================
        # HEADER
        # =========================
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame

        # =========================
        # NED → ENU (POSIÇÃO)
        # =========================
        x = msg.position[1]
        y = msg.position[0]
        z = -msg.position[2]

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = float(z)

        # =========================
        # ORIENTAÇÃO (quaternion)
        # PX4: [w, x, y, z] (NED) | scipy: [x, y, z, w]
        # =========================
        q = msg.q
        rot_ned = R.from_quat([q[1], q[2], q[3], q[0]])
        # (body FRD -> NED) -> (body FLU -> ENU): T @ R @ D; T @ R @ T.T
        # deixaria o yaw com offset de -90 graus.
        D = np.diag([1.0, -1.0, -1.0])  # FRD <- FLU (corpo)
        rot_enu = R.from_matrix(self.T @ rot_ned.as_matrix() @ D)
        q_enu = rot_enu.as_quat()  # x, y, z, w

        odom.pose.pose.orientation = Quaternion(
            x=float(q_enu[0]),
            y=float(q_enu[1]),
            z=float(q_enu[2]),
            w=float(q_enu[3])
        )

        # =========================
        # VELOCIDADE LINEAR (NED → ENU)
        # =========================
        odom.twist.twist.linear.x = float(msg.velocity[1])
        odom.twist.twist.linear.y = float(msg.velocity[0])
        odom.twist.twist.linear.z = float(-msg.velocity[2])

        # =========================
        # VELOCIDADE ANGULAR (corpo FRD -> FLU)
        # =========================
        odom.twist.twist.angular.x = float(msg.angular_velocity[0])
        odom.twist.twist.angular.y = float(-msg.angular_velocity[1])
        odom.twist.twist.angular.z = float(-msg.angular_velocity[2])

        self.pub.publish(odom)

        # =========================
        # TF map -> base_link (opcional)
        # =========================
        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.world_frame
            tf_msg.child_frame_id = self.body_frame
            tf_msg.transform.translation.x = float(x)
            tf_msg.transform.translation.y = float(y)
            tf_msg.transform.translation.z = float(z)
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackOdomConverter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
