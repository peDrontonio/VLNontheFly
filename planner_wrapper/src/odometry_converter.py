#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

import math

import numpy as np
from scipy.spatial.transform import Rotation as R


class VehicleOdomConverter(Node):

    def __init__(self):
        super().__init__('vehicle_odom_converter')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.body_frame = self.get_parameter('body_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        # QoS compatível com PX4
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            VehicleOdometry,
            '/vehicle_odometry',
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
            "VehicleOdometry -> Odometry converter started "
            f"| map_frame={self.map_frame} | body_frame={self.body_frame} "
            f"| publish_tf={self.publish_tf}"
        )

    def callback(self, msg: VehicleOdometry):
        odom = Odometry()

        # =========================
        # HEADER
        # =========================
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.map_frame
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
        # PX4: [w, x, y, z] (NED)
        # scipy: [x, y, z, w]
        # =========================
        q = msg.q

        # Conversão (body FRD -> NED) -> (body FLU -> ENU):
        #   R_enu = T_enu<-ned @ R_ned @ D_frd<-flu
        # T sozinho (T @ R @ T.T) deixa o yaw com offset de -90 graus (eixo x
        # do corpo viraria "direita" em vez de "frente").
        rot_ned = R.from_quat([q[1], q[2], q[3], q[0]])
        T = np.array([[0,1,0],
                      [1,0,0],
                      [0,0,-1]])   # ENU <- NED (mundo)
        D = np.diag([1.0,-1.0,-1.0])  # FRD <- FLU (corpo)
        rot_enu = R.from_matrix(T @ rot_ned.as_matrix() @ D)
        q_enu = rot_enu.as_quat()  # x, y, z, w

        # ---- DEBUG: raw PX4 input vs converted output (throttled ~1 Hz) ----
        # Identity NED q=[1,0,0,0] -> OUT yaw +90 deg is correct (NED-North =
        # ENU +Y). If IN q is NaN/garbage, suspect a px4_msgs ABI mismatch
        # against the firmware publishing /fmu/out/vehicle_odometry.
        yaw_in = math.degrees(rot_ned.as_euler('zyx')[0])
        yaw_out = math.degrees(rot_enu.as_euler('zyx')[0])
        self.get_logger().info(
            f"IN q[w,x,y,z]=[{q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f}] "
            f"(NED yaw {yaw_in:+.1f}) -> OUT yaw {yaw_out:+.1f} deg "
            f"(ENU q[x,y,z,w]=[{q_enu[0]:+.3f},{q_enu[1]:+.3f},{q_enu[2]:+.3f},{q_enu[3]:+.3f}])",
            throttle_duration_sec=1.0,
        )

        odom.pose.pose.orientation = Quaternion(
            x=float(q_enu[0]),
            y=float(q_enu[1]),
            z=float(q_enu[2]),
            w=float(q_enu[3])
        )
        # =========================
        # VELOCIDADE LINEAR (NED → ENU)
        # =========================
        vx = msg.velocity[1]
        vy = msg.velocity[0]
        vz = -msg.velocity[2]

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = float(vz)

        # =========================
        # VELOCIDADE ANGULAR (corpo FRD -> FLU)
        # =========================
        wx = msg.angular_velocity[0]
        wy = -msg.angular_velocity[1]
        wz = -msg.angular_velocity[2]

        odom.twist.twist.angular.x = float(wx)
        odom.twist.twist.angular.y = float(wy)
        odom.twist.twist.angular.z = float(wz)

        # =========================
        # PUBLICA
        # =========================
        self.pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.map_frame
            tf_msg.child_frame_id = self.body_frame
            tf_msg.transform.translation.x = float(x)
            tf_msg.transform.translation.y = float(y)
            tf_msg.transform.translation.z = float(z)
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleOdomConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
