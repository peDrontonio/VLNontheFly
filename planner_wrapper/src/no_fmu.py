#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R


class VioBridgeDirect(Node):
    def __init__(self):
        super().__init__('vio_bridge_direct_node')

        self.declare_parameter('camera_pitch_deg', -7.0)
        self.declare_parameter('body_offset_x',     0.0)
        self.declare_parameter('body_offset_y',     0.0)
        self.declare_parameter('body_offset_z',     0.0)

        pitch_deg    = self.get_parameter('camera_pitch_deg').value
        self.t_offset = np.array([
            self.get_parameter('body_offset_x').value,
            self.get_parameter('body_offset_y').value,
            self.get_parameter('body_offset_z').value,
        ])

        # Camera frame → FRD body frame (estática, igual ao nó original)
        R_slam2frd = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0],
        ], dtype=float)
        R_pitch = R.from_euler('y', pitch_deg, degrees=True).as_matrix()
        self.q_cam2body = R.from_matrix(R_pitch @ R_slam2frd)

        self.pose_sub = self.create_subscription(
            PoseStamped, '/orb_slam3/pose', self.on_pose, 10)

        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'VIO Bridge (direct) started | camera_pitch={pitch_deg:.1f} deg')

    def on_pose(self, msg: PoseStamped):
        o    = msg.pose.orientation
        # Tcw: world → camera  (convenção ORB-SLAM3)
        q_cw = R.from_quat([o.x, o.y, o.z, o.w])
        t_cw = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z])

        # Posição da câmera no frame mundo: p_wc = -R_wc · t_cw
        q_wc = q_cw.inv()
        p_wc = -(q_wc.apply(t_cw))

        # Orientação do body no frame mundo:
        #   R_{world←body} = R_{world←cam} · R_{cam←body}
        #                  = q_wc · q_cam2body⁻¹
        q_body = q_wc * self.q_cam2body.inv()

        # Posição do body (FC) = posição câmera − R_{world←body} · lever_arm
        p_body = p_wc - q_body.apply(self.t_offset)

        # ── TF broadcast: map → base_link ──────────────────────────────────
        qb     = q_body.as_quat()   # [x, y, z, w]
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = msg.header.stamp
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id  = 'base_link'
        tf_msg.transform.translation.x = float(p_body[0])
        tf_msg.transform.translation.y = float(p_body[1])
        tf_msg.transform.translation.z = float(p_body[2])
        tf_msg.transform.rotation.x = float(qb[0])
        tf_msg.transform.rotation.y = float(qb[1])
        tf_msg.transform.rotation.z = float(qb[2])
        tf_msg.transform.rotation.w = float(qb[3])
        self.tf_broadcaster.sendTransform(tf_msg)

        # ── VehicleOdometry ────────────────────────────────────────────────
        ts = (msg.header.stamp.sec * 1_000_000
              + msg.header.stamp.nanosec // 1000)

        out = VehicleOdometry()
        out.timestamp        = ts
        out.timestamp_sample = ts
        # Sem ancoragem NED real — usamos UNKNOWN para ser honesto com o PX4.
        out.pose_frame = VehicleOdometry.POSE_FRAME_UNKNOWN

        out.position[0] = float(p_body[0])
        out.position[1] = float(p_body[1])
        out.position[2] = float(p_body[2])

        # PX4 quaternion order: [w, x, y, z]
        out.q[0] = float(qb[3])
        out.q[1] = float(qb[0])
        out.q[2] = float(qb[1])
        out.q[3] = float(qb[2])

        out.velocity_frame          = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity[0]             = out.velocity[1]             = out.velocity[2]             = float('nan')
        out.angular_velocity[0]     = out.angular_velocity[1]     = out.angular_velocity[2]     = float('nan')
        out.position_variance[0]    = out.position_variance[1]    = out.position_variance[2]    = float('nan')
        out.orientation_variance[0] = out.orientation_variance[1] = out.orientation_variance[2] = float('nan')
        out.velocity_variance[0]    = out.velocity_variance[1]    = out.velocity_variance[2]    = float('nan')

        self.odom_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VioBridgeDirect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
