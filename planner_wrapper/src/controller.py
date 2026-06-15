#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tracking_utils import MPC_Controller

from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from planner.msg import Trajectory2D

import numpy as np
import threading
import os


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('speed', 0.0)
        self.declare_parameter('controller_max_frequency', 60.0)

        self.mutex_odom = threading.Lock() # Odometry value mutex
        self.mutex_trajectory = threading.Lock() # Trajectory mutex
        self.freq_controller = self.get_parameter('controller_max_frequency').get_parameter_value().double_value # Maximum controller frequency in hz
        self.controller_group = MutuallyExclusiveCallbackGroup()
        self.odometry_group = MutuallyExclusiveCallbackGroup()
        self.trajectory_group = MutuallyExclusiveCallbackGroup()

        self.mpc = None
        self.current_odom = None
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.trajectory = None

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry',
            self.update_odometry,
            self.qos,
            callback_group=self.odometry_group
        )

        self.trajectory_sub = self.create_subscription(
            Trajectory2D,
            '/trajectory',
            self.update_trajectory,
            self.qos,
            callback_group = self.trajectory_group
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.control_loop = self.create_timer(
            1.0 / self.freq_controller,
            self.controller,
            callback_group=self.controller_group
        )

        self.get_logger().info("Controller node started.")
    
    def controller(self):
        with self.mutex_odom:
            odom = self.current_odom
        
        if odom is None:
            #self.get_logger().warn('(Controller): No odometry detected for control.')
            return

        with self.mutex_trajectory:
            mpc = self.mpc
                
        if mpc is None:
            #self.get_logger().warn('(Controller): No trajectory detected for control.')
            return
        
        # position
        pos = odom.pose.pose.position
        camera_pos = np.array([pos.x, pos.y, pos.z])

        # orientation
        quat = odom.pose.pose.orientation
        camera_rot = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
        
        x0 = np.array([camera_pos[0], camera_pos[1], np.arctan2(camera_rot[1,0], camera_rot[0,0])])
        opt_u_controls, opt_x_states = mpc.solve(x0)
        v, w = opt_u_controls[1, 0], opt_u_controls[1, 1]

        # Body-frame Twist (FRD): linear.x is forward speed, angular.z is yaw rate.
        # The offboard node rotates body→NED using PX4 heading, so do NOT pre-rotate here.
        cmd_vel = Twist()
        cmd_vel.linear.x = float(v)
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = float(w)

        # cmd_vel.linear.x = float(v * np.cos(theta))
        # cmd_vel.linear.y = float(v * np.sin(theta))


        self.cmd_vel_pub.publish(cmd_vel)


    def update_odometry(self, odom):
        with self.mutex_odom:
            self.current_odom = odom

    def update_trajectory(self, msg):
        with self.mutex_trajectory:
            trajectory = np.array(msg.data, dtype=np.float32).reshape(msg.rows, msg.cols)
            self.mpc = MPC_Controller(trajectory,
                        desired_v=self.speed,
                        v_max=self.speed,
                        w_max=self.speed)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
