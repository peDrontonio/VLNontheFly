#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped


class PointPublisher(Node):

    def __init__(self):
        super().__init__('point_publisher')

        self.publisher_ = self.create_publisher(PointStamped, '/waypoint', 10)

        timer_period = 0.5  # segundos
        self.timer = self.create_timer(timer_period, self.publish_point)
        #self.publish_point()

    def publish_point(self):
        msg = PointStamped()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # ou 'base_link', dependendo do seu caso

        # Coordenadas
        msg.point.x = 2.0
        msg.point.y = 2.0
        msg.point.z = 0.0

        self.publisher_.publish(msg)

        #self.get_logger().info(
        #    f'Publicando ponto: ({msg.point.x}, {msg.point.y}, {msg.point.z})'
        #)


def main(args=None):
    rclpy.init(args=args)

    node = PointPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
