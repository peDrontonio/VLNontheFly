#!/usr/bin/env python3
#
# IPlanner launch — OptiTrack test variant.
#
# Flow:
#   optitrack_bridge_node -> /fmu/in/vehicle_visual_odometry -> [PX4 EKF]
#       -> /fmu/out/vehicle_odometry (px4_msgs, NED/FRD)
#       -> odometry_converter_optitrack.py  (NED->ENU)  -> /odometry (nav_msgs)
#       -> iplanner_optitrack.py -> /trajectory -> controller_optitrack.py -> /cmd_vel
#
# The OptiTrack bridge (vio_bridge package) is expected to be launched separately.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('planner')

    config = os.path.join(pkg, 'configs', 'iplanner.yaml')
    checkpoint = os.path.join(pkg, 'checkpoints', 'iplanner.pt')
    parameters = os.path.join(pkg, 'configs', 'parameters.yaml')

    return LaunchDescription([
        # PX4-fused OptiTrack odometry -> nav_msgs/Odometry (ENU) on /odometry
        Node(
            package='planner',
            executable='odometry_converter_optitrack.py',
            name='optitrack_odom_converter',
            parameters=[{
                'world_frame': 'map',
                'body_frame': 'base_link',
                'publish_tf': True,
            }],
            output='screen'
        ),

        Node(
            package='planner',
            executable='iplanner_optitrack.py',
            name='planner_node',
            parameters=[{"config": config,
                        "checkpoint": checkpoint},
                        parameters
            ],
            remappings=[
                #('/depth', '/camera/camera/depth/image_rect_raw'),
                ('/depth', '/depth/image_raw'),
                ('/camera/image_raw', '/camera/camera/color/image_raw'),
                ('/waypoint', '/vlm/point'),
            ],
            output='screen'
        ),

        Node(
            package='planner',
            executable='controller_optitrack.py',
            name='controller_node',
            parameters=[parameters],
            remappings=[
            ]
        ),
    ])
