#!/usr/bin/env python3

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
        Node(
            package='planner',
            executable='iplanner.py',
            name='planner_node',
            parameters=[{"config": config,
                        "checkpoint": checkpoint},
                        parameters
            ],
            remappings=[
                #('/depth', '/camera/camera/depth/image_rect_raw'),
                ('/depth', '/depth/image_raw'),
                ('/camera/image_raw', '/camera/camera/color/image_raw'),
                ('/waypoint', '/vlm/point')
            ],
            output='screen'
        ),
        
        Node(
            package='planner',
            executable='controller.py',
            name='controller_node',
            parameters=[parameters],
            remappings=[

            ]
        ),

        
        Node(
        package='planner',
        executable='odometry_converter.py',
        remappings=[
            ('/vehicle_odometry', '/fmu/out/vehicle_odometry')
        ]
    )#,
    
        #Node(
        #package='planner',
        #executable='point.py',
        #remappings=[
        #    ('/point_topic', '/waypoint')
        #]
    #),
    
        
    #     Node(
    #     package='planner',
    #     executable='no_fmu.py',
    # )
    ])
