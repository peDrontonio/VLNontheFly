from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vio_bridge',
            executable='vio_bridge_node',
            name='vio_bridge_node',
            output='screen',
            parameters=[{
                # Camera is mounted 20° nose-down relative to drone body (calibrated with OptiTrack).
                'camera_pitch_deg': -20.0,

                # Scale factor: real-world metric / SLAM internal units (calibrated with OptiTrack).
                # gt_path_length / slam_path_length measured on bag_optitrack_1.
                'slam_scale': 0.61,

                # Camera is 11 cm ahead of the flight controller (center of rotation).
                # X = forward in FRD body frame.
                'body_offset_x': 0.11,
                'body_offset_y': 0.0,
                'body_offset_z': 0.0,
            }],
        ),
    ])
