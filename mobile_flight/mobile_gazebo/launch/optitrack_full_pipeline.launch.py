"""
optitrack_full_pipeline.launch.py

Unified launch file that starts the ENTIRE GPS-free flight pipeline using
OptiTrack as the pose source (instead of ORB-SLAM3 / VIO):
  1. MicroXRCEAgent     — serial bridge to the flight controller
  2. RealSense D435i    — full topic set (color/depth/infra/IMU) for recording
  3. OptiTrack Bridge   — PoseStamped (/drone/pose) → PX4 VehicleOdometry
  4. Offboard Control   — cmd_vel → PX4 trajectory setpoints, arming, takeoff, land

Notes:
  - This launch assumes something else (e.g. mocap_optitrack, NatNet ROS driver,
    or a rosbag) is publishing the OptiTrack PoseStamped on `pose_topic`.
  - PX4 talks DDS over serial (/dev/ttyTHS1); MAVProxy talks over USB
    (/dev/ttyACM0) and is launched separately.

Usage:
  ros2 launch mobile_gazebo optitrack_full_pipeline.launch.py

  # Override defaults:
  ros2 launch mobile_gazebo optitrack_full_pipeline.launch.py \
      device:=/dev/ttyTHS1 baudrate:=921600 \
      pose_topic:=/mocap/drone/pose body_offset_x:=0.05
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments ─────────────────────────────────────────────
        # Serial / FC (PX4 DDS)
        DeclareLaunchArgument('device',   default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baudrate', default_value='921600'),
        # OptiTrack bridge
        DeclareLaunchArgument('pose_topic',       default_value='/drone/pose'),
        DeclareLaunchArgument('body_offset_x',    default_value='0.0'),
        DeclareLaunchArgument('body_offset_y',    default_value='0.0'),
        DeclareLaunchArgument('body_offset_z',    default_value='-0.02'),
        DeclareLaunchArgument('publish_velocity', default_value='true'),
        DeclareLaunchArgument('publish_tf',       default_value='true'),

        # ── 1. MicroXRCEAgent (serial to FC) ─────────────────────────────
        ExecuteProcess(
            cmd=[
                'MicroXRCEAgent', 'serial',
                '--dev', LaunchConfiguration('device'),
                '-b',   LaunchConfiguration('baudrate'),
            ],
            output='screen',
        ),

        # ── 2. RealSense D435i (full topics for recording) ──────────────
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'enable_color':      True,
                'enable_depth':      True,
                'enable_infra1':     True,
                'enable_infra2':     True,
                'align_depth.enable': False,
                'enable_gyro':       True,
                'enable_accel':      True,
                'unite_imu_method':  2,      # linear_interpolation
                'infra_width':       640,
                'infra_height':      480,
                'infra_fps':         30,
                'gyro_fps':          200,
                'accel_fps':         250,
                'depth_module.emitter_enabled': 0,
            }],
        ),

        # ── 3. OptiTrack → PX4 bridge ───────────────────────────────────
        Node(
            package='vio_bridge',
            executable='optitrack_bridge_node',
            name='optitrack_bridge_node',
            output='screen',
            parameters=[{
                'pose_topic':       LaunchConfiguration('pose_topic'),
                'body_offset_x':    LaunchConfiguration('body_offset_x'),
                'body_offset_y':    LaunchConfiguration('body_offset_y'),
                'body_offset_z':    LaunchConfiguration('body_offset_z'),
                'publish_velocity': LaunchConfiguration('publish_velocity'),
                'publish_tf':       LaunchConfiguration('publish_tf'),
            }],
        ),

        # ── 4. Offboard velocity control ────────────────────────────────
        Node(
            package='mobile_gazebo',
            executable='offboard_velocity_control',
            name='offboard_velocity_control',
            output='screen',
        ),
    ])
