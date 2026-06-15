"""
vio_full_pipeline.launch.py

Unified launch file that starts the ENTIRE GPS-free flight pipeline:
  1. MicroXRCEAgent     — serial bridge to the flight controller
  2. RealSense D435i    — infra1/infra2 stereo + fused IMU
  3. ORB-SLAM3          — stereo-inertial SLAM (delayed 3 s for camera warm-up)
  4. VIO Bridge         — translates SLAM PoseStamped → PX4 VehicleOdometry
  5. Offboard Control   — cmd_vel → PX4 trajectory setpoints, arming, takeoff, land

Usage:
  ros2 launch mobile_gazebo vio_full_pipeline.launch.py

  # Override defaults:
  ros2 launch mobile_gazebo vio_full_pipeline.launch.py \
      device:=/dev/ttyUSB0 baudrate:=460800 \
      camera_pitch_deg:=-83.0 body_offset_x:=0.05
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # ── Paths (sensible defaults for this Jetson Orin setup) ──────────────
    default_vocab = os.path.expanduser(
        '~/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    default_settings = os.path.expanduser(
        '~/ros2_ws/src/ORB_SLAM3/Examples/Stereo-Inertial/RealSense_D435i.yaml')

    return LaunchDescription([
        # ── Launch arguments ─────────────────────────────────────────────
        # Serial / FC
        DeclareLaunchArgument('device',    default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baudrate',  default_value='921600'),
        # ORB-SLAM3
        DeclareLaunchArgument('vocab_path',    default_value=default_vocab),
        DeclareLaunchArgument('settings_path', default_value=default_settings),
        DeclareLaunchArgument('do_rectify',    default_value='false'),
        DeclareLaunchArgument('do_equalize',   default_value='false'),
        # VIO Bridge
        DeclareLaunchArgument('camera_pitch_deg', default_value='-7.0'),
        DeclareLaunchArgument('body_offset_x',    default_value='0.11'),
        DeclareLaunchArgument('body_offset_y',    default_value='0.0'),
        DeclareLaunchArgument('body_offset_z',    default_value='0.0'),

        # ── 1. MicroXRCEAgent (serial to FC) ─────────────────────────────
        ExecuteProcess(
           cmd=[
               'MicroXRCEAgent', 'serial',
               '--dev', LaunchConfiguration('device'),
               '-b',   LaunchConfiguration('baudrate'),
           ],
           output='screen',
        ),

        # ── 2. RealSense D435i camera ────────────────────────────────────
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'enable_color':     True,
                'enable_depth':     True,
                'enable_infra1':    True,
                'align_depth.enable': False,
                'enable_infra2':    True,
                'enable_gyro':      True,
                'enable_accel':     True,
                'unite_imu_method': 2,      # linear_interpolation
                'infra_width':      640,
                'infra_height':     480,
                'infra_fps':        30,
                'gyro_fps':         200,
                'accel_fps':        250,
                'depth_module.emitter_enabled': 0, # Desliga o projetor de pontos IR para nao bugar o SLAM
                'publish_mount_tf': True,
                'mount_parent_frame_id': 'base_link',
                'mount_x': LaunchConfiguration('body_offset_x'),
                # VIO bridge mount offsets are FRD; ROS base_link is FLU.
                'mount_y': PythonExpression([
                    'str(-float("', LaunchConfiguration('body_offset_y'), '"))'
                ]),
                'mount_z': PythonExpression([
                    'str(-float("', LaunchConfiguration('body_offset_z'), '"))'
                ]),
                'mount_roll': 0.0,
                'mount_pitch': PythonExpression([
                    'str(-float("', LaunchConfiguration('camera_pitch_deg'),
                    '") * 3.141592653589793 / 180.0)'
                ]),
                'mount_yaw': 0.0,
            }],
        ),

        # ── 3. ORB-SLAM3 stereo-inertial (delayed 3 s) ──────────────────
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='orb_slam3_ros2',
                    executable='stereo_inertial',
                    name='orb_slam3_stereo_inertial',
                    output='screen',
                    arguments=[
                        LaunchConfiguration('vocab_path'),
                        LaunchConfiguration('settings_path'),
                        LaunchConfiguration('do_rectify'),
                        LaunchConfiguration('do_equalize'),
                    ],
                    remappings=[
                        ('/camera/left/image_raw',
                         '/camera/camera/infra1/image_rect_raw'),
                        ('/camera/right/image_raw',
                         '/camera/camera/infra2/image_rect_raw'),
                        ('/imu', '/camera/camera/imu'),
                    ],
                ),
            ],
        ),

        # ── 4. VIO Bridge (delayed 5 s — needs SLAM to start first) ─────
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='vio_bridge',
                    executable='vio_bridge_node',
                    name='vio_bridge_node',
                    output='screen',
                    parameters=[{
                        'camera_pitch_deg': LaunchConfiguration('camera_pitch_deg'),
                        'body_offset_x':    LaunchConfiguration('body_offset_x'),
                        'body_offset_y':    LaunchConfiguration('body_offset_y'),
                        'body_offset_z':    LaunchConfiguration('body_offset_z'),
                    }],
                ),
            ],
        ),

        Node(
            package='mobile_gazebo',
            executable='offboard_velocity_control',
            name='offboard_velocity_control',
            output='screen',
        ),
    ])
