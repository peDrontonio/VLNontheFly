"""
flight_stack.launch.py — canonical entry point for the cleaned-up pipeline.

Brings up, in order:
  1. MicroXRCEAgent      (serial bridge to the flight controller)         [toggle: with_dds]
  2. RealSense D435i     (stereo IR + fused IMU + depth + color)
  3. ORB-SLAM3           (stereo-inertial, delayed 3 s for camera warm-up)
  4. vio_bridge          (SLAM PoseStamped → /fmu/in/vehicle_visual_odometry, NED)
  5. offboard_velocity_control  (/cmd_vel → PX4 trajectory setpoints + services)
  6. iplanner + controller + odometry_converter (planner_wrapper)           [toggle: with_planner]

Frame contract (after the recent cleanup):
  - /orb_slam3/pose                       PoseStamped, SLAM world frame   (orb_slam3_ros2 only)
  - /fmu/in/vehicle_visual_odometry       VehicleOdometry, NED            (vio_bridge only)
  - /cmd_vel                              Twist, BODY (FRD)               (controller only)
  - /fmu/in/trajectory_setpoint           NED                              (offboard only)

Common invocations
------------------
Full real-flight stack (default):
  ros2 launch mobile_gazebo flight_stack.launch.py

Bench test (no FC connected, no planner, estimator/battery guards bypassed):
  ros2 launch mobile_gazebo flight_stack.launch.py with_dds:=false \
      with_planner:=false bench_mode:=true

Manual flight (no autonomous planner, real FC):
  ros2 launch mobile_gazebo flight_stack.launch.py with_planner:=false

Override frame geometry (camera tilt / lever arm in body frame):
  ros2 launch mobile_gazebo flight_stack.launch.py \
      camera_pitch_deg:=-7.0 body_offset_x:=0.11
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # ── Default paths (Jetson Orin layout) ───────────────────────────────────
    default_vocab = os.path.expanduser(
        '~/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    default_settings = os.path.expanduser(
        '~/ros2_ws/src/ORB_SLAM3/Examples/Stereo-Inertial/RealSense_D435i.yaml')

    # ── Launch arguments ─────────────────────────────────────────────────────
    args = [
        # Toggles
        DeclareLaunchArgument('with_dds',     default_value='true',
            description='Start MicroXRCEAgent (false = no FC connected, bench only).'),
        DeclareLaunchArgument('with_planner', default_value='true',
            description='Start iplanner + controller + odometry_converter.'),
        DeclareLaunchArgument('bench_mode',   default_value='false',
            description='Bypass estimator/battery guards in offboard_velocity_control. '
                        'Set true ONLY for ground testing.'),

        # FC serial link
        DeclareLaunchArgument('device',   default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baudrate', default_value='921600'),

        # ORB-SLAM3
        DeclareLaunchArgument('vocab_path',    default_value=default_vocab),
        DeclareLaunchArgument('settings_path', default_value=default_settings),
        DeclareLaunchArgument('do_rectify',    default_value='false'),
        DeclareLaunchArgument('do_equalize',   default_value='false'),

        # VIO bridge geometry
        DeclareLaunchArgument('camera_pitch_deg', default_value='-7.0'),
        DeclareLaunchArgument('body_offset_x',    default_value='0.11'),
        DeclareLaunchArgument('body_offset_y',    default_value='0.0'),
        DeclareLaunchArgument('body_offset_z',    default_value='0.0'),
    ]

    # ── 1. MicroXRCEAgent (serial → FC) ──────────────────────────────────────
    uxrce_agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'serial',
            '--dev', LaunchConfiguration('device'),
            '-b',   LaunchConfiguration('baudrate'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_dds')),
    )

    # ── 2. RealSense D435i ───────────────────────────────────────────────────
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_color':       True,
            'enable_depth':       True,
            'enable_infra1':      True,
            'enable_infra2':      True,
            'align_depth.enable': True,
            'enable_gyro':        True,
            'enable_accel':       True,
            'unite_imu_method':   2,      # linear_interpolation
            'infra_width':        640,
            'infra_height':       480,
            'infra_fps':          15,
            'gyro_fps':           200,
            'accel_fps':          100,
            'publish_mount_tf':   True,
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
    )

    # ── 3. ORB-SLAM3 stereo-inertial (delayed 3 s) ───────────────────────────
    orb_slam3 = TimerAction(period=3.0, actions=[
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
                ('/camera/left/image_raw',  '/camera/camera/infra1/image_rect_raw'),
                ('/camera/right/image_raw', '/camera/camera/infra2/image_rect_raw'),
                ('/imu',                    '/camera/camera/imu'),
            ],
        ),
    ])

    # ── 4. vio_bridge (delayed 5 s — needs SLAM running) ─────────────────────
    vio_bridge = TimerAction(period=5.0, actions=[
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
    ])

    # ── 5. Offboard velocity control ─────────────────────────────────────────
    offboard = Node(
        package='mobile_gazebo',
        executable='offboard_velocity_control',
        name='offboard_velocity_control',
        output='screen',
        parameters=[{
            'bench_mode': LaunchConfiguration('bench_mode'),
        }],
    )

    # ── 6. Planner stack (iplanner + controller + odometry_converter) ────────
    # Gated behind with_planner — kept off for manual flight or bench tests.
    planner_share = get_package_share_directory('planner')
    planner_config     = os.path.join(planner_share, 'configs', 'iplanner.yaml')
    planner_checkpoint = os.path.join(planner_share, 'checkpoints', 'iplanner.pt')
    planner_parameters = os.path.join(planner_share, 'configs', 'parameters.yaml')

    planner_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('with_planner')),
        actions=[
            Node(
                package='planner',
                executable='odometry_converter.py',
                name='odometry_converter',
                output='screen',
                remappings=[
                    # vio_bridge writes here; converter republishes as nav_msgs/Odometry on /odometry.
                    ('/vehicle_odometry', '/fmu/in/vehicle_visual_odometry'),
                ],
            ),
            Node(
                package='planner',
                executable='iplanner.py',
                name='planner_node',
                output='screen',
                parameters=[
                    {'config': planner_config, 'checkpoint': planner_checkpoint},
                    planner_parameters,
                ],
                remappings=[
                    ('/depth',            '/depth/image_raw'),
                    ('/camera/image_raw', '/camera/camera/color/image_raw'),
                ],
            ),
            Node(
                package='planner',
                executable='controller.py',
                name='controller_node',
                output='screen',
                parameters=[planner_parameters],
                # controller publishes /cmd_vel in BODY frame (FRD) — see CHANGES.md
            ),
        ],
    )

    return LaunchDescription([
        *args,
        uxrce_agent,
        realsense,
        orb_slam3,
        vio_bridge,
        offboard,
        planner_group,
    ])
