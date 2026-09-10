"""
ego_planner_flight.launch.py — ego-planner-swarm on real hardware, MOCAP odometry.

This is the ORB-SLAM-free variant of the flight stack: the odometry source is a
motion-capture system (OptiTrack/VICON via a VRPN or NatNet client) instead of
stereo-inertial SLAM. Everything downstream of the odometry source is unchanged.

Pipeline
--------
  1. MicroXRCEAgent            serial bridge to the flight controller   [with_dds]
  2. RealSense D435i           DEPTH (+ color) — obstacle input for the grid map
  3. optitrack_bridge_node     /drone/pose (mocap) -> /fmu/in/vehicle_visual_odometry (NED)
  4. offboard_velocity_control set_velocity / arm / takeoff / land / estop
  5. odometry_converter.py     <planner_odom_topic> (NED) -> /odometry (ENU "map")
  6. ego_planner + traj_server real_single_drone.launch.py -> /drone_0_planning/pos_cmd
  7. ego_planner_bridge.py     PositionCommand (ENU) -> set_velocity (NED)

What changed vs. the ORB-SLAM stack (flight_stack.launch.py)
------------------------------------------------------------
  - ORB-SLAM3 + vio_bridge_node  ->  mocap client + optitrack_bridge_node.
  - RealSense IR projector may stay ON now (no stereo SLAM to corrupt); we use
    the depth stream, not infra+IMU.

⚠ FIXED-FRAME CONTRACT — read before flying
--------------------------------------------
  - The real-flight planner lives entirely in the ENU **map** frame:
    grid_map/frame_id, FSM markers, trajectory commands, and /odometry all use
    "map". This launch overrides the shared RViz preset to **Fixed Frame = map**.
    Do NOT use "world" or "odom_ned" for real flight. "world" is retained only
    by simulation configurations; "odom_ned" uses PX4's NED convention.
  - optitrack_bridge_node would otherwise broadcast a NED TF
    (odom_ned -> base_link_frd) that is disconnected from the ENU "map" tree.
    We launch it with publish_tf:=false to keep the TF tree clean. The grid map
    does NOT need TF — it reads the body pose straight from /odometry
    (grid_map/pose_type=2) and applies its own fixed camera extrinsic.
  - The "map" axes are anchored to the drone's heading at bridge startup
    (North = initial forward), origin at the first mocap sample. RViz goals are
    relative to that frame, same as the old ORB-SLAM behaviour.

Prerequisite (external, NOT started here)
-----------------------------------------
  A mocap client must publish the rigid body as geometry_msgs/PoseStamped on
  `/drone/pose` (world Z-up, body FLU). e.g. vrpn_mocap / mocap4r2 / NatNet.
  Override the topic with pose_topic:=...

Examples
--------
  # full real flight (FC connected, mocap client already running)
  ros2 launch mobile_gazebo ego_planner_flight.launch.py

  # ground bench (no FC, estimator guards bypassed)
  ros2 launch mobile_gazebo ego_planner_flight.launch.py \
      with_dds:=false bench_mode:=true

  # feed the planner from PX4's fused EKF2 output instead of the raw mocap bridge
  ros2 launch mobile_gazebo ego_planner_flight.launch.py \
      planner_odom_topic:=/fmu/out/vehicle_odometry
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, TimerAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ego_share = get_package_share_directory('ego_planner')
    real_drone_launch = os.path.join(ego_share, 'launch', 'real_single_drone.launch.py')
    rviz_config = os.path.join(ego_share, 'launch', 'default.rviz')

    args = [
        # Toggles
        DeclareLaunchArgument('with_dds',   default_value='true',
            description='Start MicroXRCEAgent (false = no FC connected, bench only).'),
        DeclareLaunchArgument('bench_mode', default_value='false',
            description='Bypass estimator/battery guards in offboard_velocity_control. '
                        'Ground testing ONLY.'),
        DeclareLaunchArgument('with_rviz',  default_value='true',
            description='Open RViz with the shared ego_planner config overridden to Fixed Frame=map.'),

        # FC serial link
        DeclareLaunchArgument('device',   default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baudrate', default_value='921600'),

        # Mocap -> PX4 bridge
        DeclareLaunchArgument('pose_topic', default_value='/drone/pose',
            description='geometry_msgs/PoseStamped from the mocap client (world Z-up, body FLU).'),
        DeclareLaunchArgument('body_offset_x', default_value='0.0',
            description='Rigid-body origin -> FC centre offset, FRD body frame [m].'),
        DeclareLaunchArgument('body_offset_y', default_value='0.0'),
        DeclareLaunchArgument('body_offset_z', default_value='0.0'),

        # Which NED odometry feeds the planner.
        #   /fmu/in/vehicle_visual_odometry  = raw mocap bridge output (lowest latency, default)
        #   /fmu/out/vehicle_odometry        = PX4 EKF2-fused estimate
        DeclareLaunchArgument('planner_odom_topic',
            default_value='/fmu/in/vehicle_visual_odometry',
            description='NED VehicleOdometry topic that odometry_converter republishes as /odometry.'),

        # ego_planner_bridge
        DeclareLaunchArgument('set_velocity_service', default_value='/set_velocity',
            description='set_velocity service name. Verify: ros2 service list | grep set_velocity'),
    ]

    # ── 1. MicroXRCEAgent (serial → FC) ──────────────────────────────────────
    uxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial',
             '--dev', LaunchConfiguration('device'),
             '-b',   LaunchConfiguration('baudrate')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_dds')),
    )

    # ── 2. RealSense D435i — DEPTH for the grid map (IR projector may stay ON) ─
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_color':       True,
            'enable_depth':       True,
            'enable_infra1':      False,
            'enable_infra2':      False,
            'align_depth.enable': False,   # grid map uses raw depth intrinsics
            'enable_gyro':        False,
            'enable_accel':       False,
            # IMPORTANT: the grid_map intrinsics baked into real_single_drone.launch.py
            # (fx=fy=382.613, cx=320.183, cy=236.455) are for 640x480 depth. Make sure
            # the depth stream runs at 640x480, or update those intrinsics to match.
            # The profile parameter name differs across realsense2_camera versions
            # (depth_module.profile vs depth_module.depth_profile) — set it for your
            # driver build rather than hardcoding a possibly-wrong key here.
        }],
    )

    # ── 3. Mocap → PX4 bridge (publish_tf OFF: keep TF tree clean, see header) ─
    optitrack_bridge = Node(
        package='vio_bridge',
        executable='optitrack_bridge_node',
        name='optitrack_bridge_node',
        output='screen',
        parameters=[{
            'pose_topic':       LaunchConfiguration('pose_topic'),
            'body_offset_x':    LaunchConfiguration('body_offset_x'),
            'body_offset_y':    LaunchConfiguration('body_offset_y'),
            'body_offset_z':    LaunchConfiguration('body_offset_z'),
            'publish_velocity': True,
            'publish_tf':       False,
        }],
    )

    # ── 4. Offboard velocity control ─────────────────────────────────────────
    offboard = Node(
        package='mobile_gazebo',
        executable='offboard_velocity_control',
        name='offboard_velocity_control',
        output='screen',
        parameters=[{'bench_mode': LaunchConfiguration('bench_mode')}],
    )

    # ── 5. NED VehicleOdometry -> ENU /odometry (nav_msgs/Odometry, "map") ────
    odometry_converter = Node(
        package='planner',
        executable='odometry_converter.py',
        name='odometry_converter',
        output='screen',
        remappings=[('/vehicle_odometry', LaunchConfiguration('planner_odom_topic'))],
    )

    # ── 6 + 7. ego_planner + traj_server + PositionCommand→set_velocity bridge ─
    # Delayed so /odometry and depth are live before the grid map starts.
    ego_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_drone_launch),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )
    ego_planner_bridge = Node(
        package='planner',
        executable='ego_planner_bridge.py',
        name='ego_planner_bridge',
        output='screen',
        parameters=[{
            'pos_cmd_topic':        '/drone_0_planning/pos_cmd',
            'set_velocity_service': LaunchConfiguration('set_velocity_service'),
            'forward_rate_hz':      50.0,
            'command_duration':     0.15,
            'auto_arm':             False,
        }],
    )
    planner_stage = TimerAction(period=5.0, actions=[ego_planner, ego_planner_bridge])

    # The shared RViz preset serves simulations too, so override its fixed frame
    # here instead of changing default.rviz globally from world to map.
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['--display-config', rviz_config, '--fixed-frame', 'map'],
        condition=IfCondition(LaunchConfiguration('with_rviz')),
    )

    return LaunchDescription([
        *args,
        uxrce_agent,
        realsense,
        optitrack_bridge,
        offboard,
        odometry_converter,
        planner_stage,
        rviz,
    ])
