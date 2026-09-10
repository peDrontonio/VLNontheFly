"""
ego_planner_bag_test.launch.py — planner dry-run against a recorded bag.

Runs the full ego-planner + bridge pipeline with NO flight controller and NO
live camera. Useful to validate planning, the grid map, and the velocity
commands the offboard node *would* send to PX4 (watch /fmu/in/trajectory_setpoint).

Bag topic reconstruction
-------------------------
The recorded bags do NOT contain /odometry. They carry the raw pose source plus
depth, so we rebuild the odometry chain live:

  odom_source:=optitrack  (default):
    /drone/pose → optitrack_bridge_node → /fmu/in/vehicle_visual_odometry
                → odometry_converter → /odometry
  odom_source:=vio:
    /orb_slam3/pose → vio_bridge_node → /fmu/in/vehicle_visual_odometry
                → odometry_converter → /odometry

Both the planner depth input (/camera/camera/depth/image_rect_raw) and the pose
source come straight from the bag.

use_sim_time
------------
The bag is played with --clock and every node runs with use_sim_time:=true, so
the reconstructed /odometry is stamped on bag time and the grid map's depth↔odom
sync lines up. Do not drop --clock.

Usage
-----
  # mocap bag (has /drone/pose)
  ros2 launch mobile_gazebo ego_planner_bag_test.launch.py \
      bag_path:=/home/orin/ros2_ws/rosbag2_1969_12_31-21_32_39

  # VIO bag (has /orb_slam3/pose)
  ros2 launch mobile_gazebo ego_planner_bag_test.launch.py \
      odom_source:=vio bag_path:=/home/orin/ros2_ws/rosbag2_2026_05_27-15_27_19

Then send a goal from RViz ("2D Nav Goal" → /move_base_simple/goal) and watch
/drone_0_planning/pos_cmd and /fmu/in/trajectory_setpoint.

Leave bag_path empty to start only the nodes and play the bag yourself
(remember `--clock`).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ego_launch = os.path.join(
        get_package_share_directory('ego_planner'), 'launch', 'real_single_drone.launch.py')

    sim_time = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument('odom_source', default_value='optitrack',
            description="'optitrack' (bag /drone/pose) or 'vio' (bag /orb_slam3/pose)."),
        DeclareLaunchArgument('bag_path', default_value='',
            description='Path to the rosbag2 directory. Empty = play the bag manually.'),
        DeclareLaunchArgument('rate', default_value='1.0',
            description='Bag playback rate.'),
        DeclareLaunchArgument('pose_topic', default_value='/drone/pose'),
        DeclareLaunchArgument('camera_pitch_deg', default_value='-7.0'),
        DeclareLaunchArgument('body_offset_x', default_value='0.11'),
        DeclareLaunchArgument('body_offset_y', default_value='0.0'),
        DeclareLaunchArgument('body_offset_z', default_value='0.0'),
    ]

    # Bag playback (optional — only if bag_path is non-empty).
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'),
             '--clock', '--rate', LaunchConfiguration('rate')],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('bag_path'), "' != ''"])),
    )

    # OptiTrack bridge: /drone/pose → /fmu/in/vehicle_visual_odometry
    optitrack_bridge = Node(
        package='vio_bridge', executable='optitrack_bridge_node',
        name='optitrack_bridge_node', output='screen',
        parameters=[sim_time, {
            'pose_topic': LaunchConfiguration('pose_topic'),
            'body_offset_x': LaunchConfiguration('body_offset_x'),
            'body_offset_y': LaunchConfiguration('body_offset_y'),
            'body_offset_z': LaunchConfiguration('body_offset_z'),
        }],
        condition=LaunchConfigurationEquals('odom_source', 'optitrack'),
    )

    # VIO bridge: /orb_slam3/pose → /fmu/in/vehicle_visual_odometry
    vio_bridge = Node(
        package='vio_bridge', executable='vio_bridge_node',
        name='vio_bridge_node', output='screen',
        parameters=[sim_time, {
            'camera_pitch_deg': LaunchConfiguration('camera_pitch_deg'),
            'body_offset_x': LaunchConfiguration('body_offset_x'),
            'body_offset_y': LaunchConfiguration('body_offset_y'),
            'body_offset_z': LaunchConfiguration('body_offset_z'),
        }],
        condition=LaunchConfigurationEquals('odom_source', 'vio'),
    )

    # VehicleOdometry → /odometry (nav_msgs/Odometry, ENU)
    odom_converter = Node(
        package='planner', executable='odometry_converter.py',
        name='odometry_converter', output='screen',
        parameters=[sim_time],
        remappings=[('/vehicle_odometry', '/fmu/in/vehicle_visual_odometry')],
    )

    # Offboard control in bench mode (no real arming; observe trajectory_setpoint).
    offboard = Node(
        package='mobile_gazebo', executable='offboard_velocity_control',
        name='offboard_velocity_control', output='screen',
        parameters=[sim_time, {'bench_mode': True}],
    )

    # ego-planner (sim time so grid map syncs on bag time).
    ego_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ego_launch),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # pos_cmd → set_velocity bridge.
    ego_bridge = Node(
        package='planner', executable='ego_planner_bridge.py',
        name='ego_planner_bridge', output='screen',
        parameters=[sim_time],
    )

    # Give the odometry chain a moment before the planner starts consuming it.
    delayed_planner = TimerAction(period=2.0, actions=[ego_planner, ego_bridge])

    return LaunchDescription([
        *args,
        bag_play,
        optitrack_bridge,
        vio_bridge,
        odom_converter,
        offboard,
        delayed_planner,
    ])
