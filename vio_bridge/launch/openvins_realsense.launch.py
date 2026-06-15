"""Launch OpenVINS (stereo IR MSCKF) + the OpenVINS->PX4 bridge for the D435i bag.

  ros2 launch vio_bridge openvins_realsense.launch.py
  ros2 launch vio_bridge openvins_realsense.launch.py rviz_enable:=true
  ros2 launch vio_bridge openvins_realsense.launch.py config_path:=/abs/estimator_config.yaml

Then play the bag with --clock:
  ros2 bag play rosbag2_2026_05_26-17_57_21 --clock
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_config_path():
    # With `colcon build --symlink-install`, realpath points back into the source
    # tree: src/vio_bridge/launch/ -> ../../open_vins/config/realsense_d435i/.
    here = os.path.dirname(os.path.realpath(__file__))
    cand = os.path.normpath(os.path.join(
        here, "..", "..", "open_vins", "config", "realsense_d435i", "estimator_config.yaml"))
    if os.path.isfile(cand):
        return cand
    # Fallback to the known workspace location on this machine.
    fallback = "/home/orin/ros2_ws/src/open_vins/config/realsense_d435i/estimator_config.yaml"
    return fallback


def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not os.path.isfile(config_path):
        from launch.actions import LogInfo
        return [LogInfo(msg=f"ERROR: estimator config not found: {config_path} "
                            f"(override with config_path:=...)")]

    ov_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace="ov_msckf",
        output="screen",
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": True},
            {"max_cameras": 2},
            {"config_path": config_path},
        ],
    )

    bridge_node = Node(
        package="vio_bridge",
        executable="openvins_bridge_node",
        output="screen",
        parameters=[{
            "odom_topic": LaunchConfiguration("odom_topic"),
            "body_offset_x": LaunchConfiguration("body_offset_x"),
            "body_offset_y": 0.0,
            "body_offset_z": 0.0,
            "publish_velocity": True,
            "publish_tf": True,
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=["--ros-args", "--log-level", "warn"],
    )

    return [ov_node, bridge_node, rviz_node]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("config_path", default_value=_default_config_path(),
                              description="absolute path to estimator_config.yaml"),
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument("odom_topic", default_value="/ov_msckf/odomimu"),
        DeclareLaunchArgument("body_offset_x", default_value="0.11",
                              description="IMU/camera forward offset from FC centre (m, FRD)"),
        DeclareLaunchArgument("rviz_enable", default_value="false"),
    ]
    ld = LaunchDescription(args)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
