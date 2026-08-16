# imav_indoor.launch.py — the tree against the real flight stack.
#
#   ros2 launch imav_bt imav_indoor.launch.py
#
# Starts the tree and the payload services, and OPTIONALLY brings up the
# existing ego-planner stack via planner/launch/ego_raptor.launch.py.
#
# ---------------------------------------------------------------------------
# The flight protocol has not changed
# ---------------------------------------------------------------------------
# This package does not automate arming or takeoff (see behaviors/flight.py).
# The documented sequence still applies:
#
#   1. arm
#   2. take off by hand
#   3. enable Raptor with the RC switch
#   4. start this launch file
#   5. `ros2 service call /imav_bt/set_running std_srvs/srv/SetBool "{data: true}"`
#
# `autostart` defaults to FALSE here, unlike bt_bench.launch.py. The tree ticks
# from the moment it starts but sits in its Idle branch commanding nothing until
# step 5. The RC kill switch remains the real override throughout.
#
# ---------------------------------------------------------------------------
# The VLM stack is deliberately NOT launched
# ---------------------------------------------------------------------------
# Three things in this workspace can publish to /move_base_simple/goal, and they
# will fight over the drone if more than one runs:
#
#   * this tree
#   * vlm_region_gate with auto_execute: true (both shipped configs point its
#     goal_topic straight at /move_base_simple/goal)
#   * vlm_nav_supervisor, which drives the gates' execute services
#
# The tree is the sole goal publisher in this phase. If you re-introduce the VLM
# later, run the gates with auto_execute: false and have a behaviour call their
# ~/execute_next Trigger services, so arbitration stays in one place.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    flight_plan = LaunchConfiguration("flight_plan")
    autostart = LaunchConfiguration("autostart")
    config_dir = LaunchConfiguration("config_dir")
    with_stack = LaunchConfiguration("with_stack")
    payload_backend = LaunchConfiguration("payload_backend")

    default_config = os.path.join(
        get_package_share_directory("imav_bt"), "config"
    )

    ego_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("planner"), "launch", "ego_raptor.launch.py")),
        condition=IfCondition(with_stack),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "flight_plan", default_value="flight_a",
            description="flight_a (missions 1-3 chained + precision landing) or "
                        "flight_b (mission 4, which the rules forbid combining)."),
        DeclareLaunchArgument(
            "autostart", default_value="false",
            description="Arm the tree at startup. Keep FALSE on the aircraft — "
                        "arm deliberately with ~/set_running once airborne."),
        DeclareLaunchArgument(
            "config_dir", default_value=default_config,
            description="Directory holding bt.yaml, arena.yaml and missions.yaml."),
        DeclareLaunchArgument(
            "with_stack", default_value="false",
            description="Also launch planner/ego_raptor.launch.py. Keep false if "
                        "you already have the stack running in another terminal."),
        DeclareLaunchArgument(
            "payload_backend", default_value="mock",
            description="Payload driver backend. Only 'mock' exists today."),

        ego_stack,
        Node(
            package="imav_bt",
            executable="payload_node.py",
            name="payload_node",
            output="screen",
            parameters=[{"backend": payload_backend}],
        ),
        Node(
            package="imav_bt",
            executable="bt_node.py",
            name="imav_bt",
            output="screen",
            parameters=[{
                "config_dir": config_dir,
                "flight_plan": flight_plan,
                "autostart": ParameterValue(autostart, value_type=bool),
            }],
        ),
    ])
