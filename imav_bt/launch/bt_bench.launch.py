# bt_bench.launch.py — run the whole tree on a desk, with no aircraft.
#
#   ros2 launch imav_bt bt_bench.launch.py
#
# Starts three nodes and nothing else:
#
#   mock_arena    a fake drone and a fake ego-planner (scripts/mock_arena.py)
#   payload_node  the payload services, mock backend
#   bt_node       the tree itself, armed automatically
#
# Watch it with:
#
#   ros2 topic echo /imav_bt/snapshot --field data
#   ros2 topic echo /imav_bt/status --field data
#
# Fault injection -- each of these reproduces a real failure mode:
#
#   planner_mode:=silent            ego-planner accepts a goal then never
#                                   reports anything again. Confirms the
#                                   Deadline decorators do their job.
#   planner_mode:=reject            every goal refused
#   battery_drain_per_min:=20.0     safety branch preempts mid-mission
#   odom_stops_after_s:=10.0        pose lost mid-flight
#   flight_plan:=flight_b           run mission 4 instead of the chain
#   payload_fail:=[grab_ring]       a jammed grabber
#
# See docs/TESTING.md for the full matrix and what each one should prove.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    planner_mode = LaunchConfiguration("planner_mode")
    flight_plan = LaunchConfiguration("flight_plan")
    autostart = LaunchConfiguration("autostart")
    battery_drain = LaunchConfiguration("battery_drain_per_min")
    odom_stops_after = LaunchConfiguration("odom_stops_after_s")
    speed = LaunchConfiguration("speed_mps")
    payload_fail = LaunchConfiguration("payload_fail")

    return LaunchDescription([
        DeclareLaunchArgument(
            "planner_mode", default_value="normal",
            description="Fake planner behaviour: normal | silent | reject. "
                        "'silent' reproduces ego-planner's unreportable stall."),
        DeclareLaunchArgument(
            "flight_plan", default_value="flight_a",
            description="flight_a (missions 1-3 chained + landing) or flight_b "
                        "(mission 4 alone)."),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Arm the tree without an operator service call. True is "
                        "fine on the bench; NEVER use it on the aircraft."),
        DeclareLaunchArgument(
            "battery_drain_per_min", default_value="0.0",
            description="Fractional battery drain per minute. 20.0 drains a full "
                        "pack in five minutes, so the safety branch trips."),
        DeclareLaunchArgument(
            "odom_stops_after_s", default_value="0.0",
            description="Stop publishing odometry after N seconds (0 disables), "
                        "to test the localization guard."),
        DeclareLaunchArgument(
            "speed_mps", default_value="1.5",
            description="How fast the fake drone flies toward a goal."),
        DeclareLaunchArgument(
            "payload_fail", default_value="",
            description="Comma-separated payloads to force-fail, "
                        "e.g. payload_fail:=grab_ring,drop_cone."),

        Node(
            package="imav_bt",
            executable="mock_arena.py",
            name="mock_arena",
            output="screen",
            parameters=[{
                "planner_mode": planner_mode,
                "speed_mps": ParameterValue(speed, value_type=float),
                "battery_drain_per_min": ParameterValue(
                    battery_drain, value_type=float),
                "odom_stops_after_s": ParameterValue(
                    odom_stops_after, value_type=float),
            }],
        ),
        Node(
            package="imav_bt",
            executable="payload_node.py",
            name="payload_node",
            output="screen",
            parameters=[{
                "backend": "mock",
                "fail": ParameterValue(payload_fail, value_type=str),
            }],
        ),
        Node(
            package="imav_bt",
            executable="bt_node.py",
            name="imav_bt",
            output="screen",
            parameters=[{
                "flight_plan": flight_plan,
                "autostart": ParameterValue(autostart, value_type=bool),
            }],
        ),
    ])
