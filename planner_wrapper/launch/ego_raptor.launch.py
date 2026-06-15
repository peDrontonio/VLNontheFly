# ego_raptor.launch.py — ego-planner -> Raptor EXTERNAL flight stack.
#
# The "(then ...)" step of the protocol: arm -> take off by hand -> Raptor on
# via RC switch -> `ros2 launch planner ego_raptor.launch.py` -> goal in RViz.
#
# Data flow:
#   /fmu/out/vehicle_odometry --odometry_converter--> /odometry (ENU)
#     -> ego_planner + grid_map (real_single_drone.launch.py)
#     -> traj_server -> /drone_0_planning/pos_cmd (ENU, 100 Hz)
#     -> pos_cmd_to_raptor (ENU->NED, 1:1)
#     -> /fmu/in/trajectory_setpoint_raptor -> rl_tools_commander (EXTERNAL)
#
# If set_external:=true, also runs set_external_mode.py once (MAVLink shell
# over the FC USB port) to put rl_tools_commander into EXTERNAL automatically.
# Safe before setpoints arrive: the commander freezes at the Raptor activation
# target until the stream starts. RC switch stays the master override.
#
# Optional hardware bring-up:
#   - with_xrce:=true starts MicroXRCEAgent serial --dev <device> -b <baudrate>
#   - with_optitrack:=true starts optitrack_bridge_node:
#       PoseStamped on <pose_topic> -> /fmu/in/vehicle_visual_odometry
#   - with_realsense:=true starts RealSense depth under /camera/camera/...
#
# Defaults stay conservative. In the usual QGC/MAVProxy setup, MAVProxy owns
# /dev/ttyACM0, so set_external defaults false and EXTERNAL should be set from
# the FC shell/QGC console or by running set_external_mode.py against a free
# MAVLink endpoint.
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mavlink_url = LaunchConfiguration('mavlink_url')
    set_external = LaunchConfiguration('set_external')
    start_planner = LaunchConfiguration('start_planner')
    with_xrce = LaunchConfiguration('with_xrce')
    with_optitrack = LaunchConfiguration('with_optitrack')
    with_realsense = LaunchConfiguration('with_realsense')
    with_relative_goal = LaunchConfiguration('with_relative_goal')
    camera_pitch_deg = LaunchConfiguration('camera_pitch_deg')

    device = LaunchConfiguration('device')
    baudrate = LaunchConfiguration('baudrate')
    pose_topic = LaunchConfiguration('pose_topic')
    body_offset_x = LaunchConfiguration('body_offset_x')
    body_offset_y = LaunchConfiguration('body_offset_y')
    body_offset_z = LaunchConfiguration('body_offset_z')
    publish_velocity = LaunchConfiguration('publish_velocity')
    publish_tf = LaunchConfiguration('publish_tf')

    uxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', device, '-b', baudrate],
        output='screen',
        condition=IfCondition(with_xrce),
    )

    optitrack_bridge = Node(
        package='vio_bridge',
        executable='optitrack_bridge_node',
        name='optitrack_bridge_node',
        output='screen',
        parameters=[{
            'pose_topic': pose_topic,
            'body_offset_x': ParameterValue(body_offset_x, value_type=float),
            'body_offset_y': ParameterValue(body_offset_y, value_type=float),
            'body_offset_z': ParameterValue(body_offset_z, value_type=float),
            'publish_velocity': ParameterValue(publish_velocity, value_type=bool),
            'publish_tf': ParameterValue(publish_tf, value_type=bool),
        }],
        condition=IfCondition(with_optitrack),
    )

    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'align_depth.enable': True,
            'enable_gyro': True,
            'enable_accel': True,
            # Hard-coded camera mount TF (base_link -> camera_link), matching
            # mobile_flight/flight_stack.launch.py. VIO bridge offsets are FRD,
            # ROS base_link is FLU, so y/z are negated; pitch is -camera_pitch_deg.
            'publish_mount_tf': True,
            'mount_parent_frame_id': 'base_link',
            'mount_x': body_offset_x,
            'mount_y': PythonExpression([
                'str(-float("', body_offset_y, '"))'
            ]),
            'mount_z': PythonExpression([
                'str(-float("', body_offset_z, '"))'
            ]),
            'mount_roll': 0.0,
            'mount_pitch': PythonExpression([
                'str(-float("', camera_pitch_deg, '") * 3.141592653589793 / 180.0)'
            ]),
            'mount_yaw': 0.0,
        }],
        condition=IfCondition(with_realsense),
    )

    odometry_converter = Node(
        package='planner',
        executable='odometry_converter.py',
        name='vehicle_odom_converter',
        output='screen',
        remappings=[
            # EKF output (NOT /fmu/in/vehicle_visual_odometry) so the planner
            # shares the PX4 local origin by construction.
            ('/vehicle_odometry', '/fmu/out/vehicle_odometry'),
        ],
    )

    ego_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'),
            'launch', 'real_single_drone.launch.py')),
        condition=IfCondition(start_planner),
    )

    pos_cmd_bridge = Node(
        package='planner',
        executable='pos_cmd_to_raptor.py',
        name='pos_cmd_to_raptor',
        output='screen',
    )

    relative_goal_adapter = Node(
        package='planner',
        executable='relative_goal_to_map.py',
        name='relative_goal_to_map',
        output='screen',
        condition=IfCondition(with_relative_goal),
    )

    set_external_mode = TimerAction(
        period=3.0,  # let the MAVLink USB port settle before the one-shot
        actions=[ExecuteProcess(
            cmd=[os.path.join(get_package_prefix('planner'),
                              'lib', 'planner', 'set_external_mode.py'),
                 mavlink_url],
            output='screen',
            condition=IfCondition(set_external),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mavlink_url', default_value='/dev/ttyACM0',
            description='MAVLink port to the FC for the set_mode EXTERNAL one-shot.'),
        DeclareLaunchArgument(
            'set_external', default_value='false',
            description='Run set_external_mode.py at startup. Keep false when MAVProxy owns '
                        '/dev/ttyACM0; set EXTERNAL manually from the FC shell/QGC console.'),
        DeclareLaunchArgument(
            'start_planner', default_value='true',
            description='Start ego-planner. false = pipeline-isolation tests: only odometry '
                        'converter + pos_cmd bridge run, feed pos_cmd from fig8_pos_cmd.py.'),
        DeclareLaunchArgument(
            'with_xrce', default_value='false',
            description='Start MicroXRCEAgent for the PX4 DDS serial link.'),
        DeclareLaunchArgument('device', default_value='/dev/ttyTHS1',
                              description='PX4 DDS serial device used when with_xrce:=true.'),
        DeclareLaunchArgument('baudrate', default_value='921600',
                              description='PX4 DDS serial baudrate used when with_xrce:=true.'),
        DeclareLaunchArgument(
            'with_optitrack', default_value='false',
            description='Start optitrack_bridge_node to feed PX4 visual odometry.'),
        DeclareLaunchArgument(
            'pose_topic', default_value='/drone/pose',
            description='OptiTrack PoseStamped topic consumed when with_optitrack:=true.'),
        DeclareLaunchArgument('body_offset_x', default_value='0.0',
                              description='Rigid-body origin -> FC center offset, FRD x [m].'),
        DeclareLaunchArgument('body_offset_y', default_value='0.0',
                              description='Rigid-body origin -> FC center offset, FRD y [m].'),
        DeclareLaunchArgument('body_offset_z', default_value='0.0',
                              description='Rigid-body origin -> FC center offset, FRD z [m].'),
        DeclareLaunchArgument('publish_velocity', default_value='true',
                              description='Have optitrack_bridge_node finite-difference velocity.'),
        DeclareLaunchArgument('publish_tf', default_value='false',
                              description='Publish optitrack NED TF. Keep false for ego-planner.'),
        DeclareLaunchArgument(
            'with_realsense', default_value='false',
            description='Start RealSense depth on /camera/camera/depth/image_rect_raw. '
                        'Needed for full planner mode, not for start_planner:=false.'),
        DeclareLaunchArgument(
            'with_relative_goal', default_value='true',
            description='Start /relative_goal -> /move_base_simple/goal adapter. '
                        '/relative_goal uses body-relative FLU offsets: +x forward, +y left, +z up.'),
        DeclareLaunchArgument(
            'camera_pitch_deg', default_value='-7.0',
            description='Camera mount pitch [deg] for the hard-coded base_link -> camera_link '
                        'mount TF (published by the RealSense node when with_realsense:=true). '
                        'mount_x/y/z come from body_offset_x/y/z (FRD -> FLU).'),
        uxrce_agent,
        optitrack_bridge,
        realsense,
        odometry_converter,
        ego_planner,
        pos_cmd_bridge,
        relative_goal_adapter,
        set_external_mode,
    ])
