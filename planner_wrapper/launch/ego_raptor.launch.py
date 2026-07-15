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
    goal_gate_markers_topic = LaunchConfiguration('goal_gate_markers_topic')
    goal_gate_frame_id = LaunchConfiguration('goal_gate_frame_id')
    goal_gate_z_min = LaunchConfiguration('goal_gate_z_min')
    goal_gate_z_max = LaunchConfiguration('goal_gate_z_max')
    use_tf_camera_pose = LaunchConfiguration('use_tf_camera_pose')
    start_traj_server = LaunchConfiguration('start_traj_server')
    planning_bspline_topic = LaunchConfiguration('planning_bspline_topic')
    obstacles_inflation = LaunchConfiguration('obstacles_inflation')
    depth_filter_top_margin = LaunchConfiguration('depth_filter_top_margin')
    tf_lookup_timeout = LaunchConfiguration('tf_lookup_timeout')
    optimization_dist0 = LaunchConfiguration('optimization_dist0')
    optimization_lambda_collision = LaunchConfiguration('optimization_lambda_collision')
    device = LaunchConfiguration('device')
    baudrate = LaunchConfiguration('baudrate')
    mocap_offset_x = LaunchConfiguration('mocap_offset_x')
    mocap_offset_y = LaunchConfiguration('mocap_offset_y')
    mocap_offset_z = LaunchConfiguration('mocap_offset_z')
    camera_mount_x = LaunchConfiguration('camera_mount_x')
    camera_mount_y = LaunchConfiguration('camera_mount_y')
    camera_mount_z = LaunchConfiguration('camera_mount_z')
    camera_mount_roll_deg = LaunchConfiguration('camera_mount_roll_deg')
    camera_mount_pitch_deg = LaunchConfiguration('camera_mount_pitch_deg')
    camera_mount_yaw_deg = LaunchConfiguration('camera_mount_yaw_deg')
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
            # Production contract: the Motive rigid body is named "drone" (NOT
            # base_link, so the natnet TF map->drone cannot collide with the
            # odometry converter's map->base_link).
            'pose_topic': '/drone/pose',
            'body_offset_x': ParameterValue(mocap_offset_x, value_type=float),
            'body_offset_y': ParameterValue(mocap_offset_y, value_type=float),
            'body_offset_z': ParameterValue(mocap_offset_z, value_type=float),
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
            # Camera mount is independent of the OptiTrack lever arm. Translation
            # is base_link -> camera_link in ROS FLU; rotation inputs are degrees.
            'publish_mount_tf': True,
            'mount_parent_frame_id': 'base_link',
            'mount_x': ParameterValue(camera_mount_x, value_type=float),
            'mount_y': ParameterValue(camera_mount_y, value_type=float),
            'mount_z': ParameterValue(camera_mount_z, value_type=float),
            'mount_roll': PythonExpression([
                'str(float("', camera_mount_roll_deg,
                '") * 3.141592653589793 / 180.0)'
            ]),
            'mount_pitch': PythonExpression([
                'str(float("', camera_mount_pitch_deg,
                '") * 3.141592653589793 / 180.0)'
            ]),
            'mount_yaw': PythonExpression([
                'str(float("', camera_mount_yaw_deg,
                '") * 3.141592653589793 / 180.0)'
            ]),
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
        launch_arguments={
            'goal_gate_markers_topic': goal_gate_markers_topic,
            'goal_gate_frame_id': goal_gate_frame_id,
            'goal_gate_z_min': goal_gate_z_min,
            'goal_gate_z_max': goal_gate_z_max,
            'use_tf_camera_pose': use_tf_camera_pose,
            'start_traj_server': start_traj_server,
            'planning_bspline_topic': planning_bspline_topic,
            'obstacles_inflation': obstacles_inflation,
            'depth_filter_top_margin': depth_filter_top_margin,
            'tf_lookup_timeout': tf_lookup_timeout,
            'optimization_dist0': optimization_dist0,
            'optimization_lambda_collision': optimization_lambda_collision,
        }.items(),
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
            'goal_gate_markers_topic', default_value='/drone_0_plan_vis/goal_gate',
            description='MarkerArray topic for the allowed goal volume and keep-out cylinders. '
                        'Published by ego-planner when start_planner:=true.'),
        DeclareLaunchArgument(
            'goal_gate_frame_id', default_value='map',
            description='Fixed planner-world frame used by both the goal gate and its markers.'),
        DeclareLaunchArgument(
            'goal_gate_z_min', default_value='-0.2',
            description='Lowest accepted goal altitude in map coordinates [m]. The default '
                        'allows ground-level planner tests and small localization Z drift.'),
        DeclareLaunchArgument(
            'goal_gate_z_max', default_value='4.0',
            description='Highest accepted goal altitude in map coordinates [m].'),
        DeclareLaunchArgument(
            'use_tf_camera_pose', default_value='true',
            description='Project RealSense depth using synchronized CameraInfo and the '
                        'timestamped map-to-depth-optical TF chain.'),
        DeclareLaunchArgument(
            'start_traj_server', default_value='true',
            description='Start trajectory execution. Set false for a non-actuating planner sweep.'),
        DeclareLaunchArgument(
            'planning_bspline_topic', default_value='/drone_0_planning/bspline',
            description='Planner B-spline output topic; isolate this during non-actuating tests.'),
        DeclareLaunchArgument(
            'obstacles_inflation', default_value='0.20',
            description='Occupied-voxel inflation radius [m].'),
        DeclareLaunchArgument(
            'depth_filter_top_margin', default_value='32',
            description='Top depth-image rows cropped to remove the fixed airframe return.'),
        DeclareLaunchArgument(
            'tf_lookup_timeout', default_value='0.15',
            description='Maximum timestamped camera-TF lookup wait [s].'),
        DeclareLaunchArgument(
            'optimization_dist0', default_value='0.40',
            description='Optimizer clearance from inflated occupancy [m].'),
        DeclareLaunchArgument(
            'optimization_lambda_collision', default_value='0.5',
            description='Trajectory collision-cost weight.'),
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
        # OptiTrack lever arm. The bridge computes p_fc = p_rigid - R * offset,
        # so these values are FC center -> rigid-body origin in body FRD.
        DeclareLaunchArgument('mocap_offset_x', default_value='0.0',
                              description='FC center -> OptiTrack rigid-body origin, FRD x [m].'),
        DeclareLaunchArgument('mocap_offset_y', default_value='0.0',
                              description='FC center -> OptiTrack rigid-body origin, FRD y [m].'),
        DeclareLaunchArgument('mocap_offset_z', default_value='0.0',
                              description='FC center -> OptiTrack rigid-body origin, FRD z [m].'),
        DeclareLaunchArgument('publish_velocity', default_value='true',
                              description='Have optitrack_bridge_node finite-difference velocity.'),
        DeclareLaunchArgument('publish_tf', default_value='false',
                              description='Publish optitrack NED TF. Keep false for ego-planner.'),
        DeclareLaunchArgument(
            'with_realsense', default_value='false',
            description=(
                'Start the RealSense camera, including RGB/depth streams, CameraInfo, '
                'internal camera TFs, and the static base_link -> camera_link mount TF. '
                'Required for full planner and image-projection modes.'
            ),
        ),
        # RealSense mounting extrinsic, independent of the mocap lever arm.
        DeclareLaunchArgument('camera_mount_x', default_value='0.155',
                              description='base_link -> camera_link translation, FLU x [m].'),
        DeclareLaunchArgument('camera_mount_y', default_value='0.0',
                              description='base_link -> camera_link translation, FLU y [m].'),
        DeclareLaunchArgument('camera_mount_z', default_value='0.0',
                              description='base_link -> camera_link translation, FLU z [m].'),
        DeclareLaunchArgument('camera_mount_roll_deg', default_value='0.0',
                              description='camera_link roll relative to base_link [deg].'),
        DeclareLaunchArgument('camera_mount_pitch_deg', default_value='7.0',
                              description='camera_link pitch relative to base_link [deg].'),
        DeclareLaunchArgument('camera_mount_yaw_deg', default_value='0.0',
                              description='camera_link yaw relative to base_link [deg].'),
        DeclareLaunchArgument(
            'with_relative_goal', default_value='true',
            description='Start /relative_goal -> /move_base_simple/goal adapter. '
                        '/relative_goal uses body-relative FLU offsets: +x forward, +y left, +z up.'),
        uxrce_agent,
        optitrack_bridge,
        realsense,
        odometry_converter,
        ego_planner,
        pos_cmd_bridge,
        relative_goal_adapter,
        set_external_mode,
    ])
