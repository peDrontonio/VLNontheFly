import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for a SINGLE REAL DRONE using ego-planner-swarm.

    input:
        '/move_base_simple/goal':
            formato: PoseStamped

    output:
        /drone_0_planning/pos_cmd:
            formato: quadrotor_msgs/PositionCommand

    The pos_cmd → PX4 velocity bridge is the `ego_planner_bridge` node in the
    `planner` package (see mobile_gazebo ego_planner_flight.launch.py).

    Launch arguments:
        use_sim_time: set true when replaying a bag with `--clock` so the grid
                      map syncs depth and odometry on bag time, not wall time.

    Adjust map parameters to your flight area
    """

    use_sim_time = LaunchConfiguration('use_sim_time')
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

    odom_topic = '/odometry'
    depth_topic = '/camera/camera/depth/image_rect_raw'

    drone_id = '0'

    cx = 320.183
    cy = 236.455
    fx = 382.613
    fy = 382.613

    max_vel = 0.3
    max_acc = 0.5
    max_yaw_rate = 0.4
    planning_horizon = 7.5

    # Map Parameters
    map_size_x = 15.0   
    map_size_y = 15.0
    map_size_z = 3.0

    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='drone_' + drone_id + '_ego_planner_node',
        output='screen',
        remappings=[
            # --- FSM odometry input ---
            ('odom_world', odom_topic),

            # --- Planner outputs ---
            ('planning/bspline', planning_bspline_topic),
            ('planning/data_display', 'drone_' + drone_id + '_planning/data_display'),
            ('planning/goal_status', 'drone_' + drone_id + '_planning/goal_status'),
            ('goal_gate_markers', goal_gate_markers_topic),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),

            # --- Visualization topics ---
            ('goal_point',    'drone_' + drone_id + '_plan_vis/goal_point'),
            ('global_list',   'drone_' + drone_id + '_plan_vis/global_list'),
            ('init_list',     'drone_' + drone_id + '_plan_vis/init_list'),
            ('optimal_list',  'drone_' + drone_id + '_plan_vis/optimal_list'),
            ('a_star_list',   'drone_' + drone_id + '_plan_vis/a_star_list'),

            # --- Grid map sensor inputs ---
            ('grid_map/odom',  odom_topic),
            ('grid_map/depth', depth_topic),
            ('grid_map/camera_info', '/camera/camera/depth/camera_info'),

            # --- Grid map output ---
            ('grid_map/occupancy_inflate',
             'drone_' + drone_id + '_grid/grid_map/occupancy_inflate'),
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            # ---- FSM ----
            {'fsm/flight_type': 1},             # 1=RViz goal, 2=preset waypoints
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': True},  # skips trigger, safe for real
            {'fsm/fail_safe': True},

            # ---- Goal gate (geofence on incoming goals: VLM, RViz, topic pub) ----
            # All values in the planner map frame (= /odometry origin; with
            # OptiTrack feeding the EKF this is the fixed OptiTrack origin).
            # Goals outside the box or inside a keep-out cylinder are rejected
            # and 'rejected:zone'/'rejected:keepout' is published on
            # drone_0_planning/goal_status. Measure the room before enabling.
            # Room surveyed 2026-06-12 on /odometry, drone placed at all four
            # corners (LB -2.38,-0.75 / LF -2.47,+2.77 / RF +2.28,+2.92 /
            # RB +2.42,-0.59). Origin = OptiTrack calibration square (takeoff
            # spot), +y = front, +x = right, floor z = 0. Each wall = innermost
            # corner reading pulled in 0.5 m for tracking overshoot + drone
            # radius.
            {'fsm/goal_gate_enable': True},
            {'fsm/goal_gate_frame_id': goal_gate_frame_id},
            {'fsm/goal_gate_x_min': -5.85},
            {'fsm/goal_gate_x_max': 5.75},
            {'fsm/goal_gate_y_min': -5.05},
            {'fsm/goal_gate_y_max': 5.25},
            # Keep the geofence slightly below the nominal z=0 floor so goals
            # at the grounded vehicle's current altitude survive mocap/EKF noise.
            {'fsm/goal_gate_z_min': ParameterValue(goal_gate_z_min, value_type=float)},
            {'fsm/goal_gate_z_max': ParameterValue(goal_gate_z_max, value_type=float)},
            # Keep-out cylinders (full height), e.g. OptiTrack tripods.
            # Parallel arrays; radius should cover tripod legs + planner
            # inflation margin. Placeholder entry is inert (radius 0).
            {'fsm/keepout_x': [0.0]},
            {'fsm/keepout_y': [0.0]},
            {'fsm/keepout_radius': [0.0]},

            # Waypoints (only used when flight_type=2)
            {'fsm/waypoint_num': 1},
            {'fsm/waypoint0_x': 0.0},
            {'fsm/waypoint0_y': 0.0},
            {'fsm/waypoint0_z': 1.0},

            # ---- Grid Map ----
            {'grid_map/resolution': 0.1},
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': 5.5},
            {'grid_map/local_update_range_y': 5.5},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': ParameterValue(
                obstacles_inflation, value_type=float)},
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': -0.01},

            # Camera intrinsics
            {'grid_map/cx': cx},
            {'grid_map/cy': cy},
            {'grid_map/fx': fx},
            {'grid_map/fy': fy},

            # Depth filter
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 4.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            # The fixed mount sees a persistent self-return in RealSense rows
            # 24..30. Crop only the top edge; retain the side/bottom FOV.
            {'grid_map/depth_filter_top_margin': ParameterValue(
                depth_filter_top_margin, value_type=int)},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},

            # Occupancy probabilities
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': 0.1},
            {'grid_map/max_ray_length': 4.0},   # D435i effective range

            # Map display & limits
            {'grid_map/virtual_ceil_height': 2.5},
            {'grid_map/visualization_truncate_height': 2.2},
            {'grid_map/show_occ_time': False},

            # Retained only as the fallback when TF camera projection is disabled.
            {'grid_map/pose_type': 2},

            # Production camera pose: resolve map <- depth optical directly at
            # the depth measurement stamp. Never compose with latest odometry.
            {'grid_map/use_tf_camera_pose': ParameterValue(
                use_tf_camera_pose, value_type=bool)},
            {'grid_map/tf_lookup_timeout': ParameterValue(
                tf_lookup_timeout, value_type=float)},

            {'grid_map/frame_id': 'map'},
            {'grid_map/odom_depth_timeout': 3.0},  # tolerant for real HW

            # ---- Planner Manager ----
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': True},
            {'manager/drone_id': int(drone_id)},

            # ---- Trajectory Optimization ----
            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': ParameterValue(
                optimization_lambda_collision, value_type=float)},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': ParameterValue(optimization_dist0, value_type=float)},
            {'optimization/swarm_clearance': 0.5},
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},

            # ---- B-Spline limits ----
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},

            # ---- Object prediction (not used, but params required) ----
            {'prediction/obj_num': 0},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0},
        ]
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name='drone_' + drone_id + '_traj_server',
        output='screen',
        remappings=[
            ('position_cmd', 'drone_' + drone_id + '_planning/pos_cmd'),
            ('planning/bspline', planning_bspline_topic),
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'traj_server/time_forward': 1.0},
            {'traj_server/max_yaw_rate': max_yaw_rate}
        ],
        condition=IfCondition(start_traj_server),
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock (bag replay) instead of wall time.'))
    ld.add_action(DeclareLaunchArgument(
        'goal_gate_markers_topic', default_value='/drone_0_plan_vis/goal_gate',
        description='MarkerArray topic for the allowed goal volume and keep-out cylinders.'))
    ld.add_action(DeclareLaunchArgument(
        'goal_gate_frame_id', default_value='map',
        description='Fixed planner-world frame used by both the goal gate and its markers.'))
    ld.add_action(DeclareLaunchArgument(
        'goal_gate_z_min', default_value='-0.2',
        description='Lowest accepted goal altitude in map coordinates [m]. The default '
                    'includes the nominal z=0 floor plus localization noise.'))
    ld.add_action(DeclareLaunchArgument(
        'goal_gate_z_max', default_value='4.0',
        description='Highest accepted goal altitude in map coordinates [m].'))
    ld.add_action(DeclareLaunchArgument(
        'use_tf_camera_pose', default_value='true',
        description='Use synchronized CameraInfo and timestamped TF for depth projection.'))
    ld.add_action(DeclareLaunchArgument(
        'start_traj_server', default_value='true',
        description='Start the B-spline to position-command server. Disable for isolated sweeps.'))
    ld.add_action(DeclareLaunchArgument(
        'planning_bspline_topic', default_value='/drone_0_planning/bspline',
        description='Planner B-spline output; use an isolated topic for non-actuating sweeps.'))
    ld.add_action(DeclareLaunchArgument(
        'obstacles_inflation', default_value='0.20',
        description='Occupied-voxel inflation radius [m]. At 0.1 m resolution this is 2 voxels.'))
    ld.add_action(DeclareLaunchArgument(
        'depth_filter_top_margin', default_value='32',
        description='Depth rows cropped only from the top to mask the fixed airframe return.'))
    ld.add_action(DeclareLaunchArgument(
        'tf_lookup_timeout', default_value='0.15',
        description='Maximum wait for timestamped map-to-camera TF [s].'))
    ld.add_action(DeclareLaunchArgument(
        'optimization_dist0', default_value='0.40',
        description='Optimizer clearance distance from inflated occupancy [m].'))
    ld.add_action(DeclareLaunchArgument(
        'optimization_lambda_collision', default_value='0.5',
        description='Collision cost weight for trajectory optimization.'))
    ld.add_action(ego_planner_node)
    ld.add_action(traj_server_node)

    return ld
