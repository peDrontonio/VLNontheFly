from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vlm_params_file = LaunchConfiguration("vlm_params_file")
    primitive_gate_params_file = LaunchConfiguration("primitive_gate_params_file")
    point_gate_params_file = LaunchConfiguration("point_gate_params_file")
    region_gate_params_file = LaunchConfiguration("region_gate_params_file")
    nav_supervisor_params_file = LaunchConfiguration("nav_supervisor_params_file")
    prompt_mode = LaunchConfiguration("prompt_mode")
    target_object = LaunchConfiguration("target_object")
    image_topic = LaunchConfiguration("image_topic")
    fixed_rate_hz = LaunchConfiguration("fixed_rate_hz")
    backend_type = LaunchConfiguration("backend_type")
    engine_dir = LaunchConfiguration("engine_dir")
    multimodal_engine_dir = LaunchConfiguration("multimodal_engine_dir")
    plugin_path = LaunchConfiguration("plugin_path")
    enable_primitive_gate = LaunchConfiguration("enable_primitive_gate")
    enable_point_gate = LaunchConfiguration("enable_point_gate")
    enable_region_gate = LaunchConfiguration("enable_region_gate")
    enable_nav_supervisor = LaunchConfiguration("enable_nav_supervisor")

    default_vlm_params = PathJoinSubstitution(
        [FindPackageShare("edgellm_vlm_ros"), "config", "vlm_node.yaml"]
    )
    default_primitive_gate_params = PathJoinSubstitution(
        [FindPackageShare("edgellm_vlm_ros"), "config", "primitive_gate.yaml"]
    )
    default_point_gate_params = PathJoinSubstitution(
        [FindPackageShare("edgellm_vlm_ros"), "config", "point_gate.yaml"]
    )
    default_region_gate_params = PathJoinSubstitution(
        [FindPackageShare("edgellm_vlm_ros"), "config", "region_gate_pipeline.yaml"]
    )
    default_nav_supervisor_params = PathJoinSubstitution(
        [FindPackageShare("edgellm_vlm_ros"), "config", "nav_supervisor.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("vlm_params_file", default_value=default_vlm_params),
            DeclareLaunchArgument(
                "primitive_gate_params_file", default_value=default_primitive_gate_params
            ),
            DeclareLaunchArgument("point_gate_params_file", default_value=default_point_gate_params),
            DeclareLaunchArgument("region_gate_params_file", default_value=default_region_gate_params),
            DeclareLaunchArgument(
                "nav_supervisor_params_file", default_value=default_nav_supervisor_params
            ),
            DeclareLaunchArgument(
                "prompt_mode",
                default_value="point",
                description="VLM prompt profile: point, primitive, or region. "
                "Set this to match the gate you enable.",
            ),
            DeclareLaunchArgument(
                "target_object",
                default_value="",
                description="In region mode, the object to navigate toward (e.g. 'tripod'). "
                "Empty means open-space region selection.",
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument("fixed_rate_hz", default_value="0.5"),
            DeclareLaunchArgument("backend_type", default_value="edgellm"),
            DeclareLaunchArgument(
                "engine_dir",
                default_value="/home/orin/imav/tensorrt-edgellm-workspace/Qwen3.5-2B/int4_awq/engines/llm",
            ),
            DeclareLaunchArgument(
                "multimodal_engine_dir",
                default_value="/home/orin/imav/tensorrt-edgellm-workspace/Qwen3.5-2B/int4_awq/engines/visual",
            ),
            DeclareLaunchArgument(
                "plugin_path",
                default_value="/home/orin/TensorRT-Edge-LLM/build/libNvInfer_edgellm_plugin.so",
            ),
            DeclareLaunchArgument("enable_primitive_gate", default_value="false"),
            DeclareLaunchArgument("enable_point_gate", default_value="false"),
            DeclareLaunchArgument("enable_region_gate", default_value="false"),
            DeclareLaunchArgument("enable_nav_supervisor", default_value="false"),
            Node(
                package="edgellm_vlm_ros",
                executable="edgellm_vlm_node",
                name="edgellm_vlm_node",
                output="screen",
                parameters=[
                    vlm_params_file,
                    {
                        "image_topic": image_topic,
                        "fixed_rate_hz": ParameterValue(fixed_rate_hz, value_type=float),
                        "backend_type": backend_type,
                        "engine_dir": engine_dir,
                        "multimodal_engine_dir": multimodal_engine_dir,
                        "plugin_path": plugin_path,
                        "prompt_mode": prompt_mode,
                        "target_object": target_object,
                    },
                ],
            ),
            Node(
                package="edgellm_vlm_ros",
                executable="vlm_primitive_gate.py",
                name="vlm_primitive_gate",
                output="screen",
                condition=IfCondition(PythonExpression([
                    "'", enable_primitive_gate, "' == 'true' or '",
                    enable_nav_supervisor, "' == 'true'"
                ])),
                parameters=[primitive_gate_params_file],
            ),
            Node(
                package="edgellm_vlm_ros",
                executable="vlm_point_gate.py",
                name="vlm_point_gate",
                output="screen",
                condition=IfCondition(PythonExpression([
                    "'", enable_point_gate, "' == 'true' or '",
                    enable_nav_supervisor, "' == 'true'"
                ])),
                parameters=[point_gate_params_file],
            ),
            Node(
                package="edgellm_vlm_ros",
                executable="vlm_region_gate.py",
                name="vlm_region_gate",
                output="screen",
                condition=IfCondition(enable_region_gate),
                parameters=[region_gate_params_file],
            ),
            Node(
                package="edgellm_vlm_ros",
                executable="vlm_nav_supervisor.py",
                name="vlm_nav_supervisor",
                output="screen",
                condition=IfCondition(enable_nav_supervisor),
                parameters=[nav_supervisor_params_file],
            ),
        ]
    )
