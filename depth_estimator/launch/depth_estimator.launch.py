"""
depth_estimator.launch.py
Lança o nó de estimativa de profundidade com parâmetros configuráveis.

Uso:
  ros2 launch depth_estimator depth_estimator.launch.py
  ros2 launch depth_estimator depth_estimator.launch.py device:=cpu input_topic:=/my_cam/image_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------ args
    args = [
        DeclareLaunchArgument("model_name",    default_value="depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf",
                              description="HuggingFace model ID ou caminho local"),
        DeclareLaunchArgument("device",        default_value="cuda",
                              description="'cuda' ou 'cpu'"),
        DeclareLaunchArgument("input_topic",   default_value="/camera/camera/color/image_raw",
                              description="Tópico de entrada de imagem"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info",
                              description="Tópico de CameraInfo (opcional)"),
        DeclareLaunchArgument("output_topic",  default_value="/depth/image_raw",
                              description="Tópico de saída de depth (32FC1)"),
        DeclareLaunchArgument("visual_topic",  default_value="/depth/image_visual",
                              description="Tópico de saída colorida (rgb8)"),
        DeclareLaunchArgument("publish_visual",default_value="True",
                              description="Publica visualização colorida?"),
        DeclareLaunchArgument("min_depth",     default_value="100.0",
                              description="Profundidade mínima para normalização visual (m)"),
        DeclareLaunchArgument("max_depth",     default_value="10000.0",
                              description="Profundidade máxima para normalização visual (m)"),
    ]

    # ------------------------------------------------------------------ node
    node = Node(
        package="depth_estimator",
        executable="depth_estimator_node",
        name="depth_estimator",
        output="screen",
        parameters=[{
            "model_name":          LaunchConfiguration("model_name"),
            "device":              LaunchConfiguration("device"),
            "input_topic":         LaunchConfiguration("input_topic"),
            "camera_info_topic":   LaunchConfiguration("camera_info_topic"),
            "output_topic":        LaunchConfiguration("output_topic"),
            "visual_topic":        LaunchConfiguration("visual_topic"),
            "publish_visual":      LaunchConfiguration("publish_visual"),
            "min_depth":           LaunchConfiguration("min_depth"),
            "max_depth":           LaunchConfiguration("max_depth"),
        }],
    )

    return LaunchDescription(args + [node])
