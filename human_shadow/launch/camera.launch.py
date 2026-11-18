"""
Arranca la Orbbec Astra Pro y publica donde esta la camara.

Usa el driver ros2_astra_camera con la profundidad alineada al color.

Uso:
  ros2 launch human_shadow camera.launch.py camera_height:=1.0 camera_pitch_deg:=10
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def camera_tf(context):
    """TF estatico world -> camera_link (altura y pitch de la camara)."""
    h = LaunchConfiguration('camera_height').perform(context)
    pitch = math.radians(float(LaunchConfiguration('camera_pitch_deg').perform(context)))
    return [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='world_to_camera',
        arguments=['--x', '0', '--y', '0', '--z', h,
                   '--roll', '0', '--pitch', str(pitch), '--yaw', '0',
                   '--frame-id', 'world', '--child-frame-id', 'camera_link'],
    )]


def generate_launch_description():
    driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('astra_camera'), 'launch',
            LaunchConfiguration('camera_launch')])),
        launch_arguments={
            # Alinea la profundidad con la imagen a color (necesario para pose_node)
            'depth_registration': 'true',
            'enable_point_cloud': 'false',
            'enable_ir': 'false',
            'color_width': '640', 'color_height': '480', 'color_fps': '30',
            'depth_width': '640', 'depth_height': '480', 'depth_fps': '30',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_launch', default_value='astra_pro.launch.xml',
            description='Launch del driver: astra_pro.launch.xml (Astra Pro) o '
                        'astro_pro_plus.launch.xml (Astra Pro Plus)'),
        DeclareLaunchArgument('camera_height', default_value='1.0',
                              description='Altura del lente al piso (m)'),
        DeclareLaunchArgument('camera_pitch_deg', default_value='0.0',
                              description='Inclinacion hacia abajo (grados)'),
        driver,
        OpaqueFunction(function=camera_tf),
    ])
