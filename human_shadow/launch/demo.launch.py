"""Sombra digital sin camara: persona virtual + Gazebo + RViz."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('human_shadow'), 'launch', 'shadow.launch.py'])),
        launch_arguments={'source': 'demo'}.items(),
    )])
