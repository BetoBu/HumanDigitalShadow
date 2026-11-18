"""
Launch principal de la sombra digital humana.

  camara -> pose_node -> /human/keypoints -> shadow_node -> /joint_states
                                                         -> /human/cmd/* -> Gazebo

Ejemplos:
  # Con la camara, Gazebo y RViz
  ros2 launch human_shadow shadow.launch.py camera_height:=1.0 camera_pitch_deg:=10

  # Sin camara: persona virtual de demostracion
  ros2 launch human_shadow shadow.launch.py source:=demo

  # Solo RViz (sin Gazebo)
  ros2 launch human_shadow shadow.launch.py gazebo:=false
"""

import os

from human_shadow.kinematics import ALL_JOINTS
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('human_shadow')
    cfg = LaunchConfiguration
    use_camera = PythonExpression(["'", cfg('source'), "' == 'camera'"])
    use_demo = PythonExpression(["'", cfg('source'), "' == 'demo'"])

    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([pkg, 'urdf', 'human.urdf.xacro']),
        ' body_config:=', cfg('body_config')]), value_type=str)

    args = [
        DeclareLaunchArgument('source', default_value='camera',
                              description='camera = Astra Pro, demo = persona virtual'),
        DeclareLaunchArgument('gazebo', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = Gazebo sin ventana (solo servidor)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('camera_height', default_value='1.0'),
        DeclareLaunchArgument('camera_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('camera_launch', default_value='astra_pro.launch.xml'),
        DeclareLaunchArgument(
            'model_path',
            default_value=os.path.expanduser('~/models/pose_landmarker_full.task')),
        DeclareLaunchArgument(
            'body_config', default_value=PathJoinSubstitution([pkg, 'config', 'body.yaml'])),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'camera.launch.py'])),
        launch_arguments={
            'camera_height': cfg('camera_height'),
            'camera_pitch_deg': cfg('camera_pitch_deg'),
            'camera_launch': cfg('camera_launch'),
        }.items(),
        condition=IfCondition(use_camera),
    )
    pose = Node(
        package='human_shadow', executable='pose_node', output='screen',
        parameters=[{
            'model_path': cfg('model_path'),
            'camera_height': ParameterValue(cfg('camera_height'), value_type=float),
            'camera_pitch_deg': ParameterValue(cfg('camera_pitch_deg'), value_type=float),
        }],
        condition=IfCondition(use_camera),
    )
    demo = Node(package='human_shadow', executable='demo_node', output='screen',
                parameters=[{'body_config': cfg('body_config')}],
                condition=IfCondition(use_demo))
    shadow = Node(package='human_shadow', executable='shadow_node', output='screen',
                  parameters=[{'body_config': cfg('body_config')}])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description}])
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'shadow.rviz'])],
                condition=IfCondition(cfg('rviz')))

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r ', PythonExpression(
            ["'-s ' if '", cfg('headless'), "' == 'true' else ''"]), PathJoinSubstitution(
            [pkg, 'worlds', 'shadow_world.sdf'])]}.items(),
        condition=IfCondition(cfg('gazebo')),
    )
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'shadow_world', '-name', 'human',
                            '-topic', 'robot_description'],
                 condition=IfCondition(cfg('gazebo')))
    bridge_topics = [f'/human/cmd/{j}@std_msgs/msg/Float64]ignition.msgs.Double'
                     for j in ALL_JOINTS]
    bridge_topics.append('/human/gz_joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=bridge_topics, condition=IfCondition(cfg('gazebo')))

    return LaunchDescription(args + [camera, pose, demo, shadow, rsp, rviz, gz, spawn, bridge])
