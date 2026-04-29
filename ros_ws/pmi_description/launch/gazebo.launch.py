"""Gazebo Classic empty world + spawn URDF."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('pmi_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'pmi_description.urdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'empty_world.launch.py',
            ])
        )
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'base_link', 'base_footprint',
        ],
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'pmi_description',
            '-file', urdf_path,
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        static_tf,
        TimerAction(period=2.0, actions=[spawn]),
    ])
