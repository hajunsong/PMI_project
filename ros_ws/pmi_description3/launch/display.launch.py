"""RViz2에서 pmi_description3 URDF 표시 (정적 관절: joint_state_publisher).

빌드 후::

    source install/setup.bash
    ros2 launch pmi_description3 display.launch.py

RViz 설정 기본값: 패키지 ``config/pmi_description3.rviz`` (Grid + RobotModel + TF).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('pmi_description3')
    urdf_path = os.path.join(pkg_share, 'urdf', 'pmi_description3.urdf')
    rviz_default = os.path.join(pkg_share, 'config', 'pmi_description3.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Subscribe to /clock (simulation)',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_default,
            description='RViz2 설정 파일 경로 (.rviz)',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
