"""RViz2 + joint_state_publisher_gui 슬라이더로 관절 조절.

설치::

    sudo apt install ros-${ROS_DISTRO}-joint-state-publisher-gui

실행::

    source install/setup.bash
    ros2 launch pmi_description3 display_gui.launch.py

RViz 레이아웃은 ``config/pmi_description3.rviz`` 를 사용합니다.
다른 파일을 쓰려면::

    ros2 launch pmi_description3 display_gui.launch.py rviz_config:=/절대/경로/custom.rviz
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
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
