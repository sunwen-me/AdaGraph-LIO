import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_hybrid_slam = get_package_share_directory('hybrid_slam')
    config_yaml = os.path.join(
        pkg_hybrid_slam, 'config', 'point_lio_m2dgr.yaml'
    )
    rviz_config = os.path.join(pkg_hybrid_slam, 'rviz', 'point_lio.rviz')

    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz while replaying an M2DGR bag',
        ),
        Node(
            package='hybrid_slam',
            executable='point_lio_node',
            name='hybrid_slam_point_lio',
            output='screen',
            parameters=[config_yaml],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='hybrid_slam_point_lio_rviz',
            arguments=[
                '-d', rviz_config, '--ros-args', '--log-level', 'warn'
            ],
            condition=IfCondition(rviz),
        ),
    ])
