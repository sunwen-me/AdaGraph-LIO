import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'sim.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'lio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    base_frame = LaunchConfiguration('base_frame')
    map_file = LaunchConfiguration('map_file')
    use_prior_map = LaunchConfiguration('use_prior_map')

    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[
            config_yaml,
            {
                'use_sim_time': use_sim_time,
                'lio.ros.lidar_topic': lidar_topic,
                'lio.ros.imu_topic': imu_topic,
                'lio.nav2_tf.base_frame': base_frame,
                'lio.prior_map_localization.enable': use_prior_map,
                'lio.prior_map_localization.map_file': map_file,
                'lio.kiss_matcher.enable': use_prior_map,
            },
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='super_lio_rviz',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('lidar_topic', default_value='/mid360/points'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('use_prior_map', default_value='false'),
        super_lio_node,
        rviz2_node,
    ])
