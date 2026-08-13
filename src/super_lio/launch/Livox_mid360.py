import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'livox_360.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'lio.rviz')

    declare_publish_base_tf_arg = DeclareLaunchArgument(
        'publish_base_tf',
        default_value='true',
        description=(
            'Publish base_footprint to base_link; disable when a robot '
            'description already publishes this transform'
        )
    )
    publish_base_tf = LaunchConfiguration('publish_base_tf')

    declare_publish_sensor_tf_arg = DeclareLaunchArgument(
        'publish_sensor_tf',
        default_value='true',
        description=(
            'Publish base_link to the Livox message frame; disable when a '
            'robot description already publishes this transform'
        )
    )
    publish_sensor_tf = LaunchConfiguration('publish_sensor_tf')

    declare_base_link_x_arg = DeclareLaunchArgument(
        'base_link_x',
        default_value='-0.196',
        description='base_link x offset from the front drive axle, in metres'
    )
    base_link_x = LaunchConfiguration('base_link_x')

    declare_sensor_frame_arg = DeclareLaunchArgument(
        'sensor_frame',
        default_value='livox_frame',
        description='Frame ID used by the Livox point cloud and IMU messages'
    )
    sensor_frame = LaunchConfiguration('sensor_frame')

    declare_sensor_x_arg = DeclareLaunchArgument(
        'sensor_x',
        default_value='0.143',
        description='Livox x offset from base_link, in metres'
    )
    sensor_x = LaunchConfiguration('sensor_x')

    declare_sensor_z_arg = DeclareLaunchArgument(
        'sensor_z',
        default_value='0.735',
        description='Livox z offset from base_link, in metres'
    )
    sensor_z = LaunchConfiguration('sensor_z')

    declare_sensor_roll_arg = DeclareLaunchArgument(
        'sensor_roll',
        default_value='3.141592653589793',
        description='Livox mounting roll in radians; pi for upside-down mounting'
    )
    sensor_roll = LaunchConfiguration('sensor_roll')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')

    declare_lidar_type_arg = DeclareLaunchArgument(
        'lidar_type',
        default_value='1',
        description=(
            '1: livox_ros_driver2 CustomMsg; '
            '9: livox_ros_driver2 PointCloud2 with xfer_format=0'
        )
    )
    lidar_type = LaunchConfiguration('lidar_type')

    declare_use_prior_map_arg = DeclareLaunchArgument(
        'use_prior_map',
        default_value='false',
        description='Enable prior-map localization and KISS initialization'
    )
    use_prior_map = LaunchConfiguration('use_prior_map')

    declare_prior_map_file_arg = DeclareLaunchArgument(
        'prior_map_file',
        default_value='',
        description='Absolute path to the prior PCD map'
    )
    prior_map_file = LaunchConfiguration('prior_map_file')

    declare_localization_voxel_size_arg = DeclareLaunchArgument(
        'localization_voxel_size',
        default_value='0.10',
        description='Voxel size used by prior-map VGICP registration'
    )
    localization_voxel_size = LaunchConfiguration(
        'localization_voxel_size'
    )

    declare_kiss_voxel_size_arg = DeclareLaunchArgument(
        'kiss_voxel_size',
        default_value='0.20',
        description='Independent voxel and feature scale used by KISS-Matcher'
    )
    kiss_voxel_size = LaunchConfiguration('kiss_voxel_size')

    declare_kiss_use_quatro_arg = DeclareLaunchArgument(
        'kiss_use_quatro',
        default_value='false',
        description='Use gravity-aligned Quatro rotation estimation in KISS'
    )
    kiss_use_quatro = LaunchConfiguration('kiss_use_quatro')

    declare_kiss_use_ratio_test_arg = DeclareLaunchArgument(
        'kiss_use_ratio_test',
        default_value='true',
        description='Preferred descriptor ratio-test mode for KISS-Matcher'
    )
    kiss_use_ratio_test = LaunchConfiguration('kiss_use_ratio_test')

    declare_kiss_adaptive_ratio_test_arg = DeclareLaunchArgument(
        'kiss_adaptive_ratio_test',
        default_value='true',
        description='Evaluate ratio-on and ratio-off KISS candidates with VGICP'
    )
    kiss_adaptive_ratio_test = LaunchConfiguration(
        'kiss_adaptive_ratio_test'
    )

    declare_save_map_arg = DeclareLaunchArgument(
        'save_map',
        default_value='false',
        description='Accumulate and save a PCD map when the node stops'
    )
    save_map = LaunchConfiguration('save_map')

    declare_save_map_dir_arg = DeclareLaunchArgument(
        'save_map_dir',
        default_value='map',
        description='Output directory for mapping; absolute paths are preferred'
    )
    save_map_dir = LaunchConfiguration('save_map_dir')

    declare_map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map.pcd',
        description='Final PCD filename'
    )
    map_name = LaunchConfiguration('map_name')

    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[
            config_yaml,
            {
                'lio.sensor.lidar_type': ParameterValue(
                    lidar_type, value_type=int
                ),
                'lio.prior_map_localization.enable': ParameterValue(
                    use_prior_map, value_type=bool
                ),
                'lio.prior_map_localization.map_file': prior_map_file,
                'lio.prior_map_localization.map_voxel_size': ParameterValue(
                    localization_voxel_size, value_type=float
                ),
                'lio.prior_map_localization.local_voxel_size': ParameterValue(
                    localization_voxel_size, value_type=float
                ),
                'lio.kiss_matcher.enable': ParameterValue(
                    use_prior_map, value_type=bool
                ),
                'lio.kiss_matcher.voxel_size': ParameterValue(
                    kiss_voxel_size, value_type=float
                ),
                'lio.kiss_matcher.use_quatro': ParameterValue(
                    kiss_use_quatro, value_type=bool
                ),
                'lio.kiss_matcher.use_ratio_test': ParameterValue(
                    kiss_use_ratio_test, value_type=bool
                ),
                'lio.kiss_matcher.adaptive_ratio_test': ParameterValue(
                    kiss_adaptive_ratio_test, value_type=bool
                ),
                'lio.map.save_map': ParameterValue(
                    save_map, value_type=bool
                ),
                'lio.map.save_map_dir': save_map_dir,
                'lio.map.map_name': map_name,
            },
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    base_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        output='screen',
        arguments=[
            '--x', base_link_x,
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link',
        ],
        condition=IfCondition(publish_base_tf)
    )

    sensor_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox',
        output='screen',
        arguments=[
            '--x', sensor_x,
            '--y', '0.0',
            '--z', sensor_z,
            '--roll', sensor_roll,
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', sensor_frame,
        ],
        condition=IfCondition(publish_sensor_tf)
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='super_lio',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(rviz_flag)
    )

    ld = LaunchDescription()
    ld.add_action(declare_publish_base_tf_arg)
    ld.add_action(declare_publish_sensor_tf_arg)
    ld.add_action(declare_base_link_x_arg)
    ld.add_action(declare_sensor_frame_arg)
    ld.add_action(declare_sensor_x_arg)
    ld.add_action(declare_sensor_z_arg)
    ld.add_action(declare_sensor_roll_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_lidar_type_arg)
    ld.add_action(declare_use_prior_map_arg)
    ld.add_action(declare_prior_map_file_arg)
    ld.add_action(declare_localization_voxel_size_arg)
    ld.add_action(declare_kiss_voxel_size_arg)
    ld.add_action(declare_kiss_use_quatro_arg)
    ld.add_action(declare_kiss_use_ratio_test_arg)
    ld.add_action(declare_kiss_adaptive_ratio_test_arg)
    ld.add_action(declare_save_map_arg)
    ld.add_action(declare_save_map_dir_arg)
    ld.add_action(declare_map_name_arg)
    ld.add_action(base_tf_node)
    ld.add_action(sensor_tf_node)
    ld.add_action(super_lio_node)
    ld.add_action(rviz2_node)

    return ld
