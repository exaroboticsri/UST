import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

import lifecycle_msgs.msg
from launch.actions import LogInfo
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    pkg_name = 'exa_robot'
    cartographer_prefix = os.path.join(get_package_share_directory(pkg_name), 'config')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=cartographer_prefix)
    configuration_basename = LaunchConfiguration('configuration_basename', default='slam_local.lua')
   
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')

    DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
    DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),

    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            #output='screen',
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
                '-load_state_filename', '/home/jw/robot/slam_data/map.pbstream', 
                ],
            remappings=[('/imu','/camera_front_up/imu')],
                # remappings=[('/scan','/scan_rs')],
            )

    DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

    DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

    cartographer_launch_file_dir = os.path.join(get_package_share_directory(pkg_name), 'launch')
    occupancy_grid_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cartographer_launch_file_dir , '/occupancy_grid_launch.py']),
            launch_arguments={
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
                }.items(),
        )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0','0', '0', '0', '1','map','odom'],
                    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_launch,
        tf2_node,
    ])
