# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch realsense2_camera node."""
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node



configurable_parameters = [
                           {'name': 'base_frame_id',                'default': 'camera_link', 'description': ''},
                           #{'name': 'imu_optical_frame_id',         'default': "'imu_link'", 'description': ''},
                           {'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'enable_pointcloud',            'default': 'true', 'description': 'enable pointcloud'},
                        #    {'name': 'unite_imu_method',             'default': 'copy', 'description': '[copy|linear_interpolation]'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'log_level',                    'default': 'INFO', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},

                           {'name': 'depth_width',                  'default': '424', 'description': 'depth image width'},
                           {'name': 'depth_height',                 'default': '240', 'description': 'depth image height'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},

                           {'name': 'color_width',                  'default': '424', 'description': 'color image width'},
                           {'name': 'color_height',                 'default': '240', 'description': 'color image height'},
                           {'name': 'enable_color',                 'default': 'false', 'description': 'enable color stream'},

                           {'name': 'infra_width',                  'default': '424', 'description': 'infra width'},
                           {'name': 'infra_height',                 'default': '240', 'description': 'infra width'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},

                           {'name': 'confidence_width',             'default': '424', 'description': 'depth image width'},
                           {'name': 'confidence_height',            'default': '240', 'description': 'depth image height'},
                           {'name': 'enable_confidence',            'default': 'false', 'description': 'enable depth stream'},

                        #    {'name': 'fisheye_fps',                  'default': '-1.', 'description': ''},
                           {'name': 'depth_fps',                    'default': '30.', 'description': ''},
                           {'name': 'confidence_fps',               'default': '30.', 'description': ''},
                           {'name': 'infra_fps',                    'default': '30.', 'description': ''},
                           {'name': 'color_fps',                    'default': '30.', 'description': ''},
                        #    {'name': 'gyro_fps',                     'default': '200.', 'description': ''},
                        #    {'name': 'accel_fps',                    'default': '200.', 'description': ''},

                           {'name': 'color_qos',                    'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'confidence_qos',               'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'depth_qos',                    'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'fisheye_qos',                  'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'infra_qos',                    'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'pointcloud_qos',               'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'enable_gyro',                  'default': 'false', 'description': ''},
                           {'name': 'enable_accel',                 'default': 'false', 'description': ''},
                           {'name': 'pointcloud_texture_stream',    'default': 'RS2_STREAM_ANY', 'description': 'testure stream for pointcloud'},
                           {'name': 'pointcloud_texture_index',     'default': '0', 'description': 'testure stream index for pointcloud'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': ''},
                           {'name': 'align_depth',                  'default': 'true', 'description': ''},
                           {'name': 'filters',                      'default': "temporal, decimation, spatial, disparity", 'description': ''},
                        #    {'name': 'filters',                      'default': "decimation", 'description': ''},
                           {'name': 'clip_distance',                'default': '5.', 'description': ''},
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': ''},
                           {'name': 'initial_reset',                'default': 'false', 'description': ''},
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': ''},
                           {'name': 'ordered_pc',                   'default': 'false', 'description': ''},
                           {'name': 'calib_odom_file',              'default': "''", 'description': "''"},
                           {'name': 'topic_odom_in',                'default': "''", 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'temporal.holes_fill',          'default': '0', 'description': 'Persistency mode'},
                           {'name': 'stereo_module.exposure.1',     'default': '7500', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.gain.1',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.exposure.2',     'default': '1', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.gain.2',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    log_level = 'info'

    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_camera',
                    arguments=['0.25', '0', '1.15', '0', '0', '0', '1','base_link','camera_link'],
    )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # Realsense
        launch_ros.actions.Node(
            package='realsense2_camera',
            namespace=LaunchConfiguration("camera_name"),
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters=[set_configurable_parameters(configurable_parameters)
                        ],
            output='screen',
            remappings=[ ('/camera/imu','/imu'),],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
            ),
    ])