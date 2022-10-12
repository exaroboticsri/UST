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


# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_multi_camera_launch.py camera_name1:=my_D435 device_type1:=d435 camera_name2:=my_d415 device_type2:=d415 

"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch
import os
local_parameters = [{'name': 'camera_name1', 'default': 'camera_front_up', 'description': 'camera unique name'},
                    {'name': 'camera_name2', 'default': 'camera_front_down', 'description': 'camera unique name'},
                    {'name': 'serial_no1', 'default': '_136522073236', 'description': ''},
                    {'name': 'serial_no2', 'default': '_141722073310', 'description': ''},
                    {'name': 'base_frame_id1', 'default': 'camera_link1', 'description': ''},
                    {'name': 'base_frame_id2', 'default': 'camera_link2', 'description': ''},
                    {'name': 'depth_width1', 'default':  '424', 'description': ''},
                    {'name': 'depth_height1', 'default': '240', 'description': ''},
                    {'name': 'depth_width2', 'default':  '424', 'description': ''},
                    {'name': 'depth_height2', 'default': '240', 'description': ''},
                    {'name': 'depth_fps1', 'default': '30.', 'description': ''},
                    {'name': 'depth_fps2', 'default': '30.', 'description': ''},
                    {'name': 'enable_color1', 'default': 'true', 'description': 'enable color stream'},
                    {'name': 'enable_color2', 'default': 'false', 'description': 'enable color stream'},
                    {'name': 'align_depth1', 'default': 'false', 'description': ''},
                    {'name': 'align_depth2', 'default': 'false', 'description': ''},
                    {'name': 'enable_pointcloud1', 'default': 'false', 'description': 'enable pointcloud'},
                    {'name': 'enable_pointcloud2', 'default': 'true', 'description': 'enable pointcloud'},
                    {'name': 'filters1', 'default': "''", 'description': ''},






                    
                   ]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

    

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('exa_robot'), 'launch')
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) + 
        rs_launch.declare_configurable_parameters(params2) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/rs_launch.py']),
            launch_arguments=set_configurable_parameters(params1).items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/rs_launch.py']),
            launch_arguments=set_configurable_parameters(params2).items(),
        ),
        # dummy static transformation from camera1 to camera2
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments=['0.25', '0', '1.05', '0', '0', '0','base_link','camera_link1'],
        ),
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments=['0.245', '0', '0.977', '0','0.773', '0', 'base_link','camera_link2'],
            # arguments = ["-0.005", "0", "-0.073", "0", "0.733", "0", "base_link", "camera_link2"]
        ),
    ])
