# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPROSVelPub.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.28 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import multiprocessing as mp
import os
import sys
from typing import Tuple
from multiprocessing import Queue

from rospkg import get_package_name
from ament_index_python import get_package_prefix
from rclpy.executors import SingleThreadedExecutor
import threading
# from multiprocessing.connection import PipeConnection
from typing import Union
import time

import psutil
import rclpy

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
ROS_PATH = os.path.join(INCLUDE_PATH, "ROSIntegration")
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(INCLUDE_PATH, "Resources")
MULTIPROCESS_PATH = os.path.join(INCLUDE_PATH, "MultiProcessing")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH,
                 MULTIPROCESS_PATH, ROS_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH, MULTIPROCESS_PATH, ROS_PATH

from ROSIntegration.ROSLaunch import CROSLaunch

gROSNavigation = None
# dict: CMD, VALUE
# pipeChild is an instance of PipeConnection on Windows, but this is different on Linux
def proc_ros_nav_node(pipeChild, feedbackQueue: Queue, feedbackQueueBk: Union[Queue, None]):
    global gROSNavigation

    n_pid = os.getpid()
    str_name = mp.current_process().name
    this_proc = psutil.Process(n_pid)
    while this_proc.is_running():
        try:
            if pipeChild.poll(None):
                dict_cmd = pipeChild.recv()
                try:
                    cmd = dict_cmd["CMD"]
                    val = dict_cmd["VAL"]
                except KeyError:
                    cmd = ""
                    val = ""

                if cmd == "CREATE":
                    gROSLidar = CROSLaunch('ydlidar_ros2_driver','ydlidar_launch')
                    gROSMapping = CROSLaunch('exa_robot', 'cartographer_launch')
                    gROSNavigation = CROSLaunch('exa_robot','bringup_launch')

                elif cmd == "LAUNCH_LIDAR":
                    if gROSLidar == None:
                        gROSLidar = CROSLaunch('ydlidar_ros2_driver','ydlidar_launch')

                    gROSLidar.launch()

                elif cmd == "LAUNCH_CARTO":
                    if gROSMapping == None:
                        gROSMapping = CROSLaunch('exa_robot', 'cartographer_launch')

                    gROSMapping.launch()

                elif cmd == "LAUNCH_NAV":
                    if gROSNavigation == None:
                        gROSNavigation = CROSLaunch('exa_robot','bringup_launch')

                    gROSNavigation.launch()

                elif cmd == "NODE_LIST":    
                    node_list = gROSNavigation.get_child_pid()
                    print('in MP gROSNavigation:', node_list)
                    on_nav_node_list(node_list, feedbackQueue)

                    node_list = gROSLidar.get_child_pid()
                    print('in MP gROSLidar:', node_list)
                    on_nav_node_list(node_list, feedbackQueue)

                    node_list = gROSMapping.get_child_pid()
                    print('in MP gROSMapping:', node_list)
                    on_nav_node_list(node_list, feedbackQueue)

                elif cmd == "RELEASE":
                    gROSNavigation.terminate()
                    gROSLidar.terminate()
                    gROSMapping.terminate()
                    rclpy.shutdown()

                else:
                    pass

                time.sleep(1e-1)

            else:
                time.sleep(1e-1)
        except:
            time.sleep(1e-1)
            pass

        time.sleep(1e-1)
        pass


# def on_pub_cmd_vel(velCmd: Tuple[float, float], q: Queue):
#     # print(velCmd)
#     dict_msg = dict()
#     dict_msg['VEL_CMD'] = velCmd
#     q.put(dict_msg)
    
def on_nav_node_list(node_list, q: Queue):
    dict_msg = dict()
    dict_msg['NODE_LIST'] = node_list
    q.put(dict_msg)
    
