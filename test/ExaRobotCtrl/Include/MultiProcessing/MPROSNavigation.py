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
from ROSIntegration.ROSNavigator import CROSNavigation

gROSNavigation = None
gROSNavigator = None
# dict: CMD, VALUE
# pipeChild is an instance of PipeConnection on Windows, but this is different on Linux
def proc_ros_nav_node(pipeChild, feedbackQueue: Queue, feedbackQueueBk: Union[Queue, None]):
    global gROSNavigation, gROSNavigator

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
                    # gROSLidar = CROSLaunch('urg_node', 'urg_node_launch')
                    gROSMapping = CROSLaunch('exa_robot', 'cartographer_launch')
                    mode = val
                    #Navigation
                    if mode == '2D':
                        gROSNavigation = CROSLaunch('exa_robot','exa_nav_2d_launch')
                    elif mode == '3D':
                        # gROSNavigation = CROSLaunch('exa_robot','exa_nav_3d_launch')
                        # pass
                        gROSNavigation = CROSLaunch('exa_robot','exa_nav_3d_2_launch')

                    else:
                        pass

                    
            
                elif cmd == "LAUNCH":
                    gROSNavigation.launch()
                    gROSLidar.launch()
                    gROSMapping.launch()
                    # time.sleep(1)
                    
                    
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
                
                # elif cmd == "NAV_COMMANDER":
                #     rclpy.init(args = None)
                #     gROSNavigator = CROSNavigation()     
                
                elif cmd == "MOVE_CANCLE":
                    gROSNavigator.cancel_nav()
                    

                # elif cmd == 'MOVE_POSE':
                #     xPos, yPos, aAngle = val 
                #     print("MP MOVE_POSE", val)
                #     pose = gROSNavigator.make_pose([xPos, yPos])
                #     gROSNavigator.go_to_pose(pose)
                    
                elif cmd == 'MOVE_WAYPOINT':
                    pass

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
    
    # print('Send: ',dict_msg)
