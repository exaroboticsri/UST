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

gROSRealsense = None
# dict: CMD, VALUE
# pipeChild is an instance of PipeConnection on Windows, but this is different on Linux
def proc_ros_rs_node(pipeChild, feedbackQueue: Queue, feedbackQueueBk: Union[Queue, None]):
    global gROSRealsense

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
                    gROSRealsense = CROSLaunch('exa_robot','rs_launch')
                    # print("launch start")
                    # gROSRealsense = CROSLaunch('exa_robot','rs_multi_camera_launch')
        
                        
                elif cmd == "LAUNCH":
                    gROSRealsense.launch()
                
                elif cmd == "NODE_LIST":    
                    node_list = gROSRealsense.get_child_pid()
                    # print('in MP:', node_list)
                    on_nav_node_list(node_list, feedbackQueue)
                    
                elif cmd == "RELEASE":
                    gROSRealsense.terminate()
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


def on_nav_node_list(node_list, q: Queue):
    dict_msg = dict()
    dict_msg['NODE_LIST'] = node_list
    q.put(dict_msg)