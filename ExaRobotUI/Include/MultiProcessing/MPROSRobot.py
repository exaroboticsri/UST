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

from ROSIntegration.ROSNodeRobot import CROSNodeRobot
from ROSIntegration.ROSLaunch import CROSLaunch
from ROSIntegration.ROSNavigator import CROSNavigator

gROSRobotNode = None
# dict: CMD, VALUE
# pipeChild is an instance of PipeConnection on Windows, but this is different on Linux
def proc_ros_robot_node(pipeChild, feedbackQueue: Queue, feedbackQueueBk: Union[Queue, None]):
    global gROSRobotNode

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
                    rclpy.init(args=None)
                    tf_pub_period, name_topic_odom, name_topic_vel = val
                    gROSRobotNode = CROSNodeRobot(tf_pub_period, name_topic_odom, name_topic_vel)
                    gROSRobotNode.sig_ros_sub_cmd_vel.connect(lambda x: on_sub_cmd_vel(x, feedbackQueue))
                    thisExecutor = SingleThreadedExecutor()
                    thisExecutor.add_node(gROSRobotNode)
                    thisExecutorThread = threading.Thread(target=thisExecutor.spin, daemon=False)
                    thisExecutorThread.start()
                
                    
                    
                 
                elif cmd == "START_TIMER":
                    # print('MP TF PUB START TIMER')
                    gROSRobotNode.start_timer()

                elif cmd == "STOP_TIMER":
                    # print('MP TF PUB STOP TIMER')
                    gROSRobotNode.stop_timer()

                elif cmd == "UPDATE_ODOM":
                    x, y, theta, vel_c, omega_c = val
                    # print('MPROSTfPub', x, y, theta)
                    gROSRobotNode.update_odom_and_tf(x, y, theta, vel_c, omega_c)
                    
                elif cmd == "RELEASE":
                    # gROSRobotNode.destroy_topic()
                    gROSRobotNode.destroy_node()
                    thisExecutor.shutdown()
                    
                    rclpy.shutdown()

                    
                else:
                    pass

                time.sleep(1e-3)

            else:
                time.sleep(1e-3)
        except:
            time.sleep(1e-3)
            pass

        time.sleep(1e-3)
        pass


def on_sub_cmd_vel(velCmd: Tuple[float, float], q: Queue):
    # print(velCmd)
    dict_msg = dict()
    dict_msg['VEL_CMD'] = velCmd
    q.put(dict_msg)
