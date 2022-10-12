# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPROSFaceRecog.py
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

from ROSIntegration.RosNodeFaceRecog import  CROSNodeFaceRecog

gROSNode = None


# dict: CMD, VALUE
# pipeChild is an instance of PipeConnection on Windows, but this is different on Linux
def proc_ros_face_recog(pipeChild, feedbackQueue: Queue, feedbackQueueBk: Union[Queue, None]):
    global gROSNode

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
                    str_topic_name = str(val)
                    rclpy.init(args=None)
                    gROSNode = CROSNodeFaceRecog(str_topic_name)
                    gROSNode.sig_face_dist.connect(lambda x: on_pub_pos_dist(x, feedbackQueue))
                    gROSNode.sig_face_xyz.connect(lambda x: on_pub_pos_xyz(x, feedbackQueue))
                    gROSNode.sig_cam_frames.connect(lambda x: on_pub_cam_frames(x, feedbackQueueBk))
                    thisExecutor = SingleThreadedExecutor()
                    thisExecutor.add_node(gROSNode)
                    thisExecutorThread = threading.Thread(target=thisExecutor.spin, daemon=True)
                    thisExecutorThread.start()
                    rclpy.spin(gROSNode)

                elif cmd == "DESTROY":
                    gROSNode.destroy_node()

                elif cmd == "RELEASE":
                    gROSNode.destroy_node()
                    rclpy.shutdown()
                else:
                    pass

                time.sleep(1e-6)

            else:
                time.sleep(1e-6)
        except:
            time.sleep(1e-6)
            pass

        time.sleep(1e-6)
        pass


def on_pub_cam_frames(cam_frames, q: Queue):
    q.put(cam_frames)



def on_pub_pos_xyz(pos_data: Tuple[float, float, float], q: Queue):
    dict_msg = dict()
    dict_msg['POS_XYZ'] = pos_data
    q.put(dict_msg)


def on_pub_pos_dist(distance: float, q: Queue):
    dict_msg = dict()
    dict_msg['DIST'] = distance
    q.put(dict_msg)
