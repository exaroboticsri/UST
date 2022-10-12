# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ControlCore.py
# Project Name : MobileRobotTest
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.08.24 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import queue
import threading
import sys
import time
from typing import Tuple, List, Dict
import multiprocessing as mp

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = os.path.dirname(FILE_PATH)

INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
PATH_PLANNER_PATH = os.path.join(INCLUDE_PATH, "PathPlanning")
MOBILE_ROBOT_PATH = os.path.join(INCLUDE_PATH, "MobileRobot")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
MULTIPROC_PATH = os.path.join(INCLUDE_PATH, "MultiProcessing")
TTS_PATH = os.path.join(INCLUDE_PATH, 'AudioRecognition')
LIBRATY_PATH = os.path.join(ROOT_PATH, "Library")
CAMERA_DEV_PATH = os.path.join(LIBRATY_PATH, "Devices/Camera")
sys.path.extend(
    [FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, PATH_PLANNER_PATH, MOBILE_ROBOT_PATH,
     MULTIPROC_PATH, CAMERA_DEV_PATH, TTS_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, PATH_PLANNER_PATH, MOBILE_ROBOT_PATH, MULTIPROC_PATH, CAMERA_DEV_PATH, TTS_PATH

from Commons import *

from MultiProcessing.MultiProcessBase import CMultiProcessBase
from MultiProcessing.MultiProcessBase import CMultiProcessBase
import MultiProcessing.MPMobileRobot as MPMobileRobot
import MultiProcessing.MPExternalComm as MPExternalComm
import MultiProcessing.MPTextToSpeech as MPTextToSpeech

from AudioRecognition.TextToSpeech import CTextToSpeech
from AudioRecognition.SpeechToText import CSpeechToText
from Library.Devices.Camera.CameraCommon import CCameraCommon

if is_ros_installed():
    import MultiProcessing.MPROSVelPub as MPROSVelPub
    import MultiProcessing.MPROSRobot as MPROSRobot
    import MultiProcessing.MPROSNavigation as MPROSNavigation
    import MultiProcessing.MPROSRealsense as MPROSRealsense
    import MultiProcessing.MPROSFaceRecog as MPROSFaceRecog
    from ROSIntegration.ROSNavigator import CROSNavigation


class EmbdControlCore(object):
    # queuePlotter: queue.Queue = None
    # threadingPlotter: ThreadPlotter = None
    # Robot
    _mpStellaB2: CMultiProcessBase = None
    _mpKobuki2: CMultiProcessBase = None
    _mpExaRobot: CMultiProcessBase = None
    # ROS
    _mpROSCmdVelPub: CMultiProcessBase = None
    _mpROSCmdVelSub: CMultiProcessBase = None
    _mpROSRobotNode: CMultiProcessBase = None
    _mpROSNavigation: CMultiProcessBase = None
    _mpROSRealsense: CMultiProcessBase = None
    _mpROSFaceRecog: CMultiProcessBase = None
    # Etc
    _mpCamTest: CMultiProcessBase = None
    _mpExternalComm: CMultiProcessBase = None
    _mpTextToSpeech: CMultiProcessBase = None
    _ttsHandler: CTextToSpeech = None
    _sttHandler: CSpeechToText = None

    _dictProc: dict
    _camTest: CCameraCommon = None

    def __init__(self):
        self._face_pos_z = 0.
        self._face_pos_y = 0.
        self._face_pos_x = 0.
        self._face_distance = 0.
        self._current_pos_x = 0.
        self._current_pos_y = 0.
        self._current_pos_theta = 0.
        write_log("Started Control Core", self)
        super(EmbdControlCore, self).__init__()
        # self.StartThreadPlanner()
        # tts handler
        # self._ttsHandler = CTextToSpeech()

        # stt handler
        self.sig_stt_msg = PySignal(str)
        # self._sttHandler = CSpeechToText()
        # self._sttHandler.sig_stt_msg.connect(lambda x: self.sig_stt_msg.emit(x))

        # stella_b2 signals
        self.sig_stellab2_connected = PySignal()
        self.sig_stellab2_disconnected = PySignal()
        self.sig_stellab2_comm_error = PySignal()
        self.sig_stellab2_comm_status = PySignal(str)
        self.sig_stellab2_is_monitoring = PySignal(bool)
        self.sig_stellab2_get_vel = PySignal(float, float)
        self.sig_stellab2_get_enc = PySignal(int, int)
        self.sig_stellab2_get_wheel_pos = PySignal(float)
        self.sig_stellab2_get_pos = PySignal(float, float, float)
        self.sig_stellab2_get_state = PySignal(int)
        self.sig_stellab2_get_ver = PySignal(str)

        # kobuki2 signals
        self.sig_kobuki2_connected = PySignal()
        self.sig_kobuki2_disconnected = PySignal()
        self.sig_kobuki2_comm_error = PySignal()
        self.sig_kobuki2_comm_status = PySignal(str)
        self.sig_kobuki2_get_vel = PySignal(float, float)
        self.sig_kobuki2_get_enc = PySignal(int, int)
        self.sig_kobuki2_get_wheel_pos = PySignal(float)
        self.sig_kobuki2_get_pos = PySignal(float, float, float)
        self.sig_kobuki2_get_state = PySignal(int)
        self.sig_kobuki2_get_ver = PySignal(str)
        self.sig_kobuki2_get_hw_ver = PySignal(str)

        # exarobot signals
        self.sig_exarobot_connected = PySignal()
        self.sig_exarobot_disconnected = PySignal()
        self.sig_exarobot_comm_error = PySignal()
        self.sig_exarobot_comm_status = PySignal(str)
        self.sig_exarobot_get_vel = PySignal(float, float)
        self.sig_exarobot_get_enc = PySignal(int, int)
        self.sig_exarobot_get_wheel_pos = PySignal(float)
        self.sig_exarobot_get_pos = PySignal(float, float, float)
        self.sig_exarobot_get_state = PySignal(int)

        # external comm signals
        self.sig_external_comm_server_status = PySignal(str)
        self.sig_external_comm_client_connected = PySignal(tuple)
        self.sig_external_comm_client_disconnected = PySignal(tuple)

        # ros face recog
        self.sig_facerecog_stream = PySignal(tuple)
        self.sig_facerecog_dist = PySignal(float)
        self.sig_facerecog_xyz = PySignal(tuple)

        # camera
        self.sig_cam_recv_stream = PySignal(tuple)
        self.sig_cam_stopped_stream = PySignal()

        # process manager
        self.sig_proc_monitor_pid = PySignal(dict)
        self._dictProc = dict()

        # multiprocessing
        self.init_multi_processes()
        self.stellab2_create()
        self.kobuki2_create()
        self.exarobot_create()
        self.external_comm_create()
        self.ros_face_recog_create()
        # self.ros_vel_pub_create(0.05, "cmd_vel")
        self.tts_create()
        self.tts_send_msg('Robot is Ready!')


        

    def release(self):
        self.stop_multi_processes()
        write_log("Release Control Core", self)
        # self.StopThreadPlanner()

    def init_multi_processes(self):
        try:
            # main process
            # mp.set_start_method('fork')
            appProc = mp.current_process()
            pid = appProc.pid
            self._dictProc['MAIN'] = pid
            this_proc = psutil.Process(pid)
            this_proc.nice(0)
            # set_cpu_affinity(appProc.pid, {1})
        except:
            pass

        try:
            self._mpStellaB2 = CMultiProcessBase('Stella B2', MPMobileRobot.proc_mobile_robot, False, True)
            self._mpStellaB2.sig_queue_bcast.connect(self.on_stellab2_queue_bcast)
            self._mpStellaB2.sig_error.connect(self.on_mp_error)
            self._mpStellaB2.start()
            self._dictProc[self._mpStellaB2.NAME] = self._mpStellaB2.PID

            self._mpKobuki2 = CMultiProcessBase('Kobuki2', MPMobileRobot.proc_mobile_robot, False, True)
            self._mpKobuki2.sig_queue_bcast.connect(self.on_kobuki2_queue_bcast)
            self._mpKobuki2.sig_error.connect(self.on_mp_error)
            self._mpKobuki2.start()
            self._dictProc[self._mpKobuki2.NAME] = self._mpKobuki2.PID

            self._mpExaRobot = CMultiProcessBase('ExaRobot', MPMobileRobot.proc_mobile_robot, False, False)
            self._mpExaRobot.sig_queue_bcast.connect(self.on_exarobot_queue_bcast)
            self._mpExaRobot.sig_error.connect(self.on_mp_error)
            self._mpExaRobot.start()
            self._dictProc[self._mpExaRobot.NAME] = self._mpExaRobot.PID
            # set_cpu_affinity(self._mpExaRobot.PID, {1})

            self._mpExternalComm = CMultiProcessBase('ExternalComm', MPExternalComm.proc_external_comm, False, False)
            self._mpExternalComm.sig_queue_bcast.connect(self.on_external_comm_queue_bcast)
            self._mpExternalComm.sig_error.connect(self.on_mp_error)
            self._mpExternalComm.start()
            self._dictProc[self._mpExternalComm.NAME] = self._mpExternalComm.PID

            self._mpTextToSpeech = CMultiProcessBase('TextToSpeech', MPTextToSpeech.proc_text_to_speech, False, False)
            self._mpTextToSpeech.sig_error.connect(self.on_mp_error)
            self._mpTextToSpeech.start()
            self._dictProc[self._mpTextToSpeech.NAME] = self._mpTextToSpeech.PID
            # set_cpu_affinity(self._mpExternalComm.PID, {1})
            if is_ros_installed():
                # self._mpROSCmdVelPub = CMultiProcessBase('ROS Cmd Vel Pub', MPROSVelPub.proc_ros_vel_node, False, True)
                # self._mpROSCmdVelPub.sig_queue_bcast.connect(self.on_ros_cmd_vel_pub)
                # self._mpROSCmdVelPub.sig_error.connect(self.on_mp_error)
                # self._mpROSCmdVelPub.start()
                # self._dictProc[self._mpROSCmdVelPub.NAME] = self._mpROSCmdVelPub.PID

                self._mpROSRobotNode = CMultiProcessBase('ROS Robot Node', MPROSRobot.proc_ros_robot_node, False, False)
                self._mpROSRobotNode.sig_queue_bcast.connect(self.on_ros_robot_node_vel_sub)
                self._mpROSRobotNode.sig_error.connect(self.on_mp_error)
                self._mpROSRobotNode.start()
                self._dictProc[self._mpROSRobotNode.NAME] = self._mpROSRobotNode.PID
                # set_cpu_affinity(self._mpROSRobotNode.PID, {1})

                self._mpROSNavigation = CMultiProcessBase('ROS Robot Navigation', MPROSNavigation.proc_ros_nav_node,
                                                          False, False)
                self._mpROSNavigation.sig_error.connect(self.on_mp_error)
                self._mpROSNavigation.sig_queue_bcast.connect(self.on_nav_queue_bcast)
                self._mpROSNavigation.start()
                self._dictProc[self._mpROSNavigation.NAME] = self._mpROSNavigation.PID
                # set_cpu_affinity(self._mpROSNavigation.PID, {0,3,4,5})

                self._mpROSRealsense = CMultiProcessBase('ROS Robot Realsense', MPROSRealsense.proc_ros_rs_node, False,
                                                         False)
                self._mpROSRealsense.sig_error.connect(self.on_mp_error)
                self._mpROSRealsense.sig_queue_bcast.connect(self.on_ros_rs_node_queue_bcast)
                self._mpROSRealsense.start()
                self._dictProc[self._mpROSRealsense.NAME] = self._mpROSRealsense.PID

                self._mpROSFaceRecog = CMultiProcessBase('ROS Face Recognition', MPROSFaceRecog.proc_ros_face_recog,
                                                         True, False)
                self._mpROSFaceRecog.sig_error.connect(self.on_mp_error)
                self._mpROSFaceRecog.sig_queue_bcast.connect(self.on_ros_facerecog_pos)
                self._mpROSFaceRecog.sig_queue_bcast_bk.connect(self.on_ros_facerecog_camframe)
                self._mpROSFaceRecog.start()
                self._dictProc[self._mpROSFaceRecog.NAME] = self._mpROSFaceRecog.PID

                # set_cpu_affinity(self._mpROSRealsense.PID, {2})
        except:
            pass

    def get_proc_info(self):
        self.sig_proc_monitor_pid.emit(self._dictProc)

    def stop_multi_processes(self):
        try:
            # self.stellab2_release()
            # self._mpStellaB2.release()
            # self.kobuki2_release()
            # self._mpKobuki2.release()
            # self.exarobot_release()
            self._mpExaRobot.release()
            self.external_comm_release()
            self._mpExternalComm.release()

            if is_ros_installed():
                # self._mpROSCmdVelPub.release()
                self._mpROSNavigation.release()
                self._mpROSRobotNode.release()
                self._mpROSRealsense.release()
                self._mpROSFaceRecog.release()
        except:
            pass

    ### Kobuki2 ####
    def send_cmd_kobuki2(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        self._mpKobuki2.send_command(dictCmd)

    def kobuki2_create(self):
        self.send_cmd_kobuki2('CREATE', 'KOBUKI2')

    def kobuki2_release(self):
        self.send_cmd_kobuki2('RELEASE')

    def kobuki2_connect(self, strComPort: str):
        self.send_cmd_kobuki2('CONNECT', strComPort)
        print('Robot Node Create')
        self.ros_robot_node_create(0.02, 'odom', 'cmd_vel')
        print('Robot Node Timer Start (/tf, /odom publish)')
        self.ros_robot_node_start_timer()
        # print('Robot Realsense Node Launching')
        # self.ros_rs_node_create()
        # self.ros_rs_node_launch()
        # print('Robot Navigation Stack Launching')
        # self.ros_nav_node_create('3D')
        # self.ros_nav_node_launch_nav_stack()

    def kobuki2_disconnect(self):
        self.nav.release()
        # print("###ros_rs_node_release")
        # self.ros_rs_node_release()
        # print("###ros_nav_node_release")
        # self.ros_nav_node_release()
        # ("###ros_robot_node_stop_timer")

        self.ros_robot_node_stop_timer()
        print("###ros_robot_node_release")
        self.ros_robot_node_release()
        print("###send_cmd_exarobot DISCONNECT")
        self.send_cmd_kobuki2('DISCONNECT')

    def kobuki2_reset_motor(self):  # send sound sequence 1
        self.send_cmd_kobuki2('RESET_MOTOR')

    def kobuki2_factory_reset(self):  # send sound sequence 0
        self.send_cmd_kobuki2('INIT_MOTOR')

    # todo: make this extendable to other robots
    def kobuki2_move_joint_space(self, right_vel: int, left_vel: int):
        self.send_cmd_kobuki2("MOVE_JOINT_SPACE", (right_vel, left_vel))

    def kobuki2_set_velocity_control(self, vel: float, ang_vel: float):
        self.send_cmd_kobuki2("SET_VELOCITY_CONTROL", (vel, ang_vel))

    def kobuki2_stop_robot(self, stop_type: int):
        self.send_cmd_kobuki2("STOP", int(stop_type))

    def kobuki2_get_pos(self):
        self.send_cmd_kobuki2("POS")

    def on_kobuki2_queue_bcast(self, msg: dict):
        try:
            if "VERSION" in msg.keys():
                version = msg['VERSION']
                self.sig_kobuki2_get_ver.emit(version)

            elif "HW_VERSION" in msg.keys():
                version = msg['HW_VERSION']
                self.sig_kobuki2_get_hw_ver.emit(version)

            elif "COMM_STATUS" in msg.keys():
                comm_status = msg['COMM_STATUS']
                self.sig_kobuki2_comm_status.emit(comm_status)

            elif "ENCODER" in msg.keys():
                right, left = msg["ENCODER"]
                self.sig_kobuki2_get_enc.emit(int(right), int(left))

            elif "VELOCITY" in msg.keys():
                right, left = msg["VELOCITY"]
                self.sig_kobuki2_get_vel.emit(float(right), float(left))

            elif "WHEEL_POSITION" in msg.keys():
                right, left = msg["WHEEL_POSITION"]
                self.sig_kobuki2_get_wheel_pos.emit(float(right), float(left))

            elif "STATUS" in msg.keys():
                state = msg["STATUS"]
                self.sig_kobuki2_get_state.emit(int(state))

            elif "POSITION" in msg.keys():
                x, y, theta = msg["POSITION"]

                if is_ros_installed():
                    self.ros_tf_pub_update_odom(x, y, theta, 0., 0.)
            else:
                pass

        except:
            pass

    ### ROS CMD_VEL Publisher ####
    def send_cmd_ros_vel_pub(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSCmdVelPub.send_command(dictCmd)

    # todo: fPeriod should be a paramater of start_timer
    def ros_vel_pub_create(self, fPeriod: float, strTopicName: str):
        createParam = (fPeriod, strTopicName)
        self.send_cmd_ros_vel_pub("CREATE", createParam)

    def ros_vel_pub_start_timer(self):
        self.send_cmd_ros_vel_pub("START_TIMER")

    def ros_vel_pub_stop_timer(self):
        self.send_cmd_ros_vel_pub("STOP_TIMER")

    def ros_vel_pub_clear(self):
        self.send_cmd_ros_vel_pub("CLEAR")

    def ros_vel_pub_add_vel(self, velCmds: Tuple[List[float], List[float]]):
        self.send_cmd_ros_vel_pub("ADD_VEL_CMD", velCmds)

    def ros_vel_pub_reset_vel(self):
        self.send_cmd_ros_vel_pub("RESET_VEL_CMD")

    def ros_vel_pub_reg_vel(self, velCmds: Tuple[List[float], List[float]]):
        self.send_cmd_ros_vel_pub("REG_VEL_CMD", velCmds)

    def ros_vel_pub_destroy(self):
        self.send_cmd_ros_vel_pub("DESTROY")

    def ros_vel_pub_release(self):
        self.send_cmd_ros_vel_pub("RELEASE")

    def on_ros_cmd_vel_pub(self, msg: str):
        if msg == "LAST_CMD":
            self.ros_vel_pub_stop_timer()
            self.ros_vel_pub_reset_vel()

    ### ExaRobot ####
    def send_cmd_exarobot(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        self._mpExaRobot.send_command(dictCmd)

    def exarobot_create(self):
        self.send_cmd_exarobot('CREATE', 'EXAROBOT')

    def exarobot_release(self):
        self.send_cmd_exarobot('RELEASE')

    def exarobot_connect(self, strComPort: str):
        print('exarobot_connecting')
        self.send_cmd_exarobot('CONNECT', strComPort)

        print('Robot Node Create')
        self.ros_robot_node_create(0.02, 'odom', 'cmd_vel')
        print('Robot Node Timer Start (/tf, /odom publish)')
        self.ros_robot_node_start_timer()

        print('Robot Realsense Node Launching')
        self.ros_rs_node_create()
        self.ros_rs_node_launch()
        print('Robot Navigation Stack Launching')
        self.ros_nav_node_create('3D')
        self.ros_nav_node_launch_nav_stack()

        self.nav = CROSNavigation()

        self.ros_nav_node_request_pid_list()
        self.ros_rs_node_request_pid_list()
        self.tts_send_msg('Navigation is On')

        # pose = self.nav.make_pose([1.0, 0.])
        # self.nav.go_to_pose(pose)
# 
    def exarobot_disconnect(self):
        self.nav.release()
        print("###ros_rs_node_release")
        self.ros_rs_node_release()
        print("###ros_nav_node_release")
        self.ros_nav_node_release()
        print("###ros_robot_node_stop_timer")
        self.ros_robot_node_stop_timer()
        print("###ros_robot_node_release")
        self.ros_robot_node_release()
        print("###send_cmd_exarobot DISCONNECT")
        self.send_cmd_exarobot('DISCONNECT')

    def exarobot_reset_motor(self):  # send sound sequence 1
        self.send_cmd_exarobot('RESET_MOTOR')

    def exarobot_factory_reset(self):  # send sound sequence 0
        self.send_cmd_exarobot('INIT_MOTOR')

    # todo: make this extendable to other robots
    def exarobot_move_joint_space(self, right_vel: int, left_vel: int):
        self.send_cmd_exarobot("MOVE_JOINT_SPACE", (right_vel, left_vel))

    def exarobot_set_velocity_control(self, vel: float, ang_vel: float):
        self.send_cmd_exarobot("SET_VELOCITY_CONTROL", (vel, ang_vel))

    def exarobot_stop_robot(self, stop_type: int):
        self.send_cmd_exarobot("STOP", int(stop_type))

    def exarobot_get_pos(self):
        self.send_cmd_exarobot("POS")

    def on_exarobot_queue_bcast(self, msg: dict):
        try:
            if "COMM_STATUS" in msg.keys():
                comm_status = msg['COMM_STATUS']
                self.sig_exarobot_comm_status.emit(comm_status)

            elif "ENCODER" in msg.keys():
                right, left = msg["ENCODER"]
                self.sig_exarobot_get_enc.emit(int(right), int(left))

            elif "VELOCITY" in msg.keys():
                right, left = msg["VELOCITY"]
                self.sig_exarobot_get_vel.emit(float(right), float(left))

            elif "WHEEL_POSITION" in msg.keys():
                right, left = msg["WHEEL_POSITION"]
                self.sig_exarobot_get_wheel_pos.emit(float(right), float(left))

            elif "STATUS" in msg.keys():
                state = msg["STATUS"]
                self.sig_exarobot_get_state.emit(int(state))

            elif "POSITION" in msg.keys():
                x, y, theta = msg["POSITION"]
                # print('ControlCore', x,y,theta)

                if is_ros_installed():
                    self._current_pos_x = x
                    self._current_pos_y = y
                    self._current_pos_theta = theta
                    self.ros_robot_node_update_odom(x, y, theta, 0., 0.)
            else:
                pass

        except:
            pass

    ### STELLA B2 ####
    def send_cmd_stellab2(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        self._mpStellaB2.send_command(dictCmd)

    def stellab2_create(self):
        self.send_cmd_stellab2('CREATE', 'STELLAB2')

    def stellab2_release(self):
        self.send_cmd_stellab2('RELEASE')

    def stellab2_connect(self, strComPort: str):
        self.send_cmd_stellab2('CONNECT', strComPort)

    def stellab2_disconnect(self):
        self.send_cmd_stellab2('DISCONNECT')

    def stellab2_set_monitoring(self, state: bool):
        self.send_cmd_stellab2('MONITORING', bool(state))

    def stellab2_reset_motor(self):
        self.send_cmd_stellab2('RESET_MOTOR')

    def stellab2_factory_reset(self):
        self.send_cmd_stellab2('INIT_MOTOR')

    # todo: make this extendable to other robots
    def stellab2_move_joint_space(self, right_vel: int, left_vel: int):
        self.send_cmd_stellab2("MOVE_JOINT_SPACE", (right_vel, left_vel))

    def stellab2_set_velocity_control(self, vel: float, ang_vel: float):
        self.send_cmd_stellab2("SET_VELOCITY_CONTROL", (vel, ang_vel))

    def stellab2_stop_robot(self, stop_type: int):
        self.send_cmd_stellab2("STOP", int(stop_type))

    def on_stellab2_queue_bcast(self, msg: dict):
        try:
            if "VERSION" in msg.keys():
                version = msg['VERSION']
                self.sig_stellab2_get_ver.emit(version)

            elif "COMM_STATUS" in msg.keys():
                comm_status = msg['COMM_STATUS']
                self.sig_stellab2_comm_status.emit(comm_status)

            elif "MONITORING" in msg.keys():
                monitoring_state = msg["MONITORING"]
                self.sig_stellab2_is_monitoring.emit(monitoring_state)

            elif "ENCODER" in msg.keys():
                right, left = msg["ENCODER"]
                self.sig_stellab2_get_enc.emit(int(right), int(left))

            elif "VELOCITY" in msg.keys():
                right, left = msg["VELOCITY"]
                self.sig_stellab2_get_vel.emit(float(right), float(left))

            elif "WHEEL_POSITION" in msg.keys():
                right, left = msg["WHEEL_POSITION"]
                self.sig_stellab2_get_wheel_pos.emit(float(right), float(left))

            elif "STATUS" in msg.keys():
                state = msg["STATUS"]
                self.sig_stellab2_get_state.emit(int(state))

            else:
                pass

        except:
            pass

    def on_mp_error(self, msg: dict):
        pass

    ### External Communication ####
    def send_external_comm(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        self._mpExternalComm.send_command(dictCmd)

    def external_comm_create(self):
        self.send_external_comm('CREATE')

    def external_comm_start(self, bSckType: str, strAddr: str, nPort: int):
        # todo: add UDP later. socket will be determined by bScktype
        self.send_external_comm('START', (strAddr, nPort))

    def external_comm_stop(self):
        self.send_external_comm('STOP')

    def external_comm_send_raw(self, strMsg: str):
        self.send_external_comm('SEND_RAW', strMsg)

    def external_comm_disconnect_client(self, tplAddrPort: tuple):
        self.send_external_comm('REMOVE_CLIENT', tplAddrPort)

    def external_comm_release(self):
        self.send_external_comm('RELEASE')

    def on_external_comm_queue_bcast(self, msg: dict):
        try:
            if "SERVER_STATUS" in msg.keys():
                server_status = msg["SERVER_STATUS"]
                self.sig_external_comm_server_status.emit(server_status)

            elif "CLIENT_CONNECT" in msg.keys():
                tplAddrPort = msg["CLIENT_CONNECT"]
                self.sig_external_comm_client_connected.emit(tplAddrPort)
                self.tts_send_msg('connect OK')

            elif "CLIENT_DISCONNECT" in msg.keys():
                tplAddrPort = msg["CLIENT_DISCONNECT"]
                self.sig_external_comm_client_disconnected.emit(tplAddrPort)

            elif "RECV_RAW" in msg.keys():
                strMsg = msg["RECV_RAW"]
                print('RECV_RAW', strMsg)

            elif "EMG_STOP" in msg.keys():
                print("EMG STOP")
                self.tts_send_msg('Stop Stop')
                self.exarobot_set_velocity_control(0, 0)
                self.nav.cancel_nav()


            elif "MOVE_POSE" in msg.keys():
                xPos, yPos, aAngle = msg["MOVE_POSE"]
                print('MOVE_POSE', xPos, yPos, aAngle)
                # tts_msg = 'Move Pose %s %s'%(xPos, yPos)
                tts_msg = 'Move Pose'
                self.tts_send_msg(tts_msg)

                pose = self.nav.make_pose([xPos, yPos])
                self.nav.go_to_pose(pose)

            else:
                pass

        except:
            pass

    ### TTS ###
    def tts_send_data(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        self._mpTextToSpeech.send_command(dictCmd)

    def tts_create(self):
        self.tts_send_data('CREATE')

    def tts_send_msg(self, strMsg: str):
        self.tts_send_data('SEND_MSG', strMsg)

    ### STT ##
    def start_stt_listen(self, strMic: str):
        self._sttHandler.start_listen(strMic)

    ### Camera ###
    def onCamStart(self, nId: int, nWidth: int, nHeight: int):
        if self._camTest is None:
            self._camTest = CCameraCommon(nId, nWidth, nHeight)
            self._camTest.sig_image_data.connect(self.on_recv_image)
            self._camTest.sig_stopped.connect(self.on_stopped_cam)
            self._camTest.start_stream()

    def stopCam(self):
        if self._camTest is not None:
            self._camTest.stop_stream()

    def on_recv_image(self, data: tuple):
        self.sig_cam_recv_stream.emit(data)

    def on_stopped_cam(self):
        if self._camTest is not None:
            del self._camTest
            self._camTest = None

        self.sig_cam_stopped_stream.emit()

    # ROS Robot Node
    def send_cmd_ros_robot_node(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSRobotNode.send_command(dictCmd)

    def ros_robot_node_create(self, fPeriod: float, strTopicNameOdom: str, strTopicNameVel: str):
        createParam = (fPeriod, strTopicNameOdom, strTopicNameVel)
        self.send_cmd_ros_robot_node("CREATE", createParam)

    def ros_robot_node_start_timer(self):
        self.send_cmd_ros_robot_node("START_TIMER")

    def ros_robot_node_stop_timer(self):
        self.send_cmd_ros_robot_node("STOP_TIMER")

    def ros_robot_node_release(self):
        self.send_cmd_ros_robot_node("RELEASE")

    def ros_robot_node_update_odom(self, fPos_X, fPos_Y, fPos_Theta, fVel_c, fOmega_c):
        odom = (fPos_X, fPos_Y, fPos_Theta, fVel_c, fOmega_c)
        self.send_cmd_ros_robot_node("UPDATE_ODOM", odom)

    def on_ros_robot_node_vel_sub(self, msg: dict):
        if "VEL_CMD" in msg.keys():
            # print(msg)
            vel, ang_vel = msg["VEL_CMD"]
            self.exarobot_set_velocity_control(float(vel), float(ang_vel))
            # self.kobuki2_set_velocity_control(float(vel), float(ang_vel))

        else:
            pass

    # ROS Nav Node

    def on_nav_queue_bcast(self, msg: dict):
        try:
            if "NODE_LIST" in msg.keys():
                # print('RCV !!:',msg)
                node_list = msg['NODE_LIST']
                for name in node_list:
                    print('# Add MP Monitoring - NAME[{}]: PID[{}]'.format(name, node_list[name]))
                    self._dictProc[name] = node_list[name]
            else:
                pass

        except:
            pass

        self.get_proc_info()

    def send_cmd_ros_nav_node(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSNavigation.send_command(dictCmd)

    def ros_nav_node_create(self, mode: str):
        val = mode
        self.send_cmd_ros_nav_node("CREATE", val)

    def ros_nav_node_request_pid_list(self):
        self.send_cmd_ros_nav_node("NODE_LIST")

    def ros_nav_node_launch_nav_stack(self):
        self.send_cmd_ros_nav_node("LAUNCH")

    def ros_nav_node_move_pose(self, xPos, yPos, angle):
        val = xPos, yPos, angle
        self.send_cmd_ros_nav_node("MOVE_POSE", val)

    def ros_nav_node_move_cancle(self):
        self.send_cmd_ros_nav_node("MOVE_CANCLE")

    def ros_nav_node_release(self):
        self.send_cmd_ros_nav_node("RELEASE")

    # ROS RS Node
    def on_ros_rs_node_queue_bcast(self, msg: dict):
        try:
            if "NODE_LIST" in msg.keys():
                # print('RCV !!:',msg)
                node_list = msg['NODE_LIST']
                for name in node_list:
                    print('# Add MP Monitoring - NAME[{}]: PID[{}]'.format(name, node_list[name]))
                    self._dictProc[name] = node_list[name]
            else:
                pass

        except:
            pass

        self.get_proc_info()

    def send_cmd_ros_face_recog(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSFaceRecog.send_command(dictCmd)

    def on_ros_facerecog_camframe(self, cam_frames: tuple):
        self.sig_facerecog_stream.emit(cam_frames)

    def on_ros_facerecog_pos(self, msg: dict):
        try:
            if "POS_XYZ" in msg.keys():
                x, y, z, rw, rx, ry, rz = msg["POS_XYZ"]
                self.sig_facerecog_xyz.emit((x, y, z))
                self._face_pos_x = x
                self._face_pos_y = y
                self._face_pos_z = z
                self._face_rotation_w = rw
                self._face_rotation_x = rx
                self._face_rotation_y = ry
                self._face_rotation_z = rz
                # print('get XYZ: ', msg)

            elif "DIST" in msg.keys():
                distance = msg["DIST"]
                self._face_distance = distance
                # print("Get DIST:", self._face_distance)
                #todo: match coordinates with the robot and map. Robot position should also be calculated (e.g., camera offset, add map, etc)
                if is_ros_installed():
                    # todo: this can be adjusted
                    if self._face_distance > 0.1:
                        # todo: current position should be from the global map
                        pose = self.nav.make_pose([self._face_pos_z, self._face_pos_x], 
                        [self._face_rotation_x, self._face_rotation_y, self._face_rotation_z, self._face_rotation_w])

                        # print('face pose : ',self._face_pos_z, self._current_pos_x, self._face_pos_x, self._current_pos_y)
                        self.nav.clearAllCostmaps()
                        # time.sleep(0.5)
                        self.nav.go_to_pose(pose)
                        tts_msg = 'face detect'
                        self.tts_send_msg(tts_msg)
                       
                        print(self._face_pos_z, self._face_pos_x, self._face_pos_y )

                self.sig_facerecog_dist.emit(distance)
            else:
                pass
        except:
            pass

    def ros_face_recog_create(self):
        print('ros_face_recog_create')
        self.send_cmd_ros_face_recog("CREATE")
        tts_msg = 'face recog create'
        self.tts_send_msg(tts_msg)

    def ros_face_recog_release(self):
        self.send_cmd_ros_face_recog("RELEASE")

    def send_cmd_ros_rs_node(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSRealsense.send_command(dictCmd)

    def ros_rs_node_request_pid_list(self):
        self.send_cmd_ros_rs_node("NODE_LIST")

    def ros_rs_node_create(self):
        self.send_cmd_ros_rs_node("CREATE")

    def ros_rs_node_launch(self):
        self.send_cmd_ros_rs_node("LAUNCH")

    def ros_rs_node_release(self):
        self.send_cmd_ros_rs_node("RELEASE")

    ### ROS Global Path Planner Simulation ####
    # def startPathPlanningSim(self, planner_type: str, start_pos: Tuple[int, int], goal_pos: Tuple[int, int]):
    #     type = planner_type
    #
    #     if type == "AStar":
    #         my_planner = AStar(start_pos, goal_pos, "euclidean")
    #         planner_name = "A*"
    #
    #     elif type == "Dijkstra":
    #         my_planner = Dijkstra(start_pos, goal_pos, 'None')
    #         planner_name = "Dijkstra's"
    #
    #     plannerDet = (my_planner, planner_name, start_pos, goal_pos)
    #     self.queuePlotter.put(plannerDet)

    # def StartThreadPlanner(self):
    #     if self.threadingPlotter is None:
    #         self.queuePlotter = queue.Queue()
    #         self.threadingPlotter = ThreadPlotter(self.queuePlotter)
    #         self.threadingPlotter.sig_terminated.connect(self.OnThreadPlannerTerminated)
    #         self.threadingPlotter.start()
    #
    # def StopThreadPlanner(self):
    #     if self.threadingPlotter is not None:
    #         self.threadingPlotter.stop()
    #
    # def OnThreadPlannerTerminated(self):
    #     if self.threadingPlotter is not None:
    #         del self.threadingPlotter
    #         self.threadingPlotter = None


if __name__ == '__main__':
    print('Creat Contorol Core')
    core = EmbdControlCore()
    core.exarobot_connect('/dev/ttyRobot')
    # core.kobuki2_connect('/dev/ttyUSB0')
    # core.external_comm_start('x', strAddr='0.0.0.0', nPort=7506)
    while 1:
        # t = input()

        time.sleep(1)
