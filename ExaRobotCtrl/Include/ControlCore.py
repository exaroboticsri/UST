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
from cmath import tan
from tf_transformations import quaternion_from_euler, euler_from_quaternion

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
import MultiProcessing.MPMobileRobot as MPMobileRobot
import MultiProcessing.MPExternalComm as MPExternalComm
from Library.Devices.Camera.CameraCommon import CCameraCommon


if is_ros_installed():
    import MultiProcessing.MPROSVelPub as MPROSVelPub
    import MultiProcessing.MPROSRobot as MPROSRobot
    import MultiProcessing.MPROSNavigation as MPROSNavigation
    import MultiProcessing.MPROSRealsense as MPROSRealsense
    from ROSIntegration.ROSNavigator import CROSNavigator


class EmbdControlCore(object):
    """ 
    메인 컨트롤러로서 로봇제어 및 서비스 기능 구현을 위한 모듈을 제어 할 수 있도록 인터페이스 메소드를 제공한다.
    
    MultiProcess 모듈을 상속받아 각 기능별 프로세스와 Queue,Pipe,Signal 등을 사용하여 제어할 수 있도록 구현됨

    
    """
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
    # Etc
    _mpCamTest: CMultiProcessBase = None
    _mpExternalComm: CMultiProcessBase = None

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
 
        # Navigator
        self.navigator = CROSNavigator()
        self.prev_x = 0.
        self.prev_y = 0.

        self.dx = 0.
        self.dy = 0.
        self.server_status = False
    def release(self):
        """
        생성된 모든 멀티 프로세서를 종료 한다.
        """
        self.stop_multi_processes()
        write_log("Release Control Core", self)
        # self.StopThreadPlanner()

    def init_multi_processes(self):
        """
        멀티 프로세스를 생성하고 초기화한다.
        
        생성되는 프로세스
        
        proc_mobile_robot (Stella)
        
        proc_mobile_robot (Kobuki2)
        
        proc_mobile_robot (ExaRobot)
        
        proc_external_comm
        
        proc_text_to_speech
        
        proc_ros_robot_node
        
        proc_ros_nav_node
        
        proc_ros_rs_node
                
        """
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

        except:
            pass

    def get_proc_info(self):
        """
        모니터링 프로세스에 관찰할 새로운 프로세스를 추가한다. 
        """
        self.sig_proc_monitor_pid.emit(self._dictProc)

    def stop_multi_processes(self):
        """
        생성된 모든 멀티 프로세스를 종료한다.
    
        """
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
        """
        로봇제어 모듈 생성을 요청한다.
        """
        self.send_cmd_exarobot('CREATE', 'EXAROBOT')

    def exarobot_release(self):
        """
        로봇제어 모듈 종료 요청한다.
        """
        self.send_cmd_exarobot('RELEASE')

    def exarobot_connect(self, strComPort: str):
        """
        로봇의 시리얼 장치연결을 요청하고 초기화를 한다.
        
        로봇 프로세스 초기화
        
        Args:
            strComPort (str): 로봇이 연결된 시리얼 장치
             
        """
        print('exarobot_connecting')
        self.send_cmd_exarobot('CONNECT', strComPort)
        self.do_when_robot_connected()
        
# 
    def exarobot_disconnect(self):
        # self.nav.release()
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
        """
        로봇 속도 제어 명령을 보낸다.

        Args:
            vel (float): 선속도 [m/s]
            ang_vel (float): 각속도 [radian]
        """
        self.send_cmd_exarobot("SET_VELOCITY_CONTROL", (vel, ang_vel))

    def exarobot_stop_robot(self, stop_type: int):
        self.send_cmd_exarobot("STOP", int(stop_type))

    def exarobot_get_pos(self):
        """
        로봇 포지션 요청 명령
        
        모터 엔코더기반 포지션을 요청한다.
        """
        self.send_cmd_exarobot("POS")

    def on_exarobot_queue_bcast(self, msg: dict):
        """
        exarobot 프로세스로부터 받는 Queue msg수신시 동작하는  Callback이다.

        Args:
            msg (dict): Queue msg
        """
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

    def external_comm_send_tts(self, strMsg: str):
        self.send_external_comm('SEND_TTS', strMsg)
    
    def external_comm_release(self):
        self.send_external_comm('RELEASE')

    def is_server_status_alive(self) -> bool: 
        return self.server_status
    
    def on_external_comm_queue_bcast(self, msg: dict):
        """
        external_comm 프로세스로부터 받는 Queue msg수신시 동작하는  Callback이다.

        Args:
            msg (dict): Queue msg
        """
        try:
            if "SERVER_STATUS" in msg.keys():
                server_status = msg["SERVER_STATUS"]
                
                if server_status == 'START':
                    self.server_status = True
                else:
                    self.server_status = False
                print('SERVER_STATUS: ',server_status)
                self.sig_external_comm_server_status.emit(server_status)

            elif "CLIENT_CONNECT" in msg.keys():
                tplAddrPort = msg["CLIENT_CONNECT"]
                print('CLIENT_CONNECT :', tplAddrPort)
                self.sig_external_comm_client_connected.emit(tplAddrPort)
                self.tts_send_msg('connect OK')

            elif "CLIENT_DISCONNECT" in msg.keys():
                tplAddrPort = msg["CLIENT_DISCONNECT"]
                print('CLIENT_DISCONNECT :', tplAddrPort)
                self.sig_external_comm_client_disconnected.emit(tplAddrPort)

            elif "RECV_RAW" in msg.keys():
                strMsg = msg["RECV_RAW"]
                print('RECV_RAW', strMsg)

            elif "EMG_STOP" in msg.keys():
                print("EMG STOP")
                self.tts_send_msg('Stop')
                self.exarobot_set_velocity_control(0, 0)
                self.navigator.cancel_nav()


            elif "MOVE_POSE" in msg.keys():
                xPos, yPos, aAngle = msg["MOVE_POSE"]
                
                # tts_msg = 'Move Pose %s %s'%(xPos, yPos)
                if self.navigator.isTaskComplete():
                    print('MOVE_POSE', xPos, yPos, aAngle)
                    tts_msg = 'Move Pose'
                    self.tts_send_msg(tts_msg)
                    q = quaternion_from_euler(0., 0., aAngle)
                    pose = self.navigator.make_pose([xPos, yPos], [q[0],q[1],q[2],q[3]])
                    self.navigator.go_to_pose(pose)
                else:
                    pass

            else:
                pass

        except:
            pass

    def tts_send_msg(self, strMsg: str):
        if self.is_server_status_alive() == True:
            self.external_comm_send_tts(strMsg)

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
        """
        ROS 로봇 프로세스로 제어 명령을 보낸다.

        Args:
            acmd (str): 제어 명령
            aparams (object, optional): 제어 값. Defaults to None.
        """
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSRobotNode.send_command(dictCmd)

    def ros_robot_node_create(self, fPeriod: float, strTopicNameOdom: str, strTopicNameVel: str):
        """
        ROS 로봇 노드 생성 명령을 보낸다.

        Args:
            fPeriod (float): odom 토픽 발행 주기
            strTopicNameOdom (str): 발행할 odometry 토픽 이름
            strTopicNameVel (str): 구독할 속도 제어 토픽 이름 
        """
        createParam = (fPeriod, strTopicNameOdom, strTopicNameVel)
        self.send_cmd_ros_robot_node("CREATE", createParam)

    def ros_robot_node_start_timer(self):
        """
        ROS 로봇 노드의 odometry 토픽 발행을 시작한다.
        """
        self.send_cmd_ros_robot_node("START_TIMER")

    def ros_robot_node_stop_timer(self):
        """
        ROS 로봇 노드의 odometry 토픽 발행을 종료한다.
        """
        self.send_cmd_ros_robot_node("STOP_TIMER")

    def ros_robot_node_release(self):
        self.send_cmd_ros_robot_node("RELEASE")

    def ros_robot_node_update_odom(self, fPos_X, fPos_Y, fPos_Theta, fVel_c, fOmega_c):
        """
        ROS 로봇 노드 Odometry 데이터를 업데이트 한다.

        Args:
            fPos_X (float): 로봇 X 포지션
            fPos_Y (float): 로봇 Y 포지션
            fPos_Theta (float): 로봇 회전각도
            fVel_c (float): 로봇 속도
            fOmega_c (float): 로봇 회전속도
        """
        odom = (fPos_X, fPos_Y, fPos_Theta, fVel_c, fOmega_c)
        self.send_cmd_ros_robot_node("UPDATE_ODOM", odom)

    def on_ros_robot_node_vel_sub(self, msg: dict):
        """
        ROS 로봇 노드로부터 속도 제어 명령을 수신시 동작하는 Callback 함수이다

        Args:
            msg (dict): 속도 제어 메시지
        """
        if "VEL_CMD" in msg.keys():
            # print(msg)
            vel, ang_vel = msg["VEL_CMD"]
            self.exarobot_set_velocity_control(float(vel), float(ang_vel))
            # self.kobuki2_set_velocity_control(float(vel), float(ang_vel))

        else:
            pass

    # ROS Nav Node

    def on_nav_queue_bcast(self, msg: dict):
        """
        ROS Navagation2 프로세스로부터 받는 Queue msg수신시 동작하는  Callback이다.

        Args:
            msg (dict): Queue msg
        """
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

    def ros_nav_node_create(self):
        """
        ROS Navagation2 모듈 오브젝트 생성 명령을 보낸다.

        """
        self.send_cmd_ros_nav_node("CREATE")

    def ros_nav_node_request_pid_list(self):
        """
        ROS Navagation2 Launch로 생성된 프로세스의 PID 를 요청한다.
        """
        self.send_cmd_ros_nav_node("NODE_LIST")

    def ros_nav_node_launch_lidar(self):
        self.send_cmd_ros_nav_node("LAUNCH_LIDAR")

    def ros_nav_node_launch_cartograhper(self):
        self.send_cmd_ros_nav_node("LAUNCH_CARTO")

    def ros_nav_node_launch_nav(self):
        self.send_cmd_ros_nav_node("LAUNCH_NAV")

    def ros_nav_node_move_pose(self, xPos, yPos, angle):
        """
        지정위치로 이동할수 있도록 Navigation 명령을 보낸다.

        Args:
            xPos (float): X 포지션 [m]
            yPos (float): Y 포지션 [m]
            angle (float): heading angle [radian]
        """
        val = xPos, yPos, angle
        self.send_cmd_ros_nav_node("MOVE_POSE", val)

    def ros_nav_node_move_cancle(self):
        """
        현재 Navagation 중인 명령을 취소한다.
        """
        self.send_cmd_ros_nav_node("MOVE_CANCLE")

    def ros_nav_node_release(self):
        """
        rs_node 프로세스로부터 받는 Queue msg수신시 동작하는  Callback이다.

        Args:
            msg (dict): Queue msg
        """
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
            
    def do_when_robot_connected(self):
        print('Robot Node Create')
        
        
        # os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        # os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/opt/dds_config/fastrtps.xml'
        
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        self.ros_robot_node_create(0.02, 'odom', 'cmd_vel')
        print('Robot Node Timer Start (/tf, /odom publish)')
        self.ros_robot_node_start_timer()

        
        
        print('Robot Navigation Stack Launching')
        self.ros_nav_node_create()
        self.ros_nav_node_launch_lidar()
        self.ros_nav_node_launch_cartograhper()


        # self.ros_face_recog_create()

        #self.tts_send_msg('Localization start')
        self.exarobot_set_velocity_control(0. , 0.7)
        #self.tts_send_msg('wait 7 sec')
        t = time.time()
        time.sleep(15)
        self.exarobot_set_velocity_control(0. , -0.7)
        #self.tts_send_msg('wait 7 sec')
        time.sleep(15)
        print('##### spin time : ', time.time()-t)
        self.exarobot_set_velocity_control(0. , 0.)
        
        # os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        self.ros_nav_node_launch_nav()
        self.navigator.wait_nav2_active()


        # os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        print('Robot Realsense Node Launching')
        self.ros_rs_node_create()
        self.ros_rs_node_launch()

        # #self.tts_send_msg('Navigation is On')

        # #self.tts_send_msg("Hellow  I am Ready To GO")
        
        # time.sleep(2)
        # self.navigator.clear_all_costmap()
        # self.navigator.clear_all_costmap()
        
        # self.ros_nav_node_request_pid_list()
        # self.ros_rs_node_request_pid_list()

if __name__ == '__main__':
    try :
        print('Creat Contorol Core')
        core = EmbdControlCore()
        core.exarobot_connect('/dev/ttyRobot')
        # core.kobuki2_connect('/dev/ttyUSB0')
        core.external_comm_start('x', strAddr='0.0.0.0', nPort=7506)
        while 1:
            # t = input()

            time.sleep(5)
            core.tts_send_msg('test')
    except KeyboardInterrupt:
        os._exit(1)
