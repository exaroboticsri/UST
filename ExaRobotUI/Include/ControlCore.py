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
import sys
import time
import multiprocessing as mp
from tf_transformations import euler_from_quaternion
import math
from queue import Queue
FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = os.path.dirname(FILE_PATH)

INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
EXTERNAL_COMM_PATH = os.path.join(INCLUDE_PATH, "ExternalComm")
MULTIPROC_PATH = os.path.join(INCLUDE_PATH, "MultiProcessing")
TTS_PATH = os.path.join(INCLUDE_PATH, 'AudioRecognition')

SOCKET_PATH = os.path.join(LIBRARY_PATH, "Socket")
sys.path.extend(
    [FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH, LIBRARY_PATH, EXTERNAL_COMM_PATH,MULTIPROC_PATH,TTS_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH, LIBRARY_PATH, EXTERNAL_COMM_PATH

# from SocketCommon import EmbdTcpClient
from ExternalComm.ExternalCommClient import CExternalCommClient
from Commons import *
from MultiProcessing.MultiProcessBase import CMultiProcessBase
import MultiProcessing.MPTextToSpeech as MPTextToSpeech
import MultiProcessing.MPSpeechToText as MPSpeechToText

if is_ros_installed():
    import MultiProcessing.MPROSRealsense as MPROSRealsense
    # import MultiProcessing.MPROSRealsense as MPROSRealsense
    import MultiProcessing.MPROSFaceRecog as MPROSFaceRecog

    
class EmbdControlCore(object):
    # queuePlotter: queue.Queue = None
    # threadingPlotter: ThreadPlotter = None
    _external_comm: CExternalCommClient = None
    _mpROSRealsense: CMultiProcessBase = None
    _mpROSFaceRecog: CMultiProcessBase = None
    _mpTextToSpeech: CMultiProcessBase = None
    _mpExternalComm: CMultiProcessBase = None

    def __init__(self):
        super(EmbdControlCore, self).__init__()
        self._external_comm = CExternalCommClient()
        self._external_comm.sig_start.connect(self.on_client_connect)
        self._external_comm.sig_stop.connect(self.on_client_disconnect)
        self._external_comm.sig_error.connect(self.on_client_error)
        self._external_comm.sig_tts_msg.connect(lambda msg: self.on_client_get_tts(msg))
        self.sig_tcp_connected = PySignal()
        self.sig_tcp_disconnected = PySignal()
        self.sig_tcp_error = PySignal()
        
        self.sig_external_comm_server_status = PySignal(str)
        self.sig_external_comm_client_connected = PySignal(tuple)
        self.sig_external_comm_client_disconnected = PySignal(tuple)
        #ros face recog
        self.sig_facerecog_stream = PySignal(tuple)
        self.sig_facerecog_dist = PySignal(float)
        self.sig_facerecog_xyz = PySignal(tuple)
        #stt
        self.sig_stt_msg = PySignal(str)
        self._face_follow_mode= False
        self._face_target_name = ''
        self.stt_target= ''
        self._dictProc = dict()
        self.init_multi_processes()
        
        self.listTablePos = [(-2.85, -3.42, 1.0), (-16., 2.3, 0.0), ]
        
        self.ros_face_recog_create()
        time.sleep(1)
        print('rs_node_create')
        # self.ros_rs_node_create()
        # self.ros_rs_node_start()
        # self.ros_rs_node_launch()        
       
        self.tts_create()
        
        self.tts_send_msg('Application is Ready!')        
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
            
            self._mpTextToSpeech = CMultiProcessBase('TextToSpeech', MPTextToSpeech.proc_text_to_speech, False, False)
            self._mpTextToSpeech.sig_error.connect(self.on_mp_error)
            self._mpTextToSpeech.start()
            self._dictProc[self._mpTextToSpeech.NAME] = self._mpTextToSpeech.PID
            
            self._mpSpeechToText = CMultiProcessBase('SpeechToText', MPSpeechToText.proc_speech_to_text, False, False)
            self._mpSpeechToText.sig_queue_bcast.connect(self.on_speech_to_text_queue_bcast)
            self._mpSpeechToText.sig_error.connect(self.on_mp_error)
            self._mpSpeechToText.start()
            self._dictProc[self._mpSpeechToText.NAME] = self._mpSpeechToText.PID
            
            self.stt_create()
            

            if is_ros_installed():
                # self._mpROSRealsense = CMultiProcessBase('ROS Robot Realsense', MPROSRealsense.proc_ros_rs_node, True, False)
                # self._mpROSRealsense.sig_error.connect(self.on_mp_error)
                # self._mpROSRealsense.sig_queue_bcast.connect(self.on_ros_rs_node_queue_bcast)
                # self._mpROSRealsense.sig_queue_bcast_bk.connect(self.on_ros_rs_node_frame)
                # self._mpROSRealsense.start()
                # self._dictProc[self._mpROSRealsense.NAME] = self._mpROSRealsense.PID

                self._mpROSFaceRecog = CMultiProcessBase('ROS Face Recognition', MPROSFaceRecog.proc_ros_face_recog,
                                                         True, False)
                self._mpROSFaceRecog.sig_error.connect(self.on_mp_error)
                self._mpROSFaceRecog.sig_queue_bcast.connect(self.on_ros_facerecog_pos)
                self._mpROSFaceRecog.sig_queue_bcast_bk.connect(self.on_ros_facerecog_camframe)
                self._mpROSFaceRecog.start()
                self._dictProc[self._mpROSFaceRecog.NAME] = self._mpROSFaceRecog.PID

        except:
            
            pass
        

    def set_emg_stop(self):
        self._external_comm.send_emg_stop()

    def move_to_target(self, tableNumber: int):
        if tableNumber <= 1:
            xPos, yPos, aAngle = self.listTablePos[tableNumber]
            self.send_coordinates(xPos, yPos, aAngle)
    def send_raw_data(self, abStr: str):
        self._external_comm.send_raw(abStr)

    def send_coordinates(self, xPos: float, yPos: float, angleDeg: float):
        self._external_comm.send_pose(xPos, yPos, angleDeg)

    NO_CMD = 0x00
    MOVE_ROS_POS = 0x01
    EMG_STOP = 0x02
    MOVE_MOTOR_CENTER_VEL = 0x03
    MOVE_MOTOR_JOINT_VEL = 0x04
    GET_CURR_POS = 0x05

    def stop_client(self):
        self._external_comm.stop_client()

    def start_client(self, strAddress: str):
        if self._external_comm.start_client(7506, strAddress):
            self.sig_tcp_connected.emit()
            tts_msg = 'server connect success'
            self.tts_send_msg(tts_msg)
        else:
            self.sig_tcp_error.emit()

    def on_client_connect(self):
        self.sig_tcp_connected.emit()

    def on_client_disconnect(self):
        self.sig_tcp_disconnected.emit()

    def on_client_error(self):
        self.sig_tcp_error.emit()

    def on_client_get_tts(self, msg: str):
        tts_msg = msg
        # print('GET_TTS_CMD', tts_msg)
        # self.tts_send_msg(tts_msg)
                
          
    
    def release(self):
        self.stop_client()


    # ROS RS Node
    def on_ros_rs_node_frame(self, msg: dict):
        try:
            if "FRAME" in msg.keys():
                data = msg['FRAME']
                color, depth, param, fps = data
                # print("recv frame")

                self.send_cmd_ros_face_recog('FRAME',(color, depth, param, fps))
                # self.send_cmd_ros_face_recog('FRAME',data)
            else:
                pass

        except:
            pass
        
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

    def ros_rs_node_start(self):
        self.send_cmd_ros_rs_node("START")
        
    def ros_rs_node_release(self):
        self.send_cmd_ros_rs_node("RELEASE")

    
    def send_cmd_ros_face_recog(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSFaceRecog.send_command(dictCmd)
            
    #Face recognition
    def on_ros_facerecog_camframe(self, cam_frames):
        # print('on_ros_facerecog_camframe')
        self.sig_facerecog_stream.emit(cam_frames)

    def on_ros_facerecog_pos(self, msg: dict):
        try:
            if "POS_XYZ" in msg.keys():
                x, y, z, rw, rx, ry, rz = msg["POS_XYZ"]
                # self.sig_facerecog_xyz.emit((x, y, z))

                self._face_pos_x = x
                self._face_pos_y = y
                self._face_pos_z = z
                self._face_rotation_w = rw
                self._face_rotation_x = rx
                self._face_rotation_y = ry
                self._face_rotation_z = rz
                print('Control core recv data pos _xyz')
                # print('get XYZ: ', msg,self._face_target_name,"follow name :",self.stt_target,"mode :",self._face_follow_mode,"pose_x")
                # print("Get dis:", self._face_distance,"target name :",self._face_target_name,"follow name :",self.stt_target,"mode :",self._face_follow_mode,"pose_x",self._face_pos_x,"pose_x",self._face_pos_x)

            elif "DIST" in msg.keys():
                distance = msg["DIST"]
                self._face_distance = distance
                # print("Get dis:", self._face_distance,"target name :",self._face_target_name,"follow name :",self.stt_target,"mode :",self._face_follow_mode,"pose_x",self._face_pos_x,"pose_x",self._face_pos_x)
                #todo: match coordinates with the robot and map. Robot position should also be calculated (e.g., camera offset, add map, etc)
                if is_ros_installed():
                    # todo: this can be adjusted
                    
                    # todo: current position should be from the global map
                    # pose = self.navigator.make_pose([self._face_pos_x, self._face_pos_y], [self._face_rotation_x, self._face_rotation_y, self._face_rotation_z, self._face_rotation_w])
                    r , p, yaw= euler_from_quaternion((self._face_rotation_w, self._face_rotation_x, self._face_rotation_y, self._face_rotation_z))
                    print("Get dis:", self._face_distance,"target name :",self._face_target_name,"follow name :",self.stt_target,"mode :",self._face_follow_mode,"pose_x",self._face_pos_x,"pose_x",self._face_pos_x,"yaw :",yaw)

                    self._external_comm.send_pose(self._face_pos_x,self._face_pos_y, yaw)
                    # tts_msg = 'detect'
                    # self.tts_send_msg(tts_msg)
                    # print(self._face_pos_x, self._face_pos_y, yaw *180 / math.pi)
                    # self.sig_facerecog_dist.emit(distance)

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
        
    #TTS
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
        
    def on_mp_error(self, msg: dict):
        pass
    
    #STT
    def send_cmd_stt(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        
        self._mpSpeechToText.send_command(dictCmd)
    
    def stt_create(self):
        self.send_cmd_stt('CREATE')
    def stt_listen(self):
        self.send_cmd_stt('LISTEN')
    def on_speech_to_text_queue_bcast(self, msg: dict):
            """
            speech_to_text 프로세스로부터 받는 Queue msg수신시 동작하는  Callback이다.

            Args:
                msg (dict): Queue msg
            """
            try:
                if "STT_MSG" in msg.keys():
                    stt_msg= msg["STT_MSG"]
                    print('STT_MSG: ', stt_msg)
                    if '조용환' in stt_msg or '조용한' in stt_msg:
                        tts_msg= 'yong hwan follow'
                        self.tts_send_msg(tts_msg)
                        self.send_cmd_ros_face_recog('TARGET','yh')
                        self.stt_target = 'yh'
                        
                    elif '조세연' in stt_msg or '조세현' in stt_msg:
                        tts_msg='se yeon follow'
                        self.tts_send_msg(tts_msg)
                        self.send_cmd_ros_face_recog('TARGET', 'jose')
                        self.stt_target = 'jose'
                        

            except:
                pass
            
    def follow_start(self) :
        tts_msg = 'follow mode on, who should follow '
        self.tts_send_msg(tts_msg)
        time.sleep(2)
        self.stt_listen()
        mode = True
        self._face_follow_mode= mode
        # self.send_cmd_ros_face_recog("TARGET",'yh')
        self.send_cmd_ros_face_recog("FOLLOW_MODE",self._face_follow_mode)
        print('self._face_follow_mode',self._face_follow_mode)

        
        
        
    def follow_stop(self) :
        tts_msg = 'follow mode off'
        self.tts_send_msg(tts_msg)
        mode = False
        self._face_follow_mode= mode
        self.send_cmd_ros_face_recog("TARGET",'None')
        self.send_cmd_ros_face_recog("FOLLOW_MODE",self._face_follow_mode)
        # self.send_cmd_ros_face_recog("TARGET", 'None')
        self.stt_target=''
        print('self._face_follow_mode',self._face_follow_mode, self.stt_target)
        
        
    #External comm
    # def send_external_comm(self, acmd: str, aparams: object = None):
    #     aCmd = str(acmd)
    #     dictCmd = dict()
    #     dictCmd["CMD"] = aCmd
    #     dictCmd["VAL"] = aparams
    #     self._mpExternalComm.send_command(dictCmd)

    # def external_comm_create(self):
    #     self.send_external_comm('CREATE')

    # def external_comm_start(self, bSckType: str, strAddr: str, nPort: int):
    #     # todo: add UDP later. socket will be determined by bScktype
    #     self.send_external_comm('START', (strAddr, nPort))

    # def external_comm_stop(self):
    #     self.send_external_comm('STOP')

    # def external_comm_send_raw(self, strMsg: str):
    #     self.send_external_comm('SEND_RAW', strMsg)

    # def external_comm_disconnect_client(self, tplAddrPort: tuple):
    #     self.send_external_comm('REMOVE_CLIENT', tplAddrPort)

    # def external_comm_release(self):
    #     self.send_external_comm('RELEASE')


        

if __name__ == '__main__':
    try :
        print('Creat Contorol Core')
        core = EmbdControlCore()

        # core.stt_listen()
        while 1:
            # t = input()

            time.sleep(1)
    except KeyboardInterrupt:
        os._exit(1)