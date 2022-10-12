# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPTextToSpeech.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.05.05 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import multiprocessing as mp
import os
import sys
from typing import Tuple
from multiprocessing import Queue
# from multiprocessing.connection import PipeConnection
from typing import Union
import time

import psutil

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
EXTERNAL_COMM_PATH = os.path.join(INCLUDE_PATH, "ExternalComm")
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(INCLUDE_PATH, "Resources")
MULTIPROCESS_PATH = os.path.join(INCLUDE_PATH, "MultiProcessing")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
TTS_PATH = os.path.join(INCLUDE_PATH, 'AudioRecognition')

sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH,TTS_PATH,
                 MULTIPROCESS_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH, MULTIPROCESS_PATH

from Commons import PySignal
from MultiProcessBase import CMultiProcessBase
from AudioRecognition.SpeechToText import *

gSpeechToText = None
gfeedbackQueue = None

# dict: CMD, VALUE
# pipeChild is an instance of PipeConnection on Windows, but this is different on Linux
def proc_speech_to_text(pipeChild, feedbackQueue: Queue, feedbackQueueBk: Union[Queue, None]):
    """
    Speech To Text 에 대한 프로세스이다.
        
    Pipe로 통해 온 메시지를 처리한다.

    Command 리스트 : CREATE, SEND_MSG

    Args:
        pipeChild (_type_): Text_to_speech Pipe object
        feedbackQueue (Queue): Text_to_speech Queue object
        feedbackQueueBk (Union[Queue, None]): Text_to_speech QueueBk object
    """
    global gSpeechToText
    global gfeedbackQueue
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
                    gSpeechToText = CSpeechToText()
                    gSpeechToText.sig_stt_msg.connect(send_stt_msg)
                    print("create stt thread")
                    
                    gfeedbackQueue = feedbackQueue
                elif cmd == "LISTEN":
                    gSpeechToText.start_listen('default')
                    
                    
                else:
                    pass

                time.sleep(1e-3)
            else:
                time.sleep(1e-3)
        except:
            time.sleep(1e-3)

        time.sleep(1e-6)


def send_stt_msg(velMsg: str,):
    global gfeedbackQueue
    q: Queue = gfeedbackQueue
    
    print('send_stt_msg: ', velMsg)
    dict_msg = dict()
    dict_msg['STT_MSG'] = velMsg
    q.put(dict_msg)