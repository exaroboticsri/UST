# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MainWindow.py
# Project Name : MobileRobotTest
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.08.24 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
import datetime
from typing import List

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = os.path.dirname(FILE_PATH)
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
DIALOG_PATH = os.path.join(INCLUDE_PATH, "Dialog")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MISC_PATH, RESOURCES_PATH,DIALOG_PATH ])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH, MISC_PATH,

from UiCommon import *
from Commons import *
from EmbdLed import EmbdLed
from ControlCore import EmbdControlCore
from DlgFaceRecog import DlgFaceRecog


__appname__ = "EMBD Mobile Robot Test"
__author__ = u"Raim.Delgado"
__build__ = "2022.01.22"
__credits__ = "Prof. Byoung Wook Choi"
__license__ = "GPL"
__status__ = "(Alpha)"
__version__ = "0.0.1" + " " + __status__
__maintainer__ = "Raim.Delgado"
__email__ = "raim223@seoultech.ac.kr"


class ThreadMonitoring(QtCore.QThread):
    _keepAlive: bool = True

    sig_terminated = QtCore.pyqtSignal()
    sig_current_time = QtCore.pyqtSignal(datetime.datetime)

    def __init__(self):
        super(ThreadMonitoring, self).__init__()

    def run(self):
        while self._keepAlive:
            self.sig_current_time.emit(datetime.datetime.now())
            self.msleep(100)

        self.sig_terminated.emit()

    def stop(self):
        self._keepAlive = False


class MainWindow(QtWidgets.QMainWindow):
    _core: EmbdControlCore = None
    _sim_mode: bool
    _widgetTableButtons: QtWidgets.QWidget
    _widgetStatus: QtWidgets.QWidget
    _widgetHeader: QtWidgets.QWidget
    _list_btn_table: List[QtWidgets.QPushButton]
    _dlgFaceRecog: DlgFaceRecog = None

    def __init__(self, aembdCore: EmbdControlCore, a_is_sim_mode: bool = False, parent=None):
        super(MainWindow, self).__init__(parent)
        self._core = aembdCore
        self._sim_mode = a_is_sim_mode
        self.height= 1000
        self.width= 1800
        
        self.initUi()

        if not self._sim_mode:
            self.setWindowFlags(self.windowFlags() | QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint)

        self.initSlots()

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.release()

    def release(self):
        self.stopThreadMonitoring()
        self._core.release()

    def initUi(self):
        self.setStyleSheet("background-color: white;")
        centralWidget = QtWidgets.QWidget(self)
        MainLayout = QtWidgets.QVBoxLayout(centralWidget)
        self.setCentralWidget(centralWidget)
        self.setWindowTitle(__appname__)
        self.initButtons()
        self.initToolBar()
        self.initStatus()
        self.initHeader()
        self.initDialog()
       

        MainLayout.addWidget(self._widgetHeader)
        MainLayout.addStretch()
        MainLayout.addWidget(self._widgetTableButtons)
        MainLayout.addStretch()
        MainLayout.addWidget(self._widgetStatus)

    def initDialog(self):
        self._dlgFaceRecog = DlgFaceRecog(self)
        self._core.sig_facerecog_stream.connect(self._dlgFaceRecog.on_get_image)

    def initToolBar(self):
        self.toolBar = QtWidgets.QToolBar(self)
        self.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        self.toolBar.setMovable(False)  # steady
        self.toolBar.addSeparator()

        self.showDlgFaceRecognition = QtWidgets.QAction(EmbdQtIcon('face.png'), 'Face Recognition', self)
        self.showDlgFaceRecognition.triggered.connect(self.onTriggeredDlgFaceRecog)
        self.toolBar.addAction(self.showDlgFaceRecognition)
        self.toolBar.addSeparator()


    def initHeader(self):
        self._widgetHeader = QtWidgets.QWidget()
        HLayoutHeader = QtWidgets.QHBoxLayout(self._widgetHeader)
        lblHeader = QtWidgets.QLabel()
        # lblHeader.setText('RAIM')
        lblHeader.setPixmap(EmbdQtPixmap('seoultech_wide2.jpg'))
        HLayoutHeader.addWidget(lblHeader)
        HLayoutHeader.addStretch()

        self.btnStop = QtWidgets.QPushButton()
        self.btnStop.setIcon(EmbdQtIcon('stop.png'))
        self.btnStop.setFlat(True)
        self.btnStop.setIconSize(QtCore.QSize(300, 300))
        self.btnStop.setFixedSize(QtCore.QSize(300, 300))
        HLayoutHeader.addWidget(self.btnStop)

    def initButtons(self):
        self._list_btn_table = []
        self._widgetTableButtons = QtWidgets.QWidget()
        VLayoutButtons = QtWidgets.QVBoxLayout(self._widgetTableButtons)

        HlayoutChefBtn = QtWidgets.QHBoxLayout()
        self.btnMoveOrig = QtWidgets.QPushButton()
        self.btnMoveOrig.setFlat(True)
        self.btnMoveOrig.setIcon(EmbdQtIcon('506icon.png'))
        self.btnMoveOrig.setIconSize(QtCore.QSize(300, 300))
        self.btnMoveOrig.setFixedSize(QtCore.QSize(300, 300))
        HlayoutChefBtn.addStretch()
        HlayoutChefBtn.addWidget(self.btnMoveOrig)
        HlayoutChefBtn.addStretch()
        VLayoutButtons.addLayout(HlayoutChefBtn)

        GridLayoutButtons = QtWidgets.QGridLayout()
        GridLayoutButtons.setSpacing(100)

        for i in range(4):
            btnTable = QtWidgets.QPushButton()
            btnTable.setIcon(EmbdQtIcon("table_" + str(i + 1) + ".png"))
            btnTable.setIconSize(QtCore.QSize(200, 200))
            btnTable.setFixedSize(QtCore.QSize(200, 200))
            btnTable.setFlat(True)
            btnTable.MyID = i + 1
            self._list_btn_table.append(btnTable)
            


       
        GridLayoutButtons.addWidget(self._list_btn_table[0], 0, 0)
        GridLayoutButtons.addWidget(self._list_btn_table[1], 0, 1)
        GridLayoutButtons.addWidget(self._list_btn_table[2], 1, 0)
        GridLayoutButtons.addWidget(self._list_btn_table[3], 1, 1)
        VLayoutButtons.addLayout(GridLayoutButtons)
        
        HlayoutCFollowBtn = QtWidgets.QHBoxLayout()
        self.btnFollowFace = QtWidgets.QPushButton()
        self.btnFollowFace.setFlat(True)
        self.btnFollowFace.setIcon(EmbdQtIcon('off.png'))
        self.btnFollowFace.setIconSize(QtCore.QSize(300, 300))
        self.btnFollowFace.setFixedSize(QtCore.QSize(300, 300))
        self.btnFollowFace.setCheckable(True)
        HlayoutCFollowBtn.addStretch()
        HlayoutCFollowBtn.addWidget(self.btnFollowFace)
        HlayoutCFollowBtn.addStretch()
        VLayoutButtons.addLayout(HlayoutCFollowBtn)
        


    def initStatus(self):
        self._widgetStatus = QtWidgets.QWidget()
        HLayoutStatus = QtWidgets.QHBoxLayout(self._widgetStatus)

        self.ledIdle = EmbdLed('IDLE')
        self.ledIdle.setLedSize(35, 35)
        self.ledIdle.setTextSize(25)
        HLayoutStatus.addWidget(self.ledIdle)

        self.ledMoveToOrig = EmbdLed('MOVE_ORIGIN')
        self.ledMoveToOrig.setLedSize(35, 35)
        self.ledMoveToOrig.setTextSize(25)
        HLayoutStatus.addWidget(self.ledMoveToOrig)

        self.ledMoveToTable = EmbdLed('MOVE_TABLE')
        self.ledMoveToTable.setLedSize(35, 35)
        self.ledMoveToTable.setTextSize(25)
        HLayoutStatus.addWidget(self.ledMoveToTable)

        ipRange = "(?:[0-1]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])"  # Part of the regular expression
        ipRegex = QtCore.QRegExp("^" + ipRange + "\\." + ipRange + "\\." + ipRange + "\\." + ipRange + "$")
        ipValidator = QtGui.QRegExpValidator(ipRegex, self)
        self.txtAddress = QtWidgets.QLineEdit()
        self.txtAddress.setPlaceholderText("0.0.0.0")
        self.txtAddress.setText('192.168.1.3')
        self.txtAddress.setValidator(ipValidator)
        self.btnConnect = QtWidgets.QPushButton("CONNECT")
        self.btnConnect.setFixedHeight(35)

        VLayoutIP = QtWidgets.QVBoxLayout()
        VLayoutIP.addWidget(self.txtAddress)
        VLayoutIP.addWidget(self.btnConnect)
        VLayoutIP.setSpacing(5)
        VLayoutIP.setContentsMargins(0, 0, 0, 0)

        self.ledComm = EmbdLed('COMM')
        self.ledComm.setLedSize(35, 35)
        self.ledComm.setTextSize(25)

        HLayoutStatus.addStretch()
        HLayoutStatus.addLayout(VLayoutIP)
        HLayoutStatus.addWidget(self.ledComm)
        HLayoutStatus.setSpacing(10)
        HLayoutStatus.setContentsMargins(5, 0, 0, 5)

    def initSlots(self):
        self._core.sig_tcp_error.connect(lambda: self.change_tcp_status("ERROR"))
        self._core.sig_tcp_connected.connect(lambda: self.change_tcp_status("CONN"))
        self._core.sig_tcp_disconnected.connect(lambda: self.change_tcp_status("DISCONN"))
        self.btnConnect.clicked.connect(self.on_click_btn_connect)
        self.btnMoveOrig.released.connect(self.on_release_btn_home)
        self.btnFollowFace.clicked.connect(self.on_click_btn_FollowFace)
        self.btnStop.clicked.connect(self.on_clicked_emg_stop)
        for btn in self._list_btn_table:
            btn.released.connect(self.on_release_table_btn)

    def on_clicked_emg_stop(self):
        self._core.set_emg_stop()

    def on_release_btn_home(self):
        self._core.move_to_target(0)

    def on_release_table_btn(self):
        btn = self.sender()
        tableNumber = btn.MyID  # actual table number, 0 is reserved for home
        self._core.move_to_target(tableNumber)

    def on_click_btn_connect(self):
        btnTxt = self.btnConnect.text()
        ipAddress = self.txtAddress.text() if self.txtAddress.text() != "" else "0.0.0.0"

        if btnTxt == "CONNECT":
            self._core.start_client(ipAddress)
        else:
            self._core.stop_client()
            
    def on_click_btn_FollowFace(self):  
        if self.btnFollowFace.isChecked() == True:
            self.btnFollowFace.setIcon(EmbdQtIcon('on.png'))
            self._core.follow_start()
        else :
            self.btnFollowFace.setIcon(EmbdQtIcon('off.png'))
            self._core.follow_stop()
     
    def change_tcp_status(self, strState):
        bStatus = False
        if strState == "ERROR":
            self.ledComm.TurnOnR()
        elif strState == "CONN":
            self.ledComm.TurnOnG()
            bStatus = True
        else:
            self.ledComm.TurnOff()

        if bStatus:
            self.btnConnect.setText("DISCONNECT")
        else:
            self.btnConnect.setText("CONNECT")

        self.txtAddress.setEnabled(not bStatus)
        
    def onTriggeredDlgFaceRecog(self):
        if not self._dlgFaceRecog.isVisible():
            self._dlgFaceRecog = DlgFaceRecog(self)
            self._dlgFaceRecog.show()
            # self._dlgFaceRecog.sig_close.connect(self._core.stopCam)
            # self._dlgFaceRecog.sig_stop.connect(self._core.stopCam)
            # self._dlgFaceRecog.sig_start.connect(self._core.onCamStart)
            self._core.sig_facerecog_stream.connect(self._dlgFaceRecog.on_get_image)
            # self._core.sig_cam_stopped_stream.connect(self._dlgCamViewer.on_stopped_cam)

        self._dlgFaceRecog.activateWindow()
