# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DlgFaceRecog.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.09 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
import cv2

from typing import Tuple
from numpy import rad2deg, deg2rad

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include/DOckWidgets
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MOBILE_ROBOT_PATH = os.path.join(INCLUDE_PATH, "MobileRobot")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
DEVICE_PATH = os.path.join(LIBRARY_PATH, "Devices")
CAM_PATH = os.path.join(DEVICE_PATH, "Camera")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([INCLUDE_PATH, MDI_PATH, RESOURCES_PATH, SERIAL_PATH, MOBILE_ROBOT_PATH, MISC_PATH, CAM_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH, MOBILE_ROBOT_PATH, MISC_PATH
del CAM_PATH

from EmbdLed import EmbdLed
from UiCommon import *
from Commons import *
from CameraCommon import COMMON_RESOLUTION_D


class DlgFaceRecog(QtWidgets.QDialog):
    _widgetCtrl: QtWidgets.QWidget = None
    _pixmapImage: QtWidgets.QLabel = None
    sig_close = QtCore.pyqtSignal()
    sig_start = QtCore.pyqtSignal(int, int, int)
    sig_stop = QtCore.pyqtSignal()
    sig_pause = QtCore.pyqtSignal()

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()
        super().closeEvent(a0)

    def __init__(self, parent=None):
        super(DlgFaceRecog, self).__init__(parent)
        self.setWindowTitle('Camera Viewer')
        self.initUi()

    def initUi(self):
        self.initCtrl()
        self._pixmapImage_rgb = QtWidgets.QLabel()
        self._pixmapImage_depth = QtWidgets.QLabel()
        VLayoutMain = QtWidgets.QVBoxLayout(self)

        HLayoutImage = QtWidgets.QHBoxLayout()
        HLayoutImage.addStretch(1)
        HLayoutImage.addWidget(self._pixmapImage_rgb)
        HLayoutImage.addWidget(self._pixmapImage_depth)
        HLayoutImage.addStretch(1)
        VLayoutMain.addLayout(HLayoutImage, 1)
        VLayoutMain.addWidget(self._widgetCtrl)

    def initCtrl(self):
        self._widgetCtrl = QtWidgets.QWidget()
        HLayoutCtrl = QtWidgets.QHBoxLayout(self._widgetCtrl)

    def on_get_image(self, data: tuple):
        rgb, depth, cam_params = data
        qImage = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], QtGui.QImage.Format_RGBA8888)
        self._pixmapImage_rgb.setPixmap(QtGui.QPixmap.fromImage(qImage))
        qImage2 = QtGui.QImage(depth.data, depth.shape[1], depth.shape[0], QtGui.QImage.Format_Grayscale8)
        self._pixmapImage_depth.setPixmap(QtGui.QPixmap.fromImage(qImage2))

    def on_stopped_cam(self):
        self._pixmapImage_rgb.clear()
        self._pixmapImage_depth.clear()


if __name__ == '__main__':
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    MainUI = DlgFaceRecog()
    MainUI.show()

    app.exec_()
