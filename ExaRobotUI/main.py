# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : main.py
# Project Name : MobileRobotTest
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.08.24 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = FILE_PATH
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH

from MainWindow import MainWindow
from ControlCore import EmbdControlCore
from UiCommon import *


def main():
    controlApp = EmbdControlCore()
    app = QtCore.QCoreApplication.instance()

    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    app.setStyle('fusion')

    MainUI = MainWindow(controlApp, True)
    ScreenRes = QtWidgets.QApplication.desktop().screenGeometry(2)
    QtWidgets.QApplication.setStyle('fusion')
    # MainUI.move(QtCore.QPoint(ScreenRes.x(), ScreenRes.y()))

    MainUI.show()
    # MainUI.resize(ScreenRes.width(), ScreenRes.height())
    MainUI.activateWindow()

    app.exec_()
    MainUI.release()


if __name__ == '__main__':
    main()
