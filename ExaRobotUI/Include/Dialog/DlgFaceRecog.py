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
from PIL import Image
import numpy as np
import pickle

from typing import Tuple
from numpy import rad2deg, deg2rad
import jetson.inference
import jetson.utils

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include/DOckWidgets
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
ROS_PATH = os.path.join(INCLUDE_PATH, "ROSIntegration")
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
del ROOT_PATH, INCLUDE_PATH, MDI_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH, MOBILE_ROBOT_PATH, MISC_PATH 
del CAM_PATH

from Misc.EmbdLed import *
from UiCommon import *
from Commons import *
from Library.Devices.Camera.CameraCommon import *
import cv2


############
HAAR_PATH = os.path.join(ROS_PATH, "cascades/data/haarcascade_frontalface_default.xml")
face_cascade = cv2.CascadeClassifier(HAAR_PATH)
face_cascade1 = cv2.CascadeClassifier(HAAR_PATH)
recognizer = cv2.face.LBPHFaceRecognizer_create()

roi_path = os.path.join(ROS_PATH, "roi_results")
os.makedirs(roi_path, exist_ok=True)

bIdentify = False
MODEL_PATH = os.path.join(ROS_PATH, "recognizers/face-model.yml")
if os.path.isfile(MODEL_PATH):
    recognizer.read(MODEL_PATH)

    LABEL_PATH = os.path.join(ROS_PATH, "pickles/face-labels.pickle")
    labels = {"person_name": 1}
    
    
    with open(LABEL_PATH, 'rb') as f:
        og_labels = pickle.load(f)
        labels = {v: k for k, v in og_labels.items()}

    print("labels: ", labels)
    print("Model: ", MODEL_PATH)
    bIdentify = True
def identify_face(gray_frame, x, y, h, w):
    name = None
    if bIdentify:
        roi_gray = gray_frame[y:y + h, x:x + w]
        id_, conf = recognizer.predict(roi_gray)
        # print('identify_face : ',id_, conf, labels[id_])
        if 4 <= conf <= 200:
            name = labels[id_]
            # cv2.rectangle(gray_frame, (round(x +2/5*w), round(y+2/5*h)), (round(x+3/5*w),round(y+3/5*h)), (0, 0, 255), 2)
    
    return name
############
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
        self.currentRGBA = None
        
        self.strFolderPath = ""
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
        VLayoutCtrl = QtWidgets.QVBoxLayout(self._widgetCtrl)
        HLayoutCtrlUp = QtWidgets.QHBoxLayout()
        self.txtFolderName = QtWidgets.QLineEdit()
        self.txtBrowse = QtWidgets.QLineEdit()
        self.txtBrowse.setReadOnly(True)
        self.btnBrowse = QtWidgets.QPushButton('Browse')
        self.btnGoToFolder = QtWidgets.QPushButton('GOTO')
        self.btnGoToFolder.clicked.connect(self.onClickGotoFolder)
        self.btnBrowse.clicked.connect(self.onClickBtnBrowse)

        HLayoutCtrlUp.addWidget(self.txtBrowse, 1)
        HLayoutCtrlUp.addWidget(self.btnBrowse)
        HLayoutCtrlUp.addWidget(self.txtFolderName)
        VLayoutCtrl.addLayout(HLayoutCtrlUp)

        HLayoutCtrlDown = QtWidgets.QHBoxLayout()
        self.btnCapture = QtWidgets.QPushButton('CAPTURE AND SAVE')
        self.btnCapture.clicked.connect(self.onClickBtnCapture)
        self.btnTrain = QtWidgets.QPushButton("TRAIN")
        self.btnTrain.clicked.connect(self.onClickBtnTrain)
        HLayoutCtrlDown.addWidget(self.btnCapture, 1)
        HLayoutCtrlDown.addWidget(self.btnGoToFolder)
        HLayoutCtrlDown.addWidget(self.btnTrain)
        VLayoutCtrl.addLayout(HLayoutCtrlDown)
        
    def onClickGotoFolder(self):
        if self.strFolderPath != "":
            os.startfile(self.strFolderPath)
        else:
            if self.txtBrowse.text() != "":
                os.startfile(self.txtBrowse.text())

    def onClickBtnBrowse(self):
        strSelectedFolder = QtWidgets.QFileDialog.getExistingDirectory(self, 'Select Images Root Folder')
        strSelectedFolder = os.path.join(strSelectedFolder, "Images")
        self.txtBrowse.setText(strSelectedFolder)

    def onClickBtnCapture(self):
        cntPng = 0
        strFolderName = self.txtFolderName.text()
        if strFolderName == "":
            QtWidgets.QMessageBox.critical(self, "ERROR", "Image Directory Name is not Specified")
        else:
            strPath = os.path.join(self.txtBrowse.text(), strFolderName)
            # this is not really needed, makedirs does not overwrite folder
            if not os.path.isdir(strPath):
                os.makedirs(strPath, exist_ok=True)

            self.strFolderPath = strPath
            for img in os.listdir(self.strFolderPath):
                if img.endswith('.PNG') or img.endswith('.png'):
                    cntPng += 1

            if self.currentRGBA is not None:
                if cntPng > 0:
                    for j in range(cntPng + 1):
                        pngPath = os.path.join(self.strFolderPath, str(j) + ".png")
                        if not os.path.isfile(pngPath):
                            cv2.imwrite(pngPath, self.currentRGBA)
                else:
                    pngPath = os.path.join(self.strFolderPath, "1.png")
                    cv2.imwrite(pngPath, self.currentRGBA)

    def onClickBtnTrain(self):
        image_dir = self.txtBrowse.text()
        if image_dir != "":
            current_id = 0
            label_ids = {}
            y_labels = []
            x_train = []
            ctime = time.time()
            
            i = 0 
            print('traning . . . .', image_dir)
            for root, dirs, files in os.walk(image_dir):
                for file in files:
                    if file.endswith("png") or file.endswith("PNG") or file.endswith("jpg"):
                        path = os.path.join(root, file)
                        label = os.path.basename(root).replace(" ", "-").lower()
                        print(path, file, label)
                        if not label in label_ids:
                            label_ids[label] = current_id
                            current_id += 1
                        id_ = label_ids[label]
                        
                        img = cv2.imread(path) 
                        # img_ = cv2.resize(img, (848,480))
                        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                        # pil_image = Image.open(path).convert("L")  # grayscale
                        # pil_rgb=Image.open(path).convert("RGB")
                        # size = (1280, 720)
                        # final_image = pil_rgb.resize(size, Image.ANTIALIAS)
                        # gfinal_image  = pil_image.resize(size, Image.ANTIALIAS)
                        # image_array = np.array(final_image, "uint8")
                        # gimage_array = np.array(gfinal_image, "uint8")
                        faces = face_cascade.detectMultiScale(img, scaleFactor=1.5, minNeighbors=5)
                            
                        for (x, y, w, h) in faces:
                            roi = gray_image[y:y + h, x:x + w]
                            x_train.append(roi)
                            y_labels.append(id_)
                            print("label: ", label, id_, i)
                            i += 1
                            img_path = os.path.join(roi_path, str(label) + "_" + str(i) + ".png")
                            cv2.imwrite(img_path, roi)
                        
                        # cu_img=jetson.utils.cudaFromNumpy(image_array)
                        # faces =  self.facenet.Detect(cu_img,550,550)
                        # for face in faces:
                        #     face_x=round(face.Left)
                        #     face_y=round(face.Top)
                        #     face_h=round(face.Height)
                        #     face_w=round(face.Width)
                        #     roi = gimage_array[face_y:face_y + face_h, face_x:face_x + face_w]
                        #     x_train.append(roi)
                        #     y_labels.append(id_)
                        #     print("label: ", label, id_, i)
                        #     i += 1
                        #     img_path = os.path.join(roi_path, str(label) + "_" + str(i) + ".png")
                        #     cv2.imwrite(img_path, roi)
                
                
            pickle_path = os.path.join(ROS_PATH, "pickles")
            os.makedirs(pickle_path, exist_ok=True)
            with open(os.path.join(pickle_path, "face-labels.pickle"), 'wb') as f:
                pickle.dump(label_ids, f)

            print('y_labels' ,y_labels)
            recognizer.train(x_train, np.array(y_labels))
            model_path = os.path.join(ROS_PATH, "recognizers")
            os.makedirs(model_path, exist_ok=True)
            recognizer.save(os.path.join(model_path, "face-model.yml"))

            print('tran OK: ',label_ids , time.time()- ctime)
    def on_get_image(self, data: tuple):
        # print('나오나요 ?')
        
        try:
            rgb_, = data
            
            rgb = cv2.cvtColor(rgb_, cv2.COLOR_BGRA2RGBA)
            
            self.currentRGBA = rgb_
            resize_rgb = cv2.resize(rgb, (848,480))
            
            faces = face_cascade1.detectMultiScale(resize_rgb, scaleFactor=1.5, minNeighbors=5) #얼굴 인식
            for (x, y, w, h) in faces :
                cv2.rectangle(resize_rgb,(x,y),(x+w,y+h),(0,255,0),2)
                
                gray_image = cv2.cvtColor(resize_rgb, cv2.COLOR_BGR2GRAY)

                name = identify_face(gray_image, x,y,h,w)

                if name  != None:
                    cv2.putText(rgb, name, (x + w, y), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 0, 255), 1, cv2.LINE_AA)
                
            # resize_rgb = cv2.resize(rgb, (424,240))

            qImage = QtGui.QImage(resize_rgb.data, resize_rgb.shape[1], resize_rgb.shape[0], QtGui.QImage.Format_RGBA8888)
            self._pixmapImage_rgb.setPixmap(QtGui.QPixmap.fromImage(qImage))
            
            qImage2  = qImage
            # # qImage2 = QtGui.QImage(depth.data, depth.shape[1], depth.shape[0], QtGui.QImage.Format_Grayscale8)
            # qImage2 = QtGui.QImage(resize_rgb2.data, resize_rgb2.shape[1], resize_rgb2.shape[0], QtGui.QImage.Format_RGBA8888)
            self._pixmapImage_depth.setPixmap(QtGui.QPixmap.fromImage(qImage2))
            
        except:
            print("on_get_image, except")
            pass

    def on_stopped_cam(self):
        self._pixmapImage_rgb.clear()
        self._pixmapImage_depth.clear()


if __name__ == '__main__':
    # app = QtCore.QCoreApplication.instance()
    # if app is None:
    #     app = QtWidgets.QApplication(sys.argv)
    #
    # MainUI = DlgFaceRecog()
    # MainUI.show()
    #
    # app.exec_()

    for i in range(1 + 1):
        print(i)
