import os
import queue
import sys
import threading

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import rclpy
import numpy as np
import math
from message_filters import TimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.node import Node
# from rclpy import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import cv2
import pickle
from typing import List, Tuple
import time

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
DOCK_PATH = os.path.join(ROOT_PATH, "DockWidgets")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH
from Commons import PySignal
from ROSNodeBase import CROSNodeBase

# todo: add data acqusition for learning tool later
HAAR_PATH = os.path.join(FILE_PATH, "cascades/data/haarcascade_frontalface_alt2.xml")
face_cascade = cv2.CascadeClassifier(HAAR_PATH)
recognizer = cv2.face.LBPHFaceRecognizer_create()
bIdentify = False
MODEL_PATH = os.path.join(FILE_PATH, "recognizers/face-model.yml")

if os.path.isfile(MODEL_PATH):
    recognizer.read(MODEL_PATH)

    LABEL_PATH = os.path.join(FILE_PATH, "pickles/face-labels.pickle")
    labels = {"person_name": 1}
    with open(LABEL_PATH, 'rb') as f:
        og_labels = pickle.load(f)
        labels = {v: k for k, v in og_labels.items()}

    bIdentify = True


class CThreadFacialRecognition(threading.Thread):
    def __init__(self, rgb_frame, depth_frame, cam_params):
        super(CThreadFacialRecognition, self).__init__()
        # dist, x, y, z, rw, rx, ry, rz
        self.sig_calc_distance = PySignal(tuple)
        self.rgb_frame = rgb_frame
        self.depth_frame = depth_frame
        self.cam_params = cam_params

    def run(self) -> None:
        self.proc_face_recog_distance(self.rgb_frame, self.depth_frame, self.cam_params)

    def stop(self):
        pass

    def calculate_distance(self, rgb_frame, cam_params, roi_depth, x, y, h, w):
        m_fx = cam_params[0]
        m_fy = cam_params[4]
        m_cx = cam_params[2]
        m_cy = cam_params[5]
        inv_fx = 1. / m_fx
        inv_fy = 1. / m_fy

        n = 0
        sum = 0
        for i in range(0, roi_depth.shape[0]):
            for j in range(0, roi_depth.shape[1]):
                value = roi_depth.item(i, j)
                if value > 0.:
                    n = n + 1
                    sum += value

        if n != 0:
            mean_z = sum / n

            point_z = mean_z * 0.001  # distance in meters
            point_x = ((x + w / 2) - m_cx) * point_z * inv_fx
            point_y = ((y + h / 2) - m_cy) * point_z * inv_fy

            x_str = "X: " + str(format(point_x, '.2f'))
            y_str = "Y: " + str(format(point_y, '.2f'))
            z_str = "Z: " + str(format(point_z, '.2f'))

            cv2.putText(rgb_frame, x_str, (x + w, y + 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(rgb_frame, y_str, (x + w, y + 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(rgb_frame, z_str, (x + w, y + 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 1, cv2.LINE_AA)

            dist = math.sqrt(point_x * point_x + point_y * point_y + point_z * point_z)
            tplSignal = (dist, point_x, point_y, point_z)
            self.sig_calc_distance.emit(tplSignal)
            dist_str = "dist:" + str(format(dist, '.2f')) + "m"

            cv2.putText(rgb_frame, dist_str, (x + w, y + 80), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 1, cv2.LINE_AA)

    def identify_face(self, rgb_frame, gray_frame, x, y, h, w):
        if bIdentify:
            print("identify start")
            roi_gray = gray_frame[y:y + h, x:x + w]
            id_, conf = recognizer.predict(roi_gray)
            if 4 <= conf <= 85:
                name = labels[id_]
                cv2.putText(rgb_frame, name, (x + w, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (255, 255, 255), 1, cv2.LINE_AA)

    def proc_face_recog_distance(self, rgb_frame, depth_frame, cam_params):
        # print('proc_face_recog_distance start')
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5)
        for (x, y, w, h) in faces:
            cv2.rectangle(rgb_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.rectangle(depth_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            roi_depth = depth_frame[y + 30:y + h - 30, x + 30:x + w - 30]

            self.identify_face(rgb_frame, gray, x, y, h, w)
            self.calculate_distance(rgb_frame, cam_params, roi_depth, x, y, h, w)


class CROSNodeFaceRecog(CROSNodeBase):
    def __init__(self, astrTopicName: str = "face_recog"):
        super(CROSNodeFaceRecog, self).__init__('FACE_RECOGNITION', astrTopicName)
        self.bridge = CvBridge()
        color = Subscriber(self, Image, '/camera/color/image_raw')
        depth = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        camera_info = Subscriber(self, CameraInfo, '/camera/color/camera_info')
        self._topic_tf = self.create_subscription(TransformStamped, "/tf", self.tf_callback, 10)
        self.sub = TimeSynchronizer([depth, color, camera_info], 10)
        self.sub.registerCallback(self.callback)
        self.sig_cam_frames = PySignal(tuple)
        self.sig_face_dist = PySignal(float)
        self.sig_face_xyz = PySignal(tuple)
        self.sig_face_name = PySignal(str)

        self.pose_x = 0.
        self.pose_y = 0.
        self.pose_z = 0.
        self.rotation_x = 0.
        self.rotation_y = 0.
        self.rotation_z = 0.
        self.rotation_w = 0.

        # self._TfBroadcaster = TransformBroadcaster(self, 10)
        self._face_tf = TransformStamped()
        self._face_tf.header.frame_id = 'map'
        self._face_tf.child_frame_id = 'face'

        print('FACE_RECOGNITION is on ')

    # def proc_face_recog_distance(self, rgb_frame, depth_frame, cam_params):
    #     # print('proc_face_recog_distance start')
    #     gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
    #     faces = face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5)
    #     for (x, y, w, h) in faces:
    #         cv2.rectangle(rgb_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #         cv2.rectangle(depth_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #         roi_depth = depth_frame[y + 30:y + h - 30, x + 30:x + w - 30]
    #
    #         self.identify_face(rgb_frame, gray, x, y, h, w)
    #         self.calculate_distance(rgb_frame, cam_params, roi_depth, x, y, h, w)

    def identify_face(self, rgb_frame, gray_frame, x, y, h, w):
        if bIdentify:
            roi_gray = gray_frame[y:y + h, x:x + w]
            id_, conf = recognizer.predict(roi_gray)
            if 4 <= conf <= 85:
                name = labels[id_]
                cv2.putText(rgb_frame, name, (x + w, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (255, 255, 255), 1, cv2.LINE_AA)

    def calculate_distance(self, rgb_frame, cam_params, roi_depth, x, y, h, w):
        m_fx = cam_params[0]
        m_fy = cam_params[4]
        m_cx = cam_params[2]
        m_cy = cam_params[5]
        inv_fx = 1. / m_fx
        inv_fy = 1. / m_fy

        n = 0
        sum = 0
        for i in range(0, roi_depth.shape[0]):
            for j in range(0, roi_depth.shape[1]):
                value = roi_depth.item(i, j)
                if value > 0.:
                    n = n + 1
                    sum += value

        if n != 0:
            mean_z = sum / n

            point_z = mean_z * 0.001  # distance in meters
            point_x = ((x + w / 2) - m_cx) * point_z * inv_fx
            point_y = ((y + h / 2) - m_cy) * point_z * inv_fy

            x_str = "X: " + str(format(point_x, '.2f'))
            y_str = "Y: " + str(format(point_y, '.2f'))
            z_str = "Z: " + str(format(point_z, '.2f'))

            cv2.putText(rgb_frame, x_str, (x + w, y+20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(rgb_frame, y_str, (x + w, y + 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(rgb_frame, z_str, (x + w, y + 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 1, cv2.LINE_AA)

            dist = math.sqrt(point_x * point_x + point_y * point_y + point_z * point_z)

            self.sig_face_dist.emit(dist)
            self.sig_face_xyz.emit((-(point_x + self.pose_y), point_y, point_z + self.pose_x,
                                    self.rotation_w, self.rotation_x, self.rotation_y, self.rotation_z))
            # print(point_x+self.pose_y, point_y, point_z+self.pose_x, self.rotation_w, self.rotation_x, self.rotation_y, self.rotation_z)
            dist_str = "dist:" + str(format(dist, '.2f')) + "m"

            cv2.putText(rgb_frame, dist_str, (x + w, y + 80), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 1, cv2.LINE_AA)

            current_time = self.get_clock().now().to_msg()
            self._face_tf.header.stamp = current_time
            self._face_tf.transform.translation.x = point_z + self.pose_x
            self._face_tf.transform.translation.y = point_x + self.pose_y
            self._face_tf.transform.translation.z = point_y
            self._face_tf.transform.rotation.w = 1.  # self.rotation_w
            self._face_tf.transform.rotation.x = 0.  # self.rotation_x
            self._face_tf.transform.rotation.y = 0.  # elf.rotation_y
            self._face_tf.transform.rotation.z = 0.  # self.rotation_z
            # self._TfBroadcaster.sendTransform(self._face_tf)

    def proc_face_recog_distance(self, rgb_frame, depth_frame, cam_params):
        # print('proc_face_recog_distance start')
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5)
        for (x, y, w, h) in faces:
            cv2.rectangle(rgb_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.rectangle(depth_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            roi_depth = depth_frame[y + 30:y + h - 30, x + 30:x + w - 30]

            self.identify_face(rgb_frame, gray, x, y, h, w)
            self.calculate_distance(rgb_frame, cam_params, roi_depth, x, y, h, w)


        # print('proc_face_recog_distance end')

    # def identify_face(self, rgb_frame, gray_frame, x, y, h, w):
    #     if bIdentify:
    #         print("identify start")
    #         roi_gray = gray_frame[y:y + h, x:x + w]
    #         id_, conf = recognizer.predict(roi_gray)
    #         if 4 <= conf <= 85:
    #             name = labels[id_]
    #             cv2.putText(rgb_frame, name, (x + w, y), cv2.FONT_HERSHEY_SIMPLEX,
    #                         0.7, (255, 255, 255), 1, cv2.LINE_AA)

    # def calculate_distance(self, rgb_frame, cam_params, roi_depth, x, y, h, w):
    #     m_fx = cam_params[0]
    #     m_fy = cam_params[4]
    #     m_cx = cam_params[2]
    #     m_cy = cam_params[5]
    #     inv_fx = 1. / m_fx
    #     inv_fy = 1. / m_fy
    #
    #     n = 0
    #     sum = 0
    #     for i in range(0, roi_depth.shape[0]):
    #         for j in range(0, roi_depth.shape[1]):
    #             value = roi_depth.item(i, j)
    #             if value > 0.:
    #                 n = n + 1
    #                 sum += value
    #
    #     if n != 0:
    #         mean_z = sum / n
    #
    #         point_z = mean_z * 0.001  # distance in meters
    #         point_x = ((x + w / 2) - m_cx) * point_z * inv_fx
    #         point_y = ((y + h / 2) - m_cy) * point_z * inv_fy
    #
    #         x_str = "X: " + str(format(point_x, '.2f'))
    #         y_str = "Y: " + str(format(point_y, '.2f'))
    #         z_str = "Z: " + str(format(point_z, '.2f'))
    #
    #         cv2.putText(rgb_frame, x_str, (x + w, y + 20), cv2.FONT_HERSHEY_SIMPLEX,
    #                     0.7, (0, 0, 255), 1, cv2.LINE_AA)
    #         cv2.putText(rgb_frame, y_str, (x + w, y + 40), cv2.FONT_HERSHEY_SIMPLEX,
    #                     0.7, (0, 0, 255), 1, cv2.LINE_AA)
    #         cv2.putText(rgb_frame, z_str, (x + w, y + 60), cv2.FONT_HERSHEY_SIMPLEX,
    #                     0.7, (0, 0, 255), 1, cv2.LINE_AA)
    #
    #         dist = math.sqrt(point_x * point_x + point_y * point_y + point_z * point_z)
    #
    #         self.sig_face_dist.emit(dist)
    #         self.sig_face_xyz.emit((-(point_x + self.pose_y), point_y, point_z + self.pose_x,
    #                                 self.rotation_w, self.rotation_x, self.rotation_y, self.rotation_z))
    #         # print(point_x+self.pose_y, point_y, point_z+self.pose_x, self.rotation_w, self.rotation_x, self.rotation_y, self.rotation_z)
    #         dist_str = "dist:" + str(format(dist, '.2f')) + "m"
    #
    #         cv2.putText(rgb_frame, dist_str, (x + w, y + 80), cv2.FONT_HERSHEY_SIMPLEX,
    #                     0.7, (0, 255, 0), 1, cv2.LINE_AA)
    #
    #         current_time = self.get_clock().now().to_msg()
    #         self._face_tf.header.stamp = current_time
    #         self._face_tf.transform.translation.x = point_z + self.pose_x
    #         self._face_tf.transform.translation.y = point_x + self.pose_y
    #         self._face_tf.transform.translation.z = point_y
    #         self._face_tf.transform.rotation.w = 1.  # self.rotation_w
    #         self._face_tf.transform.rotation.x = 0.  # self.rotation_x
    #         self._face_tf.transform.rotation.y = 0.  # elf.rotation_y
    #         self._face_tf.transform.rotation.z = 0.  # self.rotation_z
            # self._TfBroadcaster.sendTransform(self._face_tf)

    def callback(self, depth, color, camera_info):
        # print('callback start')
        my_depth = None
        my_color = None
        my_cam_params = None
        try:
            my_depth = self.bridge.imgmsg_to_cv2(depth, "32FC1")
            # my_depth = self.bridge.imgmsg_to_cv2(depth, "passthrough")
            my_color = self.bridge.imgmsg_to_cv2(color, "rgb8")

            # Intrinsic camera matrix for the raw (distorted) images.
            #     [fx  0 cx]
            # k = [ 0 fy cy]
            #     [ 0  0  1]
            my_cam_params = camera_info.k

        except CvBridgeError as e:
            pass

        if my_depth is not None and my_color is not None and my_cam_params is not None:
            self.sig_cam_frames.emit((my_color, my_depth, my_cam_params))
            face_thread = CThreadFacialRecognition(my_color, my_depth, my_cam_params)
            face_thread.sig_calc_distance.connect(self.on_calc_distance)
            face_thread.start()
            # self.proc_face_recog_distance(my_color, my_depth, my_cam_params)

    def on_calc_distance(self, tplCalc: tuple):
        dist, point_x, point_y, point_z = tplCalc

        current_time = self.get_clock().now().to_msg()
        self._face_tf.header.stamp = current_time
        self._face_tf.transform.translation.x = point_z + self.pose_x
        self._face_tf.transform.translation.y = point_x + self.pose_y
        self._face_tf.transform.translation.z = point_y
        self._face_tf.transform.rotation.w = 1.  # self.rotation_w
        self._face_tf.transform.rotation.x = 0.  # self.rotation_x
        self._face_tf.transform.rotation.y = 0.  # elf.rotation_y
        self._face_tf.transform.rotation.z = 0.  # self.rotation_z

        self.sig_face_dist.emit(dist)
        self.sig_face_xyz.emit((-(point_x + self.pose_y), point_y, point_z + self.pose_x,
                                self.rotation_w, self.rotation_x, self.rotation_y, self.rotation_z))

    def tf_callback(self, msg: TransformStamped):

        if msg.header.frame_id != 'odom':
            return
        else:
            # translation= msg.transform.translation
            self.pose_x = msg.transform.translation.x
            self.pose_y = msg.transform.translation.y
            self.pose_z = msg.transform.translation.z
            self.rotation_x = msg.transform.rotation.x
            self.rotation_y = msg.transform.rotation.y
            self.rotation_z = msg.transform.rotation.z
            self.rotation_w = msg.transform.rotation.w
