import os
from queue import Queue
import sys
import threading
from turtle import color
import cv2

import jetson_inference
import jetson_utils

import numpy as np
import math

import pickle


import time

#
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros import TransformBroadcaster, TransformStamped
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from deep_sort_realtime.deepsort_tracker import DeepSort
from sensor_msgs.msg import Image, CameraInfo

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
DOCK_PATH = os.path.join(ROOT_PATH, "DockWidgets")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
ROS_PATH = os.path.join(ROOT_PATH, "ROSIntegration")

sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH,ROS_PATH])
sys.path = list(set(sys.path))
del ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH

from Commons import PySignal
from ROSIntegration.ROSNodeBase import CROSNodeBase
from Library.Devices.Camera.RealsenseCam import CRealsenseCamera, ThreadRealsense


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


OBJECT_TRACKING ={
    'CSRT' : cv2.legacy.TrackerCSRT_create,
    'Boosting': cv2.legacy.TrackerBoosting_create,
    'MOSSE': cv2.legacy.TrackerMOSSE_create,
    'KCF': cv2.legacy.TrackerKCF_create,
    'MIL': cv2.legacy.TrackerMIL_create,
    'MedianFlow': cv2.legacy.TrackerMedianFlow_create,
    'TLD' :cv2.legacy.TrackerTLD_create,
}


class CTracking():
    def __init__(self) -> None:
        self.tracker=cv2.legacy.TrackerCSRT_create()
        self.status =False

    def register(self,frame,roi):
        self.tracker=cv2.legacy.TrackerCSRT_create()
        self.tracker.init(frame,roi)
        self.status = True
        print(self.status)

    def update(self,frame):
        ret, roi = self.tracker.update(frame)
        if ret == False:
            self.status =False
            print('tracking lost')
        else :
            pass
        return roi,ret
    def tracking_lost(self):
            self.status = False

class CThreadFacialRecognition(threading.Thread):
    def __init__(self):
        super(CThreadFacialRecognition, self).__init__()
        self.sig_calc_distance = PySignal(tuple)
        self.tracker=CTracking()

        self.targetName = 'None'
        self.followMode = False

        self.overlay_img= None
        self.detectenet= jetson_inference.detectNet("ssd-inception-v2", threshold= 0.7)
        # 사물인식
        self.facenet= jetson_inference.detectNet("facenet", threshold= 0.7)
        # 얼굴

        self.targetPosition = [0., 0.]
        self.targetRotation = 0.
        self.targetDist = 0.
        self.count=0

    def start(self,rgb_frame, depth_frame, cam_params, tf) -> None:
        self.overlay_img = rgb_frame
        self.tf_msg = tf

        if  self.followMode is True and self.tracker.status is True :
            self.proc_traking(rgb_frame, depth_frame, cam_params)
        else :
            self.proc_detection_recognition(rgb_frame, depth_frame, cam_params)

    def stop(self):
        pass


    def proc_traking(self,rgb_frame, depth_frame, cam_params):
        ##
        # print('start tracking')
        t = time.time()
        fx, fy, ppx, ppy, cam_w, cam_h = cam_params
        param = [fx, fy, ppx, ppy]


        roi_frame,ret=self.tracker.update(rgb_frame)
        x,y,w,h=roi_frame

        if x <= 0 or y <= 0:
            ret=False


        if ret ==True :
            roi_image=rgb_frame[int(y) :int(y+h) , int(x):int(x+w)]
            print(roi_image.shape,roi_frame)
            rgb_cu=jetson_utils.cudaFromNumpy(roi_image)
            persons =self.detectenet.Detect(rgb_cu,int(w),int(h))
            _is_person = False
            if len(persons) != 0:
                for obj in persons :
                    if self.detectenet.GetClassDesc(obj.ClassID) == 'person':
                        _is_person = True

            if _is_person == True:
                self.count = 0
            else:
                self.count += 1
            (x,y,w,h)=roi_frame
            cv2.rectangle(self.overlay_img, (round(x), round(y)), (round(x +w),round(y+h)), (0, 255, 255), 2)
            roi_depth = depth_frame[round(y+9/20*h) :round(y+11/20*h) , round(x+9/20*w):round(x+11/20*w)]
            cv2.rectangle(self.overlay_img, (round(x+9/20*w), round(y+9/20*h)), (round(x+11/20*w),round(y+11/20*h)), (255, 255, 0), 2)
            if self.count < 30 :
                if self.calculate_distance(roi_depth, param, x, y, h,w) == True:
                    self.send_follow(_is_person)
                else:
                    print('calculate_distance False')
            else :
                self.tracker.tracking_lost()
                print(_is_person, self.count)
                self.count=0
                pass
        else:
            print("tracking failed")
            pass
        print('tracking_time: ', time.time() - t,'tracking_fps: ',1/(time.time() - t))

    def send_follow(self,_is_person):
        q = quaternion_from_euler(0., 0., self.targetRotation)
        pose_rotation = self.targetRotation
        dist = self.targetDist

        if dist < 0.8:
            pass
        else:
            #localization position
            offset = 0.8
            dist_ = dist - offset
            if pose_rotation > math.pi:
                pose_rotation=pose_rotation-math.pi*2
            elif pose_rotation < -math.pi:
                pose_rotation=pose_rotation+math.pi*2
            map_pose_x = float(math.cos(pose_rotation) * dist_)
            map_pose_y = float(math.sin(pose_rotation) * dist_)

            self.targetPosition[0] += map_pose_x
            self.targetPosition[1] += map_pose_y

        if self.followMode == True and _is_person ==True :
            data =(self.targetDist, self.targetPosition[0], self.targetPosition[1], 0., q)
            self.sig_calc_distance.emit(data)
        else :
            pass

    def identify_face(self, gray_frame, x, y, h, w):
        name = 'None'
        if bIdentify:
            roi_gray = gray_frame[y:y + h, x:x + w]
            id_, conf = recognizer.predict(roi_gray)
            print('name : ',labels[id_] ,' conf :',conf)
            if 4 <= conf <= 100:
                name = labels[id_]
                cv2.rectangle(self.overlay_img, (round(x +2/5*w), round(y+2/5*h)), (round(x+3/5*w),round(y+3/5*h)), (0, 0, 255), 2)
                cv2.putText(self.overlay_img, name, (x + w, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 1, cv2.LINE_AA)


        return name

    def calculate_distance(self,roi_depth, cam_params, x, y, h, w):
        """거리 측정 함수

        Args:
            rgb_frame (image): RGB 이미지
            cam_params (CameraInfo): 카메라 parameter
            roi_depth (image): 관심 영역(FaceRecognition 인식 영역)
            x (uint32): FaceRecognition 인식 영역의 가장 왼쪽 픽셀값
            y (uint32): FaceRecognition 인식 영역의 가장 위쪽 픽셀값
            h (uint32): FaceRecognition 인식 영역의 높이
            w (uint32): FaceRecognition 인식 영역의 넓이
        """
        try:
            m_fx = cam_params[0]
            m_fy = cam_params[1]
            m_cx = cam_params[2]
            m_cy = cam_params[3]
            inv_fx = 1. / m_fx
            inv_fy = 1. / m_fy
            pose_x = self.tf_msg.transform.translation.x
            pose_y = self.tf_msg.transform.translation.y
            rotation_x= self.tf_msg.transform.rotation.x
            rotation_y= self.tf_msg.transform.rotation.y
            rotation_z= self.tf_msg.transform.rotation.z
            rotation_w= self.tf_msg.transform.rotation.w
            rotation_yaw = euler_from_quaternion((rotation_x, rotation_y, rotation_z, rotation_w))[2]
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

                point_z = mean_z * 0.001   # distance in meters

                point_x = ((x + w / 2) - m_cx) * point_z * inv_fx
                point_y = ((y + h / 2) - m_cy) * point_z * inv_fy
                dist = math.sqrt(point_x * point_x + point_y * point_y + point_z * point_z)
                rot_theta = float(math.atan2(point_x ,point_z))
                #localization rotation
                pose_rotation = rotation_yaw - rot_theta
            self.targetPosition = [pose_x, pose_y]
            self.targetRotation = pose_rotation
            self.targetDist = dist

            ret = True
        except:
            ret = False

        return ret



    def proc_detection_recognition(self, rgb_frame, depth_frame, cam_params):
        """FaceRecognition 인식 및 거리 계산

        Args:
            rgb_frame (image): 입력 RGB 이미지
            depth_frame (image): 입력 Depth 이미지
            cam_params (CameraInfo): 카메라 parameter
        """

        fx, fy, ppx, ppy, w, h = cam_params
        img_w=w
        img_h=h
        param = [fx, fy, ppx, ppy]
        # t = time.time()
        cu_img=jetson_utils.cudaFromNumpy(rgb_frame)
        detections =self.detectenet.Detect(cu_img,img_w,img_h)

        if len(detections) != 0:
            for obj in detections :
                if self.detectenet.GetClassDesc(obj.ClassID) == 'person':
                    #bbox
                    x=round(obj.Left)
                    y=round(obj.Top)
                    h=round(obj.Height)
                    w=round(obj.Width)

                    detect_roi=rgb_frame[round(y):round(y+h),round(x):round(x+w)]

                    detect_cu=jetson_utils.cudaFromNumpy(detect_roi)
                    faces =self.facenet.Detect(detect_cu,w,h)

                    cv2.rectangle(self.overlay_img, (x, y), (x + w, y+ h), (0, 255, 0), 2)

                    if len(faces) != 0:
                        for face in faces :
                            face_x=round(face.Left)
                            face_y=round(face.Top)
                            face_h=round(face.Height)
                            face_w=round(face.Width)

                            cv2.rectangle(self.overlay_img, (round(x+face_x), round(y+face_y)), (round(x+face_x +face_w),round(y+face_y+face_h)), (255, 0, 0), 2)

                            # roi_depth = depth_frame[round(y+face_y+2/5*face_h) :round(y+face_y+3/5*face_h),
                            #                         round(x+face_x+2/5*face_w):round(x+face_x +3/5*face_w)]

                            # ident
                            gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
                            try:
                                name = self.identify_face(gray, x+face_x, y+face_y, face_h, face_w)
                                if name == self.targetName:
                                    print(x,y,w,h)
                                    self.tracker.register(rgb_frame,[x,y,w,h] )
                                #register
                            except:
                                print('identify_face, Execept')
        # print('detect_time: ', time.time() - t,'detect_fps: ',1/(time.time() - t))


class CROSNodeFaceRecog(CROSNodeBase):
    """CROSNodeFaceRecog은 FaceRecognition에 대한 클래스이다.
        RGB 카메라 이미지, Depth 카메라 이미지, 카메라 정보를 받아 FaceRecognition을 인식 및 거리 측정

        Args:
            astrTopicName (str, optional): Topic 이름. Defaults to "face_recog".
    """
    def __init__(self, astrTopicName: str = "face_recog"):
        super(CROSNodeFaceRecog, self).__init__('FACE_RECOGNITION', astrTopicName)

        self.face_thread = CThreadFacialRecognition()
        self.face_thread.sig_calc_distance.connect(self.on_calc_distance)

        #ROS
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_msg = None
        self._img_pub = self.create_publisher(Image, '/detected', 10)
        self.bridge = CvBridge()

        self.sig_cam_frames = PySignal(tuple)
        self.sig_face_dist = PySignal(float)
        self.sig_face_xyz = PySignal(tuple)
        self.sig_face_name = PySignal(str)

        self.fPeriod=0.1
        self.prevTime = 0
        self._timer = self.create_timer(self.fPeriod, self.callback)

        self.rs_cam=CRealsenseCamera()
        # self.rs_cam = ThreadRealsense()
        # self.rs_cam.sig_get_frame.connect(self.callback)
        # self.rs_cam.start()

        self.overlay_img = None
        print('FACE_RECOGNITION is on ')

    def callback(self):
        """RGB 이미지, Depth 이미지, 카메라 정보가 수신했을 경우 callback 함수
            각 ROS2_image_msg를 CV2 형식으로 변환시킨다.
        """
        # color, depth, param, fps = data
        color, depth = self.rs_cam.get_frame_stream()
        param = self.rs_cam.param

        currTime = time.time()

        # print(currTime - self.prevTime)
        self.prevTime = currTime

        self.color_ui = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        self.overlay_img = color


        if self.get_tf_msg() == True:
            self.face_thread.start(color, depth, param, self.tf_msg)
        else:
            self.face_thread.start(color, depth, param, -1)

        data = (self.color_ui, )
        self.sig_cam_frames.emit(data)
        self.publish_img()

    def on_calc_distance(self, tplCalc: tuple):
        dist, point_x, point_y, point_z ,q = tplCalc

        self.sig_face_xyz.emit((point_x , point_y, 0.,q[0], q[1], q[2], q[3]))
        self.sig_face_dist.emit(dist)

        self.publish_img()

    def showImg(self, title, img):
        cv2.imshow(title, img)
        cv2.waitKey(1)

    def publish_img(self, img=None):
        if img != None:
            overlay_img_ros = self.bridge.cv2_to_imgmsg(img, "rgb8")
        else:
            overlay_img_ros = self.bridge.cv2_to_imgmsg(self.overlay_img, "rgb8")

        self._img_pub.publish(overlay_img_ros)

    def get_tf_msg(self):
        #Get TF
        head_frame = 'map'
        child_frame = 'base_footprint'
        now = rclpy.time.Time()
        try:
            tf = self.tf_buffer.lookup_transform(
                            head_frame,
                            child_frame,
                            now,
                            # rclpy.duration.Duration(seconds=0.05),
                            )
            ret = True
            self.tf_msg = tf
        except:
            # print("get_tf_msg, except")
            ret = False

        return ret

    def set_target_name(self, name : str) :
        self.face_thread.targetName = name
        print('set target_name: ', self.face_thread.targetName)

    def set_follow_mode(self, mode : bool) :
        self.face_thread.followMode = mode

        if mode == False:
            self.tracker.status = False
        print('set follow_mode: ', self.face_thread.followMode)


def identify_face(gray_frame, x, y, h, w):
    name = 'None'
    if bIdentify:
        roi_gray = gray_frame[y:y + h, x:x + w]
        id_, conf = recognizer.predict(roi_gray)

        print('name: ',labels[id_], 'conf: ', conf)
        if 4 <= conf <= 200:
            name = labels[id_]
            cv2.rectangle(gray_frame, (round(x +2/5*w), round(y+2/5*h)), (round(x+3/5*w),round(y+3/5*h)), (0, 0, 255), 2)
            cv2.putText(gray_frame, name, (x + w, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 1, cv2.LINE_AA)

    return gray_frame

# detectenet= jetson_inference.detectNet("ssd-inception-v2", threshold= 0.7)
# facenet= jetson_inference.detectNet("facenet", threshold= 0.7)

def test(img):
    # if img == None:
    #     img = cv2.imread('/home/nvidia/robot/exa/ExaRobotUI/Include/ROSIntegration/KakaoTalk_20221005_155131225.jpg')

    # img2  = cv2.resize(img, (1280, 720))
    img2 = img
    t = time.time()
    h, w, c = img2.shape
    ret = img2
    t2 = time.time()
    detect_cu=jetson_utils.cudaFromNumpy(img2)
    detections =detectenet.Detect(detect_cu,w,h)
    t2_ = time.time()
    if len(detections) != 0:
            for obj in detections :
                if detectenet.GetClassDesc(obj.ClassID) == 'person':
                    x=round(obj.Left)
                    y=round(obj.Top)
                    h=round(obj.Height)
                    w=round(obj.Width)
                    detect_roi=img2[round(y):round(y+h),round(x):round(x+w)]

                    t3 = time.time()
                    detect_cu=jetson_utils.cudaFromNumpy(detect_roi)
                    faces =facenet.Detect(detect_cu,w,h)
                    t3_ = time.time()
                    if len(faces) != 0:
                        for face in faces :
                            face_x=round(face.Left)
                            face_y=round(face.Top)
                            face_h=round(face.Height)
                            face_w=round(face.Width)

                            gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
                            t4 = time.time()
                            ret = identify_face(gray, face_x, face_y, face_h, face_w)
                            t4_ = time.time()

                            print('Total: {:.3f} Person: {:.3f} Face_de: {:.3f} Face_re: {:.3f}'.format(time.time() - t,
                                                                                t2_ - t2,
                                                                                 t3_ - t3,
                                                                                 t4_ - t4,
                                                                                 ))
                    else:
                        print('not face')

    cv2.imshow('test', ret)
    cv2.waitKey(1)

import datetime


def video_cap():
    fourcc = cv2.VideoWriter_fourcc(*'X264')
    record = False

    cam = CRealsenseCamera()

    while True:

        img, depth = cam.get_frame_stream()

        frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.imshow("VideoFrame", frame)

        now = datetime.datetime.now().strftime("%d_%H-%M-%S")
        key = cv2.waitKey(33)

        if key == 27:
            break
        elif key == 26:
            print("캡쳐")
            cv2.imwrite("./" + str(now) + ".png", frame)
        elif key == 24:
            print("녹화 시작")
            record = True
            video = cv2.VideoWriter("./" + str(now) + ".avi", fourcc, 20.0, (frame.shape[1], frame.shape[0]))
        elif key == 3:
            print("녹화 중지")
            record = False
            video.release()

        if record == True:
            print("녹화 중..")
            video.write(frame)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    # rclpy.init(args=None)
    # gROSNode = CROSNodeFaceRecog()

    # thisExecutor = SingleThreadedExecutor()
    # thisExecutor.add_node(gROSNode)
    # thisExecutorThread = threading.Thread(target=thisExecutor.spin, daemon=True)
    # thisExecutorThread.start()
    # image=cv2.imread("/home/nvidia/robot/exa/ExaRobotUI/Include/ROSIntegration/KakaoTalk_20221005_144825127.jpg",cv2.IMREAD_ANYCOLOR)
    # dst = cv2.resize(image, dsize=(1280, 720), interpolation=cv2.INTER_AREA)

    # print(image)
    # print(dst.shape)
    # # detectenet= jetson_inference.detectNet("ssd-inception-v2", threshold= 0.7)
    # facenet= jetson_inference.detectNet("facenet", threshold= 0.7)

    # cu_img=jetson_utils.cudaFromNumpy(image)
    # detections =detectenet.Detect(cu_img,image.shape[-],img_h)

    # video_cap()

    cam = CRealsenseCamera()

    while True:
        # img, depth = cam.get_frame_stream()
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # test(img)

        # print('test .....')
        time.sleep(1)
