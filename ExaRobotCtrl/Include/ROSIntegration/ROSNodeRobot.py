import os
import sys
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped, StaticTransformBroadcaster, TFMessage

from typing import List, Tuple
import time
import sys
import math

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
DOCK_PATH = os.path.join(ROOT_PATH, "DockWidgets")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH

from Commons import PySignal
from ROSNodeBase import CROSNodeBase

class CROSNodeRobot(CROSNodeBase):
    _odom: Odometry
    """CROSNodeRobot은 로봇 Node를 생성하고 Topic을 발행하는 클래스이다.

        Args:
            afPeriod (float): Odom 토픽 발행 주기
            astrPubTopicTf (str, optional): 발행할 odom 토픽 이름 . Defaults to "odom".
            astrSubTopicVel (str, optional): 구독할 cmd_vel 토픽 이름. Defaults to "cmd_vel".
    """
    def __init__(self, afPeriod: float, astrPubTopicTf: str = "odom", astrSubTopicVel: str = "cmd_vel" ):
        super(CROSNodeRobot, self).__init__('RobotNode', '-')
        self.name_topic_tf = astrPubTopicTf
        self.name_topic_vel = astrSubTopicVel
        #QoS
        self._qos = QoSProfile(depth=10)
        self._qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._qos.history = QoSHistoryPolicy.KEEP_LAST
        self._qos.durability = QoSDurabilityPolicy.VOLATILE
        
        # create publisher
        self._topic_odom = self.create_publisher(Odometry, self.name_topic_tf, self._qos)
        # create subscriber
        self._topic_vel = self.create_subscription(Twist, self.name_topic_vel, self.cmd_vel_callback, 10)
        
        self.fPeriod = afPeriod  # should be in seconds
        self._odom = Odometry()
        self._odom.header.frame_id = 'odom'
        self._odom.child_frame_id = 'base_footprint'

        self._tf_static_publisher = StaticTransformBroadcaster(self)
        self._tf_publisher = TransformBroadcaster(self)
        #base_footprint
        self._base_footprint_tf = TransformStamped()
        self._base_footprint_tf.header.frame_id = 'odom'
        self._base_footprint_tf.child_frame_id = 'base_footprint'

        #base_link
        self._base_link_tf = TransformStamped()
        self._base_link_tf.header.frame_id = 'base_footprint'
        self._base_link_tf.child_frame_id = 'base_link'

        # robot signal
        self.sig_ros_sub_cmd_vel = PySignal(tuple)
        
    def destroy_topic(self):
        self._topic_odom.destroy()
        self._topic_vel.destroy()

    def start_timer(self):
        self._timer = self.create_timer(self.fPeriod, self.timer_callback)
        # print("start_timer ROS Tf Pub Cycle:{}sec".format(self.fPeriod))

    def stop_timer(self):
        self.destroy_timer(self._timer)
        # print("stop_timer ROS Tf Pub ")
    
    def cmd_vel_callback(self, msg: Twist):
        """cmd_vel 구독시 Callback 함수

        Args:
            msg (Twist): cmd_vel (선속도, 각속도)
        """
        vel = msg.linear.x
        ang_vel = msg.angular.z
        # print('robot node ',msg)
        self.sig_ros_sub_cmd_vel.emit((vel, ang_vel))

    def quaternion_from_euler(self, roll, pitch, yaw):
        """euler 좌표계에서 quaternion 좌표계로 변경

        Args:
            roll (float): X축 기준 회전각도
            pitch (float): Y축 기준 회전각도
            yaw (float): Z축 기준 회전각도

        Returns:
            _type_: _description_
        """


        odometry = Odometry()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        odometry.pose.pose.orientation.w = q[0]
        odometry.pose.pose.orientation.x = q[1]
        odometry.pose.pose.orientation.y = q[2]
        odometry.pose.pose.orientation.z = q[3]
        return odometry.pose.pose.orientation
        # return q

    def update_odom_and_tf(self, fPos_x, fPos_y, fPos_theta, fVel_c, fOmega_c):
        """Odom, tf update 함수

        Args:
            fPos_x (float): X 위치
            fPos_y (float): Y 위치
            fPos_theta (float): 회전각 
            fVel_c (float): 선속도
            fOmega_c (float): 각속도
        """
        current_time = self.get_clock().now().to_msg()

        self._odom.pose.pose.position.x = fPos_x
        self._odom.pose.pose.position.y = fPos_y
        self._odom.pose.pose.position.z = 0.0

        self._odom.pose.pose.orientation = self.quaternion_from_euler(0., 0., fPos_theta)
        self._odom.twist.twist.linear.x = fVel_c
        self._odom.twist.twist.linear.y = 0.0
        self._odom.twist.twist.linear.z = 0.0
        self._odom.twist.twist.angular.x = 0.0
        self._odom.twist.twist.angular.y = 0.0
        self._odom.twist.twist.angular.z = fOmega_c
        self._odom.header.stamp = current_time

        self._base_footprint_tf.header.stamp = current_time
        self._base_footprint_tf.transform.translation.x = 0. #fPos_x
        self._base_footprint_tf.transform.translation.y = 0. #fPos_y
        self._base_footprint_tf.transform.translation.z = 0.
        # self._base_footprint_tf.transform.rotation = self._odom.pose.pose.orientation
        self._base_footprint_tf.transform.rotation.w = 1.
        self._base_footprint_tf.transform.rotation.x = 0.
        self._base_footprint_tf.transform.rotation.y = 0.
        self._base_footprint_tf.transform.rotation.z = 0.

        self._base_link_tf.header.stamp = current_time
        self._base_link_tf.transform.translation.x = 0.
        self._base_link_tf.transform.translation.y = 0.
        self._base_link_tf.transform.translation.z = 0.
        self._base_link_tf.transform.rotation.w = 1.
        self._base_link_tf.transform.rotation.x = 0.
        self._base_link_tf.transform.rotation.y = 0.
        self._base_link_tf.transform.rotation.z = 0.
        
    def timer_callback(self):
        try:
            current_time = self.get_clock().now().to_msg()
            self._odom.header.stamp = current_time
            self._base_footprint_tf.header.stamp = current_time
            self._base_link_tf.header.stamp = current_time

            self._topic_odom.publish(self._odom)
            # self._tf_static_publisher.sendTransform(self._base_footprint_tf)
            self._tf_static_publisher.sendTransform(self._base_link_tf)

        except IndexError:
            # reset when the timer is not destroyed outside
            pass

