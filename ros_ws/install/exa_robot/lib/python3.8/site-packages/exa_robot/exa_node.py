from exa_robot.exa_driver import *
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time
import os
from tf2_ros import TransformStamped, TransformBroadcaster



class CRobotNode(Node):
    def __init__(self,  odomPublishTime = 0.02):
        super().__init__('robot_node')
        qos = QoSProfile(depth = 10)
        #qos.reliability = QoSReliabilityPolicy.BEST_EFFORT #데이터 송신 속도에 중시 
        qos.reliability = QoSReliabilityPolicy.RELIABLE #데이터 송신 신뢰성에 중시 
        qos.history = QoSHistoryPolicy.KEEP_LAST #정해진 depth 만큼 큐를 저장
        qos.durability = QoSDurabilityPolicy.VOLATILE #구독자가 생기기 전까지 데이터 무효

        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel',self.velCallback, 10)

        timer_period = odomPublishTime
        self.odometry=Odometry()
        self.odometry.header.frame_id = 'odom'
        self.odometry.child_frame_id = 'base_link'
        self.timer = self.create_timer(timer_period, self.odomUpdate)

        self.broadcaster = TransformBroadcaster(self,10)
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'

        self.robot = CExaRobot()
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """

        odometry=Odometry()
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

        odometry.pose.pose.orientation.w=q[0]
        odometry.pose.pose.orientation.x=q[1]
        odometry.pose.pose.orientation.y=q[2]
        odometry.pose.pose.orientation.z=q[3]
        

        return odometry.pose.pose.orientation
        #return q
    def odomUpdate(self):
        x,y,v ,theta, angleVel= self.robot.GetRobotData()
        current_time = self.get_clock().now().to_msg()
        
        self.odometry.pose.pose.position.x =x
        self.odometry.pose.pose.position.y =y
        self.odometry.pose.pose.position.z =0.0
        
        self.odometry.pose.pose.orientation = self.quaternion_from_euler(0,0,theta)
    
        self.odometry.twist.twist.linear.x = v
        self.odometry.twist.twist.linear.y = 0.0
        self.odometry.twist.twist.linear.z = 0.0
        self.odometry.twist.twist.angular.x = 0.0
        self.odometry.twist.twist.angular.y = 0.0
        self.odometry.twist.twist.angular.z = angleVel
        self.odometry.header.stamp = current_time
        self.odom_pub.publish(self.odometry)
        
        self.odom_trans.header.stamp = current_time
        self.odom_trans.transform.translation.x = self.odometry.pose.pose.position.x
        self.odom_trans.transform.translation.y = self.odometry.pose.pose.position.y
        self.odom_trans.transform.translation.z = self.odometry.pose.pose.position.z 
        self.odom_trans.transform.rotation = self.odometry.pose.pose.orientation
        #self.get_logger().info('Publishing: "%s"' % self.odometry)
        #print('')
        self.broadcaster.sendTransform(self.odom_trans)
        #with open('/home/jose/pose.dat', 'a', encoding='utf8') as log:
        #    dat = str(round(self.kobuki.poseX,5))+','+ str(round(self.kobuki.poseY,5))+','+ str(round(self.kobuki.linearVel,5)) + '\n'
        #    log.write(dat)

    def velCallback(self,msg):
        # self.get_logger().info('getCmdVel {}'.format(time.time()))
        self.robot.SetMoveControl(msg.linear.x, msg.angular.z)
        
      
    
def main(args=None):
    rclpy.init(args=args)
    
    robot_node = CRobotNode(0.02)
    
    rclpy.spin(robot_node)

    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
