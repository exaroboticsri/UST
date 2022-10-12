import time
from enum import Enum

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

import rclpy



class NavigationResult(Enum):
    UKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3
    ACCEPTED = 4
    EXCUTING = 5

    def to_str(result):
        if result >= 0 and result <= 5:
            str = ['UKNOWN', 'SUCCEEDED', 'CANCELED', 'FAILED', 'ACCEPTED', 'EXCUTING']
        else:
            str = 'UKNOWN'

        return str


class CROSNavigator(BasicNavigator):
    def __init__(self):
        rclpy.init()
        super(CROSNavigator, self).__init__()
        self._is_start = False
    
    def wait_nav2_active(self):
        #self.waitUntilNav2Active(localizer = 'None')
        """Block until the full navigation system is up and running."""
       
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        

        self.set_initial_pose()

        self._is_start = True
        

    def set_initial_pose(self, pose=None):
        if pose == None:
            initial_pose = self.make_pose([0., 0.])
        else:
            initial_pose = pose

        self.setInitialPose(initial_pose)

    def make_pose(self, pose_xy: list = [0., 0.], orientation: list = [0., 0., 0., 1.]):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pose_xy[0]
        pose.pose.position.y = pose_xy[1]

        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        return pose

    def go_to_pose(self, pose):
        if self._is_start == True:
            self.goToPose(pose)

    def go_Through_Pose(self, pose):
        if self._is_start == True:
            return super().goThroughPoses(pose)
    
    def go_to_waypoints(self, poses=None):
        if self._is_start == True:
            if poses != None:
                self.followWaypoints(poses)
    def clear_local_costmap(self):
        if self._is_start == True:
            self.clearLocalCostmap()

    def clear_global_cosmapp(self):
        if self._is_start == True:
            self.clearGlobalCostmap()

    def clear_all_costmap(self):
        if self._is_start == True:
            self.clearAllCostmaps()

    def cancel_nav(self):
        if self._is_start == True:
            self.cancelTask()

    def is_nav_comlete(self, print_result=True):
        result = 0
        if self.isTaskComplete():
            result = self.getResult()

        if print_result == True:
            NavigationResult.to_str(result)
        return result



