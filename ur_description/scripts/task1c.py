#!/usr/bin/python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
import tf2_ros
import rclpy
import time

class Navigator (BasicNavigator):
    def __init__ (self):
        BasicNavigator.__init__(self)
        self.robot_pose = None
        self.create_subscription (Odometry, '/odom', self.odom_cb, 10)

    def odom_cb (self, msg):
        self.robot_pose = PoseStamped (pose=msg.pose.pose, header=msg.header)
        self.robot_pose.header.frame_id = 'map'

    def amcl_cb (self, msg):
        self.robot_pose = PoseStamped (pose=msg.pose.pose, header=msg.header)

    def navigate (self, fin_pose):
        while not self.robot_pose:
            print ('waiting for AMCL pose')
            time.sleep (0.5)

        init_pose_msg = self.robot_pose
        fin_pose_msg = self.gen_pose_msg (fin_pose)

        print (f'moving from {init_pose_msg} to {fin_pose_msg}')
        path = self.getPath (init_pose_msg, fin_pose_msg)

        self.followPath (path)
        while not self.isTaskComplete ():
            time.sleep (2)

        if self.getResult() == TaskResult.SUCCEEDED:
            print ('Complete!')
        else:
            print ('DNF :/')

    def gen_pose_msg (self, pose):
        print ('generating for:', pose)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = pose['trans'][0]
        goal.pose.position.y = pose['trans'][1]
        goal.pose.orientation.z = pose['rot']
        goal.pose.orientation.w = 1.0

        return goal

if __name__ == '__main__':
    rclpy.init()

    navigator = Navigator ()
    navigator.waitUntilNav2Active ()

    pose_list = [
        # Each element here is one of the required poses for Task 1C
        #{ 'trans': [0.0,  0.0],  'rot':  1.55 },
        #{ 'trans': [ 1.8,  1.5],  'rot':  -1.57 },
        #{ 'trans': [ 2.0, -7.0],  'rot': -1.57 },
        #{ 'trans': [-3.0,  2.5],  'rot':  1.57 },
    ]

    init_pose_msg = PoseStamped ()
    for pose in pose_list:
        navigator.navigate (pose)

    rclpy.shutdown ()