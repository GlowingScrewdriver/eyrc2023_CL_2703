#!/usr/bin/python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import time, math

def gen_pose_msg (pose, time_msg):
    print ('generating for:', pose)
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = time_msg
    goal.pose.position.x = pose['trans'][0]
    goal.pose.position.y = pose['trans'][1]
    goal.pose.orientation.z = pose['rot']
    goal.pose.orientation.w = 1.0

    return goal

def navigate (init_pose, fin_pose):
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    now = navigator.get_clock().now().to_msg()
    init_pose_msg, fin_pose_msg = gen_pose_msg (init_pose, now), gen_pose_msg (fin_pose, now)
    path = navigator.getPath (init_pose_msg, fin_pose_msg)
    #print (path)

    print (f'moving from {init_pose} to {fin_pose}')
    navigator.followPath (path)

    #goal = gen_pose_msg (fin_pose)

    #navigator.goToPose (goal)
    #waitCount = 0
    while not navigator.isTaskComplete ():
        print ('Nav not complete, waiting')
        time.sleep (2)

    if navigator.getResult() == TaskResult.SUCCEEDED:
        print ('Complete!')
    else:
        print ('DNF :/')


if __name__ == '__main__':
    rclpy.init()

    init_pose = { 'trans': [0.0,  0.0],  'rot':  0.0 }
    pose_list = [
        # Each element here is one of the required poses for Task 1C
        #{ 'trans': [0.0,  0.0],  'rot':  0.0 },
        { 'trans': [ 1.8,  1.5],  'rot':  1.57 },
        #{ 'trans': [ 2.0, -7.0],  'rot': -1.57 },
        #{ 'trans': [-3.0,  2.5],  'rot':  1.57 },
    ]
    for pose in pose_list:
        navigate (init_pose, pose)
        #navigator.setInitialPose (init_pose)
        #path = navigator.getPath (init_pose, pose)
        #print (path)
        init_pose = pose

    print ('bye')
    rclpy.shutdown ()