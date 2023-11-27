#!/usr/bin/python3

import rclpy
from rclpy.task import Future
from task1b import Move
from task1a import aruco_tf
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np
import math, time
import task1b
import threading
from rclpy.callback_groups import ReentrantCallbackGroup

from linkattacher_msgs.srv import AttachLink, DetachLink

# Attaching the box to the gripper
def attach_box_with_gripper():
    global box_ids
    req = AttachLink.Request()
    req.model1_name=box_ids[0]
    req.link1_name='link'
    req.model2_name='ur5'
    req.link2_name='wrist_3_link'
    future = client1.call_async(req)

    while not future.done ():
        time.sleep (0.5)

    if future.result() is not None:
        node.get_logger().info('GripperMagnetON service call was successful')
    else:
        node.get_logger().info('GripperMagnetON service call failed')

# Detaching the box from the gripper
def detach_box_from_gripper():
    global box_ids
    req = DetachLink.Request()
    req.model1_name=box_ids.pop(0)
    req.link1_name='link'
    req.model2_name='ur5'
    req.link2_name='wrist_3_link'
    future = client2.call_async(req)

    while not future.done ():
        time.sleep (0.5)


    if future.result() is not None:
        node.get_logger().info('GripperMagnetOFF service call was successful')
    else:
        node.get_logger().info('GripperMagnetOFF service call failed')

if __name__ == "__main__":
    rclpy.init ()
    executor = rclpy.executors.MultiThreadedExecutor (2)
    callback_group = ReentrantCallbackGroup ()

    # Our main node and interface to MoveIt
    node = Move("node")
    executor.add_node (node)
    # Gripper control
    client1 = node.create_client(AttachLink, '/GripperMagnetON', callback_group=callback_group)
    client2 = node.create_client(DetachLink, '/GripperMagnetOFF', callback_group=callback_group)

    # Node just for listening to TFs, spun in the background
    tf_executor = rclpy.executors.MultiThreadedExecutor (3)
    tf_node = rclpy.create_node ('tf_listener')
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_node.listener = tf2_ros.TransformListener(tf_buffer, tf_node)
    tf_executor.add_node (tf_node)
    tf_th = threading.Thread (target = tf_executor.spin)
    tf_th.start ()

    # The drop pose, that must be visited after every box
    box_offset = 0.3    # Box dimensions are (0.16) * (0.22*0.24), as (depth) * (marker face)
    drop_pose = {
        'position': np.array([-0.5, -box_offset, 0.2]), # Leftmost box position; `box_offset`m to the left of the Task1B drop position
        'shoulder': -math.pi,
        'retract': False,
        'callback': detach_box_from_gripper,
    }
    node.destinations = []
    box_ids = ['box1', 'box3', 'box49']
    for box in (1, 3, 49):
        tf = None
        while tf is None:
	        try:
	        	tf = tf_buffer.lookup_transform('base_link', f'obj_{box}', rclpy.time.Time())
	        except tf2_ros.LookupException as e:
	            print (f'marker {box} transform not found: {e}')
	            time.sleep (0.5)

        p_x=tf.transform.translation.x
        p_y=tf.transform.translation.y
        p_z=tf.transform.translation.z

        p_quat = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
        # Starting from a box aligned with the base_frame,
        # the first two Y-Z-Y rotations put the box's marker on a vertical plane
        # Thus, the third rotation corresponds to the required shoulder angle
        p_rot = R.from_quat(p_quat).as_euler ('YZY')[2]

        #if True:
        if False:
            print (p_rot)
            rclpy.shutdown ()
            exit ()

        dest = {
            'position': np.array ([p_x, p_y, p_z]),
            'shoulder': p_rot,
            'retract': True,
            'callback': attach_box_with_gripper,
        }
        # Make a copy of the drop pose and add the box offset to the original drop pose dict
        drop_pose_adj = drop_pose.copy ()
        drop_pose_adj['position'] = drop_pose_adj['position'].copy ()
        drop_pose['position'][1] += box_offset
        node.destinations += [dest, drop_pose_adj]
    tf_executor.shutdown ()


#    node.destinations = [
#        {
#            'position': np.array([p1_x,p1_y,p1_z]),
#            'shoulder': p1_rot,
#            'retract': True,
#            'callback': attach_box_with_gripper,
#        },
#        {
#            'position': np.array([-0.37, 0.12, 0.397]),  #DROP POSITON DETACH
#            'shoulder': -math.pi, 
#            'retract': False,
#            'callback': detach_box_from_gripper ,
#        },
#    ]

    try:
        executor.spin ()
    except Move.MotionEndedException:
        print ('Motion ended')

    rclpy.shutdown ()
