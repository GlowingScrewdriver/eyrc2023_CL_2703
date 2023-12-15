#!/usr/bin/python3

# Team ID:          CL#2703
# Theme:            Cosmo Logistic
# Author List:		Vedanth Padmaraman, Vamshi Vishruth
# Filename:		    task2a.py
#
# Functions:        __init__(),attach_box_with_gripper(),detach_box_from_gripper(),pick_and_drop(box_i),main()
# Classes:          PIckAndDrop(Move)
# Globals:          

################### IMPORT MODULES #######################


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


class PickAndDrop (Move):
    def __init__(self, name):
        """
        Initialize the PickAndDrop class, inheriting from Move.

        Creates clients for the AttachLink and DetachLink services for gripper control.
        """
        callback_group = ReentrantCallbackGroup ()
        Move.__init__(self, name)
        # Gripper control
        self.client1 = self.create_client(AttachLink, '/GripperMagnetON', callback_group=callback_group)
        self.client2 = self.create_client(DetachLink, '/GripperMagnetOFF', callback_group=callback_group)

    # Attaching the box to the gripper
    def attach_box_with_gripper(self):
        """
        Attach the box to the gripper using the AttachLink service.
        """
        req = AttachLink.Request()
        req.model1_name=f'box{self.box_ids[0]}'
        req.link1_name='link'
        req.model2_name='ur5'
        req.link2_name='wrist_3_link'
        future = self.client1.call_async(req)

        while not future.done ():
            time.sleep (0.5)

        if future.result() is not None:
            self.get_logger().info('GripperMagnetON service call was successful')
        else:
            self.get_logger().info('GripperMagnetON service call failed')

    # Detaching the box from the gripper
    def detach_box_from_gripper(self):
        """
        Detach the box from the gripper using the DetachLink service.
        """
        req = DetachLink.Request()
        req.model1_name=f'box{self.box_ids.pop(0)}'
        req.link1_name='link'
        req.model2_name='ur5'
        req.link2_name='wrist_3_link'
        future = self.client2.call_async(req)

        while not future.done ():
            time.sleep (0.5)


        if future.result() is not None:
            self.get_logger().info('GripperMagnetOFF service call was successful')
        else:
            self.get_logger().info('GripperMagnetOFF service call failed')

    def pick_and_drop (self, box_ids):
        """
        Pick and drop the specified boxes using gripper control and MoveIt motion.

        Args:
            box_ids (list): List of box IDs to pick and drop.
        """
        self.box_ids = box_ids
        # The drop pose, that must be visited after every box
        box_offset = 0.3    # Box dimensions are (0.16) * (0.22*0.24), as (depth) * (marker face)
        drop_pose = {
            'position': np.array([-0.55, -box_offset, 0.2]), # Leftmost box position; `box_offset`m to the left of the Task1B drop position
            'shoulder': -math.pi,
            'retract': False,
            'callback': self.detach_box_from_gripper,
        }
        self.destinations = []
        for box in box_ids:
            tf = None
            while tf is None:
	            try:
	            	tf = self.tf_buffer.lookup_transform('base_link', f'obj_{box}', rclpy.time.Time())
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


            dest = {
                'position': np.array ([p_x, p_y, p_z]),
                'shoulder': p_rot,
                'retract': True,
                'callback': self.attach_box_with_gripper,
            }
            # Make a copy of the drop pose and add the box offset to the original drop pose dict
            drop_pose_adj = drop_pose.copy ()
            drop_pose_adj['position'] = drop_pose_adj['position'].copy ()
            drop_pose['position'][1] += box_offset
            self.destinations += [dest, drop_pose_adj]

if __name__ == "__main__":
    rclpy.init ()
    executor = rclpy.executors.MultiThreadedExecutor (3)

    # Our main node and interface to MoveIt
    node = PickAndDrop ("arm_control")
    executor.add_node (node)

    th = threading.Thread (target = executor.spin)
    th.start ()

    node.pick_and_drop ([1])
    node.motion ()

    rclpy.shutdown ()
    exit ()
