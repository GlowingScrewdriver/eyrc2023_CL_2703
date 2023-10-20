#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
`ros2 run pymoveit2 ex_servo.py`
"""


from math import cos, sin
import math, time
import numpy as np
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist, TransformStamped
from pymoveit2 import MoveIt2
from threading import Thread
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class Move(Node):
    class MotionEndedException (Exception):
        pass

    def motion(self):
        # What to do if the previous target has been reached
        if not self.dest:
            if not self.dest_list:
                print ('Done!')
                raise self.MotionEndedException ('Motion ended')
                return
            self.dest = self.dest_list.pop(0)
            self.pose_goal (self.destinations[self.dest]['shoulder'])
            print (f'Moving to position {self.dest}')

        # What follows is the servo logic, carried out after calling pose_goal() for each target
        now = self.get_clock().now()
        try:
            base_eef_tf = self.tf_buffer.lookup_transform (
                'base_link', f'tool0',
                rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            print (e)
            return

        eef_pos = np.array ([
            base_eef_tf.transform.translation.x,
            base_eef_tf.transform.translation.y,
            base_eef_tf.transform.translation.z,
        ])

        dest_pos = self.destinations[self.dest]['position'] - eef_pos 
        dist = sum(dest_pos**2)**0.5


        if (dist < 0.01): # Endpoint for a servo motion (i.e. a target is reached)
            print ('Reached destination')
            if self.destinations[self.dest]['retract']: # We may not need to retract after reaching some positions
                time.sleep (0.5) # Else the planner complains saying the arm pose does not match the expected value
                self.pose_goal (self.destinations[self.dest]['shoulder'])
            self.dest = None
        dest_vel = (dest_pos / dist) * 0.4 # Servo motion at 0.4 metres/second

        self.__twist_msg.header.stamp = now.to_msg()
        self.__twist_msg.twist.linear.x = dest_vel[0]
        self.__twist_msg.twist.linear.y = dest_vel[1]
        self.__twist_msg.twist.linear.z = dest_vel[2]
        self.__twist_msg.twist.angular.x = 0.0
        self.__twist_msg.twist.angular.y = 0.0
        self.__twist_msg.twist.angular.z = 0.0

        self.__twist_pub.publish (self.__twist_msg)

    def pose_goal (self, shoulder):
        print (f"Moving to shoulder {shoulder}")
        self.moveit2.move_to_configuration(
            [shoulder, -2.1, 2.1, -3.15, -1.58, 3.15]
        )
        self.moveit2.wait_until_executed()

    def __init__(self, name):
        super().__init__(name)

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.__twist_msg = TwistStamped ()
        self.__twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)


        # Create MoveIt 2 interface
        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        self.destinations = {
            # Each of these targets is reached by:
            # * setting the joint states to put the arm in a somewhat crouched position
            # * rotating the shoulder to make the arm EEF face the general direction of the target
            #   (this and the previous step are executed at once)
            # * servoing to reach the exact position of the target
            # * optionally retracting back to the previous crouched position

            # Each of the pick positions (P1 and P2) and the drop position (D) is a target here

             'p1': {
                'position': np.array([0.35, 0.1, 0.68]),
                'shoulder': 0.0,
                'retract': True,
             },
             'd': {
                'position': np.array([-0.37, 0.12, 0.397] ),
                'shoulder': -math.pi,
                'retract': True,
             },
             'p2': {
                'position': np.array([0.194, -0.43, 0.701]),
                'shoulder': -math.pi/2,
                'retract': True,
             },
        }
        self.declare_parameter ('targets', ['p1', 'd', 'p2', 'd']) # Task 1B requirements
        self.dest_list = self.get_parameter("targets").get_parameter_value().string_array_value
        self.dest = None

        self.create_timer (0.02, self.motion)
        # Control flow goes something like:
        # 0. Initially, `dest_list` contains the full list of targets desired and `dest` is set to `None`
        # 1. First target from `dest_list` is popped and assigned to `dest` (this is the "current target")
        # 2. Joints are set to put the arm in the crouch position for current target
        # 3. EFF is servo'd to reach the position for current target
        # 4. If needed, arm is again put in the crouch position (retracted)
        # 5. `dest` is set to `None` (this triggers the motion to the next target, refer first few lines of function `motion`
        # 6. Repeat from 1.


def main():
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor(3)
    # Create node for this example
    node = Move("ex_servo")
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
