#!/usr/bin/env python3

# Team ID:          CL#2703
# Theme:            Cosmo Logistic
# Author List:		Vedanth Padmaraman, Vamshi Vishruth
# Filename:		    task1b.py
#
# Functions:        motion(),pose_goal(shoulder,retract),__init__(name),dest_callback(),main()
# Classes:          Move(Node) 
# Globals:          no_tf, detector, params

import numpy as np
from math import cos, sin
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

class Move(Node):
    def motion(self):
        """
        Perform the servo motion logic for reaching the specified targets.
        This moves the arm based on position info in self.destinations. That must
        be populated before running this.

        Refer self.destinations assignment (commented out) under __init__
        """

        CbPending = False # This flag determines when the destination callback is called
        while True:
            # If target is reached
            if CbPending:
                self.dest['callback'] ()
                if self.dest['retract']: # We may not need to retract after reaching some positions
                    # Move 0.2 metres away from the box
                    self.dest['position'][0] -= cos (self.dest['shoulder']) * 0.13
                    self.dest['position'][1] -= sin (self.dest['shoulder']) * 0.13
                    self.dest['retract'] = False
                    self.dest['callback'] = lambda: None
                    CbPending = False
                    continue
                self.dest = None
                CbPending = False

            # What to do if the previous target has been reached
            if not self.dest:
                print (self.box_ids)
                if not self.destinations:
                    return
                self.dest = self.destinations.pop(0)
                self.pose_goal (self.dest['shoulder'])
                if self.dest['position'] is None:
                    self.dest = None
                    continue
                print (f'Moving to position {self.dest}')


            # What follows is the servo logic, carried out after calling pose_goal() for each target
            now = self.get_clock().now()
            try:
                base_eef_tf = self.tf_buffer.lookup_transform (
                    'base_link', 'wrist_3_link',
                    #'base_link', 'ee_link',
                    rclpy.time.Time()
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
                print (e)
                continue

            eef_pos = np.array ([
                base_eef_tf.transform.translation.x,
                base_eef_tf.transform.translation.y,
                base_eef_tf.transform.translation.z,
            ])

            dest_pos = self.dest['position'] - eef_pos
            dist = sum(dest_pos**2)**0.5

            tolerance = 0.12
            if (dist < tolerance): # Endpoint for a servo motion (i.e. a target is reached)
                # Stop the servo motion and set the callback-pending flag
                dest_pos *= 0
                print ('Reached destination')
                CbPending = True
            if dist < tolerance * 1.5: vel = 0.2
            else: vel = 0.5
            dest_vel = (dest_pos / dist) * vel # Servo motion at `vel` metres/second

            self.__twist_msg.header.stamp = now.to_msg()
            self.__twist_msg.header.frame_id = 'base_link'
            self.__twist_msg.twist.linear.x = dest_vel[0]
            self.__twist_msg.twist.linear.y = dest_vel[1]
            self.__twist_msg.twist.linear.z = dest_vel[2]
            self.__twist_msg.twist.angular.x = 0.0
            self.__twist_msg.twist.angular.y = 0.0
            self.__twist_msg.twist.angular.z = 0.0

            self.__twist_pub.publish (self.__twist_msg)

    def pose_goal (self, shoulder, retract=False):
        """
        Move the robot to the specified joint configuration.

        Args:
            shoulder (float): Desired position for the shoulder joint.
            retract (bool): Whether to retract the arm after reaching the goal position.
        """
        
        print (f"Moving to shoulder {shoulder}")
        if not retract:
            joint_poses = [shoulder, -2.269, 2.164, -3.023, -1.58, 3.15]
        else:
            joint_poses = [shoulder, -2.269, 2.164, -4.71,  -1.58, 3.15]
        self.moveit2.move_to_configuration(
            joint_poses
        )
        self.moveit2.wait_until_executed()

    def __init__(self, name):
        """
        Initialize the Move class.

        Args:
            name (str): Name of the ROS node.
        """
        
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

#        self.destinations = [
#            # Each of these targets is reached by:
#            # * setting the joint states to put the arm in a somewhat crouched position
#            # * rotating the shoulder to make the arm EEF face the general direction of the target
#            #   (this and the previous step are executed at once)
#            # * servoing to reach the exact position of the target
#            # * optionally retracting back to the previous crouched position
#
#            # Each of the pick positions (P1 and P2) and the drop position (D) is a target here
#
#             {
#                'position': np.array([0.35, 0.1, 0.68]),
#                'shoulder': 0.0,
#                'retract': True,
#                'callback': dest_callback,
#             },
#             {
#                'position': np.array([-0.37, 0.12, 0.397] ),
#                'shoulder': -math.pi,
#                'retract': True,
#                'callback': dest_callback,
#             },
#             {
#                'position': np.array([0.194, -0.43, 0.701]),
#                'shoulder': -math.pi/2,
#                'retract': True,
#                'callback': dest_callback,
#             },
#        ]
        self.destinations = []
        self.dest = None

        # Control flow goes something like:
        # 0. Initially, `dest_list` contains the full list of targets desired and `dest` is set to `None`
        # 1. First target from `dest_list` is popped and assigned to `dest` (this is the "current target")
        # 2. Joints are set to put the arm in the crouch position for current target
        # 3. EFF is servo'd to reach the position for current target
        # 4. If needed, arm is again put in the crouch position (retracted)
        # 5. `dest` is set to `None` (this triggers the motion to the next target, refer first few lines of function `motion`
        # 6. Repeat from 1.

def dest_callback ():
    """
    Default callback function when reaching the destination.
    """
    print ('This is the default callback on reaching the destination')

def main():
    """
    Main function to initialize and run the ROS node.
    """
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor(3)
    # Create node for this example
    node = Move("ex_servo")
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
