#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics, time

# Define a class for your ROS2 node
class RobotDockingController(Node):

    def __init__(self):
        super().__init__('robot_docking_controller')

        self.callback_group = ReentrantCallbackGroup()

        self.robot_pose = None; self.odom_pose = None; self.yaw = None
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.range_right = None; self.range_left = None
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.cmd_vel_pub = self.create_publisher (Twist, '/cmd_vel', 10)

        # Initialize a timer for the main control loop
        self.docking = False
        self.aligned = False
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

        self.spin_dir = 0 # =1 for spin in positive z-axis

    # Callback function for odometry data
    def odometry_callback(self, msg):
#        self.odom_pose = PoseStamped (pose=msg.pose.pose, header=msg.header); self.odom_pose.header.frame_id = 'map'
#        if not self.robot_pose: self.robot_pose = [0.0, 0.0, 0.0]
#        # Extract and update robot pose information from odometry message
#        self.robot_pose[0] = msg.pose.pose.position.x
#        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
#        self.robot_pose[2] = yaw
        self.robot_yaw = yaw

    def spin (self, rate):
        vel = Twist ()
        vel.angular.z = rate
        self.cmd_vel_pub.publish (vel)
        if rate > 0:
            self.spin_dir = 1
        else:
            self.spin_dir = -1

    def backup (self, rate):
        vel = Twist ()
        vel.linear.x = -rate # The robot moves _backwards_ at a positive pace
        self.cmd_vel_pub.publish (vel)

    def ultrasonic_rl_callback(self, msg):
        self.range_left = msg.range
    def ultrasonic_rr_callback(self, msg):
        self.range_right = msg.range


    def controller_loop(self):
        if not self.docking:
            self.aligned = False
            return

        # This is reached if docking is in progress but alignment hasn't been achieved
        if not self.aligned:
            angle_diff = self.range_left - self.range_right
            #print (angle_diff)

            if angle_diff**2 < 0.001**2:
                self.spin (0.0)
                self.aligned = True
                return

            if angle_diff > 0: spin_dir = 1
            else: spin_dir = -1
            if spin_dir + self.spin_dir == 0: # Robot has just passed the angle in which it faces the rack
                print (f'switched over at range diff {angle_diff}')
                self.spin (angle_diff/5) # angle_diff <= change of angle per loop time interval (0.1s)
                return

#            if angle_diff > 0: spin_dir = 1
#            else: spin_dir = -1
#            self.spin (spin_dir * 0.05)
#            if spin_dir + self.spin_dir == 0: # Robot has just passed the angle in which it faces the rack
#                self.spin (0.0)
#                self.aligned = True
#                return
#            self.spin (spin_dir * 0.1)
#            return

        # This is reached if docking is in progress and alignment has been achieved
        if self.range_left < 0.1:
            self.docking = False
            self.backup (0.0)
            return
        self.backup (0.2)

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        #while not self.robot_pose:
        while not self.robot_yaw:
            self.get_clock().sleep_for (rclpy.time.Duration(seconds = 0.5))
            print ('waiting for odom pose')
        #diff = request.orientation - self.robot_pose[2]
        diff = request.orientation - self.robot_yaw
        tolerance = 0.1 # Angle tolerance in radians

        # To meet the spin tolerance limit,
        # (spin rate) * (spin command interval) <= (spin tolerance)
        # We accordingly taper down the rotation speed, so that in the last
        # iteration of setting the speed, the spin rate satifies the above equation
        # In the limiting case (last iteration), angular position difference (`diff`)
        # approaches spin tolerance limit
        while diff**2 > tolerance**2:
            if diff > 0: rot=1
            else: rot=-1
            self.spin (rot * 0.7)
            self.get_clock().sleep_for (rclpy.time.Duration(seconds = 0.1))
            #diff = request.orientation - self.robot_pose[2]
            diff = request.orientation - self.robot_yaw

        self.docking = True
        while self.docking:
            print ('Docking...')
            self.get_clock().sleep_for (rclpy.time.Duration(seconds = 0.5))

        print ('Done docking')
        response.success = True
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    docking_controller = RobotDockingController()

    # Client for sending messages to the link attacher service needs to be on
    # a different node, to prevent deadlocks with the BasicNavigator's processes
    executor = MultiThreadedExecutor()
    executor.add_node(docking_controller)

    executor.spin()

    docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
