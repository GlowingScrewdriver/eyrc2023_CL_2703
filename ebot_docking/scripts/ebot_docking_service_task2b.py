#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

HWArena = __name__ == "ebot_docking_service_hw"

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ebot_docking.srv import DockSw  # Import custom service message
from scipy.spatial.transform import Rotation as R
from threading import Thread
from math import pi
if HWArena: from std_msgs.msg import Float32MultiArray, Float32
else: from sensor_msgs.msg import Range, Imu

# Define a class for your ROS2 node
class RobotDockingController(Node):

    def __init__(self):
        super().__init__('robot_docking_controller')

        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to ultrasonic sensor data for distance measurements
        self.range_right = None; self.range_left = None; self.orientation = None
        if HWArena:
            self.ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback_hw, 10)
            self.imu_sub = self.create_subscription(Float32, '/orientation', self.imu_callback_hw, 10)
        else:
            self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
            self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)
            self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.cmd_vel_pub = self.create_publisher (Twist, '/cmd_vel', 10)

        # Initialize a timer for the main control loop
        self.docking = False
        self.aligned = False
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

        self.spin_dir = 0 # =1 for spin in positive z-axis

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

    ### Topic/service callbacks, Gazebo ###
    def ultrasonic_rl_callback(self, msg):
        self.range_left = msg.range
    def ultrasonic_rr_callback(self, msg):
        self.range_right = msg.range

    def imu_callback (self, msg):
        self.orientation = R.from_quat ([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]).as_euler ('zxy')[0]
    ###

    ### Topic callbacks, hardware arena ###
    def ultra_callback_hw (self, msg):
        self.range_left = msg.data[4]
        self.range_right = msg.data[5]

    def imu_callback_hw (self, msg):
        self.orientation = msg.data - pi # So that -pi <= angle <= pi
    ###

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
            self.spin (spin_dir * 0.05)
            if spin_dir + self.spin_dir == 0: # Robot has just passed the angle in which it faces the rack
                self.spin (0.0)
                self.aligned = True
                return
            self.spin (spin_dir * 0.1)
            return

        # This is reached if docking is in progress and alignment has been achieved
        if self.range_left < 0.1:
            self.docking = False
            self.backup (0.0)
            return
        self.backup (0.2)

    # Callback function for the DockControl service
    if False:
     def dock_control_callback(self, request, response):
        self.docking = True
        while self.docking:
            print ('Docking...')
            self.get_clock().sleep_for (rclpy.time.Duration(seconds = 0.5))

        print ('Done docking')
        response.success = True
        return response

    else:
     def dock_control_callback (self, request, response):
        while self.orientation is None:
            self.get_clock().sleep_for (rclpy.time.Duration(seconds = 0.5))

        #angle_diff = request.orientation - self.orientation
        #while (abs(angle_diff) > 0.05):
        Aligned = False
        Driven = False
        AngleTolerance = 0.05 # Angular tolerance for alignment of the eBot with rack
        DistanceTolerance = 0.1 # Distance tolerance for eBot to be touching the rack
        while True:
            if not Aligned: # Robot is not aligned
                print ("Aligning...")
                angle_diff = request.orientation - self.orientation
                if angle_diff > 0: spin_dir = 1
                else: spin_dir = -1
                #if abs(angle_diff) < AngleTolerance:
                if not spin_dir + self.spin_dir: # Robot has just passed 0 angle difference
                    self.spin (0.0)
                    Aligned = True
                    continue

                self.spin (spin_dir * 0.1)

            elif not Driven: # Robot is aligned, but has not driven to the rack
                print ("Driving...")
                if self.range_left < DistanceTolerance:
                    self.backup (0.0)
                    Driven = True
                    continue

                self.backup (0.2)

            else: # All done, we just have to end the service request
                print ("Done")
                response.success = True
                return response

# Main function to initialize the ROS2 node and spin the executor
if __name__ == '__main__':
    rclpy.init()

    docking_controller = RobotDockingController()

    # Client for sending messages to the link attacher service needs to be on
    # a different node, to prevent deadlocks with the BasicNavigator's processes
    executor = MultiThreadedExecutor()
    executor.add_node(docking_controller)

    docking_controller.declare_parameter ("test_dock", False)
    print (docking_controller.get_parameter("test_dock").get_parameter_value().bool_value)
    if docking_controller.get_parameter("test_dock").get_parameter_value().bool_value:
        th = Thread (target = executor.spin)
        th.start ()
        print (docking_controller.dock_control_callback (
            DockSw.Request (orientation = -1.57),
            DockSw.Response ()
        ))
        executor.shutdown ()
        exit ()

    executor.spin()

    docking_controller.destroy_node()
    rclpy.shutdown()

