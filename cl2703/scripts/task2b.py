#!/usr/bin/python3

# ============================================================
# Team ID:          CL#2703
# Theme:            Cosmo Logistic
# Author List:		Vedanth Padmaraman, Vamshi Vishruth
# Filename:		    task2b.py
#
# Functions:        pretty_print_pose, normalize_angle
# Classes:          RackShift
# ============================================================

from task1c import Navigator
from geometry_msgs.msg import Pose, PoseStamped, Twist, Polygon, Point32
from nav_msgs.msg import Odometry
from linkattacher_msgs.srv import AttachLink, DetachLink
from ebot_docking.srv import DockSw
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from math import sin, cos, asin, acos, pi
import time, threading, numpy as np
from scipy.spatial.transform import Rotation as R


def pretty_print_pose (pose):
    '''
    Use, if needed, for debugging. Prints the contents of a PoseStamped message

    Arguments:
        pose (PoseStamped):  Pose to print
    '''
    print (f'Position: {pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}')
    print (f'Orientation: {pose.pose.orientation.z}, {pose.pose.orientation.w}')

def normalize_angle (an):
    '''
    Normalize and return angle `an` so that -pi <= an <= pi

    Arguments:
        an (int or float):  Angle in radians

    Example:
        normalize_angle ( (3/2) * pi ) -> -pi/2
    '''
    while (abs(an)) > pi:
        an -= 2*pi * abs(an)/an
    return an



class RackShift (Node):
    '''
    Class to simplify navigation.

    This class serves as a wrapper around class Navigator from task1c, and provides
    convenience functions for shifting racks
    '''

    def __init__(self):
        Node.__init__(self, 'rackshift')

        self.callback_group = ReentrantCallbackGroup ()
        # Docking on and picking up a rack
        self.dock_cli   = self.create_client (DockSw, '/dock_control', callback_group=self.callback_group)
        self.dock_req   = DockSw.Request ()
        self.attach_cli = self.create_client (AttachLink, '/ATTACH_LINK', callback_group=self.callback_group)
        self.attach_req = AttachLink.Request (model1_name='ebot', link1_name='ebot_base_link', link2_name='link')
        self.detach_cli = self.create_client (DetachLink, '/DETACH_LINK', callback_group=self.callback_group)
        self.detach_req = DetachLink.Request (model1_name='ebot', link1_name='ebot_base_link', link2_name='link')
        # Velocity commands and odometry
        self.vel_pub    = self.create_publisher (Twist, '/cmd_vel', 10)
        self.robot_pose = None
        self.create_subscription (Odometry, '/odom', self.odom_cb, 10, callback_group=self.callback_group)
        # Footprint updates
        self.fp_pub = self.create_publisher (Polygon, '/local_costmap/footprint', 10)
        self.ebot_fp = Polygon (points = [ # Robot's footprint
            Point32 (x = 0.21, y = 0.195),
            Point32 (x = 0.21, y = -0.195),
            Point32 (x = -0.21, y =  -0.195),
            Point32 (x = -0.21, y = 0.195),
        ])
        self.ebot_rack_fp = Polygon (points = [ # Robot's footprint with rack on. Approximated as a 
            Point32 (x = 0.21, y = 0.45),          # rectangle with y equal to rack width and x equal to that of ebot
            Point32 (x = 0.21, y = -0.45),
            Point32 (x = -0.21, y =  -0.45),
            Point32 (x = -0.21, y = 0.45),
        ])

        self.navigator = Navigator ()
        self.navigator.waitUntilNav2Active ()

    def odom_cb (self, msg):
        '''
        Callback for odometry data. Used for fine-tuning position after navigation by adjust_pose
        '''
        self.robot_pose = PoseStamped (pose=msg.pose.pose, header=msg.header)
        self.robot_pose.header.frame_id = 'map'

    def spin (self, rot=0.0, trans=0.0):
        '''
        Sends velocity commands to the ebot, through the /cmd_vel topic
        Velocities are interpreted in the ebot base frame

        Args:
            rot   (float): rotational velocity about the z-axis, rad/s
            trans (float): translational velocity in the negative x-axis, m/s
        '''
        vel = Twist ()
        vel.angular.z = rot
        vel.linear.x = -trans # Robot moves _backwards_ at positive pace
        self.vel_pub.publish (vel)

    def adjust_pose (self, drop_pose):
        '''
        Fine-tune position after navigation.
        While the Navigator2 stack is critical in planning paths around obstacles, it is rather
        inefficient for fine-grained positioning. This function provides that.

        Args:
            drop_pose (dict): drop position dictionary; refer rack_pose_info['drop'] assignment under __name__ == "__main__" for details
        '''
        loop_interval = 0.1
        while not self.robot_pose:
            time.sleep (0.2)

        spin_dir = 0
        Aligned = False # Is the robot facing the final destination?
        Reached = False # Is the robot at the destination position?
        Done = False    # Is the robot at the destination position and in the correct orientation?
        while True:
            # Position difference vector between the robot and destination
            pos_diff = np.array ([ drop_pose['trans'][0] - self.robot_pose.pose.position.x,
                drop_pose['trans'][1] - self.robot_pose.pose.position.y ])
            cur_angle = R.from_quat ([0.0, 0.0, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w]).as_euler ('xyz')[2]

            goal_angle = acos ( pos_diff[0] / (sum(pos_diff**2)**0.5) )
            # Angle corrections
            if pos_diff[1] < 0: goal_angle *= -1             # Choose between solutions for acos
            goal_angle += pi                                 # Robot faces the opposite direction
            angle_diff = goal_angle - cur_angle
            angle_diff = normalize_angle (angle_diff)

            # Alignment with goal
            if not Aligned:
                # To keep track and detect when angle difference changes sign
                if angle_diff < 0: spin_dir_n = -1
                else: spin_dir_n = 1
                if spin_dir + spin_dir_n == 0: # We have just passed required angle
                    # The robot has just passed the required angle
                    # So, spinning for one interval in the opposite direction by {overshot/interval rad/s} should get us pretty close to the final position
                    self.spin (angle_diff / loop_interval)
                    self.get_clock().sleep_for (rclpy.time.Duration(seconds = loop_interval))
                    spin_dir_n = 0
                    Aligned = True

                spin_dir = spin_dir_n
                self.spin (rot = 0.7 * spin_dir)

            # Driving to the goal
            elif not Reached:
                if sum(pos_diff**2) > 0.1**2:
                    self.spin (trans = 0.3)
                else:
                    self.spin ()
                    Reached = True

            # Alignment as per goal spec
            elif not Done:
                goal_angle = drop_pose['rot']
                rot = 0.7
                angle_diff = normalize_angle(goal_angle - cur_angle)
                if abs(angle_diff) < 0.05:
                    rot = 0.0
                    Done = True
                if angle_diff < 0: rot *= -1
                self.spin (rot = rot)

            else:
                return

            self.get_clock().sleep_for (rclpy.time.Duration(seconds = loop_interval))

    def rack_shift (self, rack_pose_info):
        '''
        Pick up a rack from a given position and drop it in front of the UR5 arm

        Args:
            rack_pose_info (dict): pickup and drop position dictionary; refer rack_pose_info assignment under __name__ == "__main__" for details
        '''
        while not self.navigator.robot_pose:
            time.sleep (0.2)
        self.navigator.setInitialPose (self.navigator.robot_pose)

        pose = rack_pose_info

        # Unit vector pointing away from the rack.
        away = np.array ([cos(pose['pickup']['rot']), sin(pose['pickup']['rot'])])
        pickup_pose = pose['pickup'].copy ()
        pickup_pose_pre = pose['pickup'].copy ()

        # Put the ebot some distance in front of the rack.
        pickup_pose['trans'] = 1.1*away + pose['pickup']['trans']
        pickup_pose_pre['trans'] = 0.5*away + pose['pickup']['trans']

        self.navigator.navigate (pickup_pose)
        self.adjust_pose (pickup_pose_pre)
        print (f'Reached rack')

        # Dock and pick up the rack
        self.dock_req.orientation = pickup_pose['rot']
        self.dock_cli.call(self.dock_req)
        #pretty_print_pose(self.navigator.robot_pose)
#        self.attach_req.model2_name = pose['rack']
#        print (self.attach_cli.call (self.attach_req))
        self.rack_grip (pose['rack'], True)
        print (f'Attached rack')

        # Update the footprint to include the edge of the rack
        self.fp_pub.publish (self.ebot_rack_fp)

        # Just in front of the drop pose
        drop_pose = pose['drop'].copy ()
        drop_pose['trans'] = [
            (drop_pose['trans'][0] + cos(drop_pose['rot'])*1.5),
            (drop_pose['trans'][1] + sin(drop_pose['rot'])*1.5),
        ]
        self.navigator.navigate (drop_pose)
        print (f'Reached arm pose')

        self.adjust_pose (pose['drop'])

        # Detach the rack
#        self.detach_req.model2_name = pose['rack']
#        print (self.detach_cli.call (self.detach_req))
        self.rack_grip (pose['rack'], False)
        print (f'Detached rack')

        # Update the footprint
        self.fp_pub.publish (self.ebot_fp)
        self.navigator.navigate (drop_pose)
        #pretty_print_pose(self.navigator.robot_pose)

    def rack_grip (self, rack, state = True):
        '''Pick up the rack'''

        if state == True: req, cli = self.attach_req, self.attach_cli
        else:             req, cli = self.detach_req, self.detach_cli

        req.model2_name = rack
        print (cli.call (req))

if __name__ == "__main__":
    rclpy.init ()

    docker = RackShift ()
    executor = rclpy.executors.MultiThreadedExecutor (2)
    executor.add_node (docker)
    docker_th = threading.Thread (target=executor.spin)
    docker_th.start ()

    rack_pose_info = {
        # Rack pickup and drop poses
        'pickup': {'trans': [1.26, 4.35], 'rot': 3.14}, # Initial position of the rack
        'drop': {'trans': [0.5, -2.455], 'rot': 3.14},  # Drop position of the rack
        'rack': 'rack1',
    }
    # Note: These are NOT the expected robot poses; these are the exact rack poses

    docker.rack_shift (rack_pose_info)

    rclpy.shutdown ()