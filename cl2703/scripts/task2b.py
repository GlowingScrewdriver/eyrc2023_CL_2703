#!/usr/bin/python3
from task1c import Navigator
from geometry_msgs.msg import Pose, PoseStamped, Twist
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
    print (f'Position: {pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}')
    print (f'Orientation: {pose.pose.orientation.z}, {pose.pose.orientation.w}')

class RackShift (Node):
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

        self.navigator = Navigator ()
        self.navigator.waitUntilNav2Active ()

    def odom_cb (self, msg):
        self.robot_pose = PoseStamped (pose=msg.pose.pose, header=msg.header)
        self.robot_pose.header.frame_id = 'map'

    def spin (self, rot=0.0, trans=0.0):
        vel = Twist ()
        vel.angular.z = rot
        vel.linear.x = -trans # Robot moves _backwards_ at positive pace
        self.vel_pub.publish (vel)


    def adjust_pose (self, drop_pose):
        loop_interval = 0.1
        while not self.robot_pose:
            time.sleep (0.2)

        spin_dir = 0
        Aligned = False
        Reached = False
        Done = False
        # The following loop exits when the robot passes the desired angle
        while True:
            pos_diff = np.array ([ drop_pose['trans'][0] - self.robot_pose.pose.position.x,
                drop_pose['trans'][1] - self.robot_pose.pose.position.y ])
            cur_angle = R.from_quat ([0.0, 0.0, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w]).as_euler ('xyz')[2]

            # Alignment with goal
            if not Aligned:
                goal_angle = acos ( pos_diff[0] / (sum(pos_diff**2)**0.5) )
                # Angle corrections
                if pos_diff[1] < 0: goal_angle *= -1             # Choose between solutions for acos
                goal_angle += pi                                 # Robot faces the opposite direction

                angle_diff = goal_angle - cur_angle
                while abs(angle_diff) > pi: angle_diff -= 2*pi   # Normalize to -pi <= angle <= +pi            

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
                    self.spin (trans = 0.5)
                else:
                    self.spin ()
                    Reached = True

            # Alignment as per goal spec
            elif not Done:
                goal_angle = drop_pose['rot']
                rot = 0.7
                angle_diff = goal_angle - cur_angle
                if abs(angle_diff) < 0.05:
                    rot = 0.0
                    Done = True
                if angle_diff < 0: rot *= -1
                self.spin (rot = rot)

            else:
                return

            self.get_clock().sleep_for (rclpy.time.Duration(seconds = loop_interval))

    def rack_shift (self, rack_pose_info):
        while not self.navigator.robot_pose:
            time.sleep (2)
        self.navigator.setInitialPose (self.navigator.robot_pose)

        pose = rack_pose_info
        print (f'Starting at {self.navigator.get_clock().now()}')

        # Navigate to a point a little away from the
        pickup_pose = pose['pickup']
        pickup_pose['trans'] = [
            (pickup_pose['trans'][0] + cos(pickup_pose['rot'])*0.5),
            (pickup_pose['trans'][1] + sin(pickup_pose['rot'])*0.5),
        ]
        self.navigator.navigate (pickup_pose)
        print (f'Reached rack')

        # Dock and pick up the rack
        self.dock_req.orientation = pickup_pose['rot']
        self.dock_cli.call(self.dock_req)
        print (f'Reached rack')
        pretty_print_pose(self.navigator.robot_pose)
        self.attach_req.model2_name = pose['rack']
        print (self.attach_cli.call (self.attach_req))
        print (f'Attached rack')

        # Navigate, again, half a metre away from desired pose
        drop_pose = pose['drop'].copy ()
        drop_pose['trans'] = [
            (drop_pose['trans'][0] + cos(drop_pose['rot'])*0.5),
            (drop_pose['trans'][1] + sin(drop_pose['rot'])*0.5),
        ]
        self.navigator.navigate (drop_pose)
        print (f'Reached arm pose')

        self.adjust_pose (pose['drop'])

        # Detach the rack
        self.detach_req.model2_name = pose['rack']
        print (self.detach_cli.call (self.detach_req))
        print (f'Detached rack')
        pretty_print_pose(self.navigator.robot_pose)

        # Back to home pose
        #self.navigator.navigate ({'trans': [0.0, 0.0], 'rot': 0.0})
        #self.adjust_pose ({'trans': [0.0, 0.0], 'rot': 0.0})

        print (f'Finishing at {self.navigator.get_clock().now()}')

if __name__ == "__main__":
    rclpy.init ()

    docker = RackShift ()
    executor = rclpy.executors.MultiThreadedExecutor (2)
    executor.add_node (docker)
    docker_th = threading.Thread (target=executor.spin)
    docker_th.start ()

    rack_pose_info = {
        # Rack pickup and drop poses
            'pickup': {'trans': [1.26, 4.35], 'rot': 3.14},
            'drop': {'trans': [0.5, -2.455], 'rot': 3.14},
            'rack': 'rack1',
    }
    # Note: These are NOT the expected robot poses; these are the exact rack poses

    docker.rack_shift (rack_pose_info)

    rclpy.shutdown ()