#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from threading import Thread
from time import sleep
from math import sin, cos, pi, atan
from numpy import ndarray, zeros, uint8, array
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Range, Imu

### Flags for script behaviour ###
DEBUG = True   # Whether to render the laser scan along with some markers for debugging
ARENA = False  # Enables switching between simulator and hardware with ease

def pos_angle (an):
    """
    Make angle an positive such that 0 < an < 2*pi
    """
    while an < 0:
        an += 2*pi
    while an >= 2*pi:
        an -= 2*pi
    return an

class LaserToImg (Node):
    def __init__ (self, name="laser_to_img"):
        Node.__init__(self, name)
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.lasercb, 10)
        self.cmd_vel_pub = self.create_publisher (Twist, '/cmd_vel', 10)

        if ARENA: # For the hardware arena
            self.imu_sub = self.create_subscription(Float32, '/orientation', self.imu_callback_hw, 10)
            self.ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback_hw, 10)
        else:     # For the simulator
            self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
            self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        

    def imu_callback_hw (self, msg):
        #self.orientation = normalize_angle(msg.data) # So that -pi <= angle <= pi
        self.orientation = pos_angle (msg.data) # So that 0 <= angle < 2pi

    def imu_callback (self, msg):
        an = R.from_quat ([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]).as_euler ('zxy')[0]
        self.orientation = pos_angle (an + 6*pi/4) # Just for some noise

    def ultra_callback_hw (self, msg):
        self.range_left = msg.data[4]
        self.range_right = msg.data[5]

    def ultrasonic_rl_callback(self, msg):
        self.range_left = msg.range

    def lasercb (self, msg):
        self.scan = msg

    def detectcoordinates(self, img):
        coords=[]
        # Detect contours and approximate them to simple shapes
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in range (len (contours)):
            cnt = contours[c]
            coords.append ([])
            approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), False)
            #approx = cv2.approxPolyDP(cnt, 3, False)
            #approx = cv2.approxPolyDP(cnt, 2, True) # The lower the value of threshold, the better
            n = approx.ravel()
            i = 0
            for i in range(len(n)):
                if(i % 2 == 0): 
                    x = n[i] 
                    y = n[i + 1] 
                    coords [c].append((x,y))
        # `coords` is now a list of simplified contours

        # Further simplify each contour by selecting only the longest edge and its
        # two neighbors
        MinEdgeLength = 10
        shapes_fil = []
        for cnt in coords:
            #_cnt = [cnt[0]] # Remove all edges less than a threshold. Probably just corners
            maxlen = 0
            maxidx = 0
            for c in range (1, len (cnt)):
                l = ((cnt[c-1][0] - cnt[c][0])**2 + (cnt[c-1][1] - cnt[c][1])**2)**0.5
                if l > maxlen:
                    maxlen = l
                    maxidx = c
                #if l > MinEdgeLength: _cnt.append (c)
            cnt_fil = cnt[maxidx-2:maxidx+2]
            #cnt_fil = _cnt[maxidx-2:maxidx+2]
            if len (cnt_fil) == 4:
                shapes_fil.append (array (cnt_fil))

        if DEBUG:
            for s in shapes_fil:
                for e in range (1, len (s)):
                    cv2.line (self.im_color, s[e-1], s[e], (0, 0, 255), 3)
        # `shapes_fil` is now a list of contours reduced to 3 edges / 4 corners

        # Identify rack from the contours
        rack_len = array ([40, 80, 40]) # Pixels
        for sh in shapes_fil:
            LenDiffMax = 15 # cm
            # Each shape here is tested as a candidate to be the rack.
            # Shape with the smallest difference in side lengths wins
            diff_max = 0
            sides = []
            for c in range (1, len (sh)):
                #diff = ((sh [c][0] - sh [c-1][0])**2 + (sh [c][1] - sh [c-1][1])**2)**0.5
                sides += [sum ((sh [c] - sh [c-1])**2)**0.5]

            #if max (abs ((rack_len - sides))) < LenDiffMax: # This shape is a good match. Return it
            if abs (rack_len - sides)[1] < LenDiffMax: # This shape is a good match. Return it
                #print (diff )
                return sh
        return None

    def scan_to_matrix (self):
        """
        Convert a laser scan to a CV2 image
        """

        cart = lambda r, t: (r * cos (t), r * sin(t))
        msg = self.scan

        Density = 100  # Pixels per meter
        Window = 4     # For each detected point (a, b), all points within the margin
                       # { (a +-Window, b +-Window) are marked

        # Determine angles of each "slice" of the scan
        start_angle, end_angle, incr_angle = msg.angle_min, msg.angle_max, msg.angle_increment
        max_range = 3.0 # Scan limits
        min_range = msg.range_min
        _max = int (max_range * Density) # Radius of the scan in pixels.
        origin = array ((0, _max)) # Position of the sensor

        # An ndarray holding the image
        #scan_map = zeros ((_max + 1, _max * 2 + 2), dtype=uint8) # Each cell of the image is (1/Density) m^2
        scan_map = zeros ((_max * 2 + 2, _max + 1), dtype=uint8) # Each cell of the image is (1/Density) m^2

        an = start_angle
        for pt in msg.ranges:
            # Each element here corresponds to an angle and
            # represents the distance of the point in that direction
            if pt > max_range or pt < min_range: continue
            x, y = cart (pt, an)
            # The real coordinate system needs to be transformed to the pixel coordinate system
            x = int (x * Density)
            y = int (y * Density) + _max

#            for i in range (-Window, Window + 1):
#                for j in range (-Window, Window + 1):
#                    scan_map [x + i][y + j] = 255
            #scan_map [x-Window:x+Window, y-Window:y+Window] = 255
            scan_map [y-Window:y+Window, x-Window:x+Window] = 255
            an += incr_angle

        scan_map_cv = cv2.Mat (scan_map)
        self.im_color = cv2.cvtColor (scan_map_cv, cv2.COLOR_GRAY2RGB) # Can be used by the other functions for debugging
        return scan_map_cv, origin # scan map, origin

    def dock_advice (self):
        """
        Outputs velocity hints and rack angle to get the ebot to dock
        Used by docking controller
        Returns a tuple of the form (trans speed, rot speed, rack offset angle)
        Note: rack offset is 0 when the rack is perfectly aligned with the ebot
        and ultrasonic sensor faces rack, and increases as ebot rotates clockwise.
        In other words, "how much should the ebot rotate in its own frame so that
        electromagnets point to the rack?"
        """
        img, origin = self.scan_to_matrix ()
        #im_color = cv2.cvtColor (img, cv2.COLOR_GRAY2RGB)
        rack = self.detectcoordinates (img)
        ret = [0.05, 0.0, -pi]
        if rack is not None:
            # Tasks:
            # * Find the further corner
            # * Rotate till trajectory points outside that corner
            # * Move forward
            c1, c2 = rack [1], rack[2]
            if DEBUG:
                    cv2.line (self.im_color, c1, c2, (0, 255, 0), 3)
            c1 -= origin; c2 -= origin
            far = max ((c1, c2), key = lambda c: sum (c**2))
            if far[1] > 0: spin = 1
            else:          spin = -1
            ret [1] = spin * 0.3
            if far[0] < 50: ret [0:2] = [0.0, 0.0] # Endpoint

            offset = atan ((c1[0] - c2[0]) / (c2[1] - c1[1])) + pi
            ret [2] = offset # This should already satisfy 0 < offset < 2pi

            # Debug info
            if DEBUG:
                cv2.circle (self.im_color, origin + far, 3, (0, 0, 255), 3)
                cv2.putText (self.im_color, f"Rack distance: {far[0]}", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
                cv2.putText (self.im_color, f"Rack offset: {offset}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
                cv2.putText (self.im_color, f"IMU reading: {self.orientation}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))

        #cv2.imshow ("Rack", im_color)
        #cv2.waitKey (1)
        return ret
            
    def dock (self, orientation):
        vel = Twist ()
        while True: # Get to the center of rack
            v = self.dock_advice ()
            vel.linear.x, vel.angular.z = v [0:2]
            self.cmd_vel_pub.publish (vel)
            if not v[0]: break # Docking is complete
            cv2.imshow ("Rack view", self.im_color)
            cv2.waitKey (1)

        start_angle = self.orientation # Track the angle difference from start_angle
        target_offset = v [2]          # Aim to reach angle start_angle + target_offset
        while True:
            diff = pos_angle (self.orientation - start_angle)
            print (start_angle, target_offset, diff)
            if diff > target_offset:
              if diff < 3*pi/2: # To guard against an IMU reading of ~2pi initially
                vel.angular.z = 0.0
                self.cmd_vel_pub.publish (vel)
                break
            vel.angular.z = 0.5
            self.cmd_vel_pub.publish (vel)

        while True: # Drive to rack
            #if self.range_left < 18.0:
            if self.range_left < 0.05:
                sleep (2)
                vel.linear.x = 0.0
                self.cmd_vel_pub.publish (vel)
                print ("Ready to attach rack")
                break
            vel.linear.x = -0.05
            self.cmd_vel_pub.publish (vel)


if __name__ == "__main__":
    rclpy.init ()
    node = LaserToImg ()

    executor = rclpy.executors.MultiThreadedExecutor (2)
    executor.add_node (node)
    th = Thread (target = executor.spin)
    th.start ()

    while node.scan is None:
        print ("Waiting for scan")
        sleep (0.5)

    node.dock (None)
    vel = Twist ()
    vel.linear.x = 0.1
    node.cmd_vel_pub.publish (vel)
    sleep (4)

    rclpy.shutdown ()
