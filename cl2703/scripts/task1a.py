#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          CL#2703
# Theme:            Cosmo Logistic
# Author List:		Vedanth Padmaraman, Vamshi Vishruth
# Filename:		    task1a.py
#
# Functions:        calculate_rectangle_area(coordinates), calculate_center (corners), detect_aruco(image)
# Classes:          aruco_tf (Node)
# Globals:          no_tf, detector, params

################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image


##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Purpose: To calculate the pixel area of an Aruco marker

    Arguments:
        coordinates (list of list):  list of 4 corners, specified as (x, y)

    Returns:
        area        (float):         area of detected aruco
        width       (float):         width of detected aruco (currently a dummy)
    '''

    x1_y1=coordinates[0]
    x2_y2=coordinates[1]
    x3_y3=coordinates[2]
    x4_y4=coordinates[3]

    # The rectangle is split into two triangles and the area of each is calculated
    triangle1=np.array([[x1_y1[0],x1_y1[1],1],[x2_y2[0],x2_y2[1],1],[x3_y3[0],x3_y3[1],1]])
    tri_1_Area=(1/2)*(np.linalg.det(triangle1))

    triangle2=np.array([[x1_y1[0],x1_y1[1],1],[x3_y3[0],x3_y3[1],1],[x4_y4[0],x4_y4[1],1]])
    tri_2_Area=(1/2)*(np.linalg.det(triangle2))


    area = tri_1_Area+tri_2_Area
    width = False

    return area, width


def calculate_center (corners):
    '''
    Purpose: To find the center of a marker

    Arguments:
        corners    (list of list):   list of 4 corners, specified as (x, y)

    Returns:
        center     (list):           list containing x and y
    '''
    # Construct linear eqns. for the diagonals and solve
    m0_2 = (corners[0][1] - corners[2][1])/(corners[0][0] - corners[2][0]) # Slope of diagonal 0-2
    m1_3 = (corners[1][1] - corners[3][1])/(corners[1][0] - corners[3][0]) # Slope of diagonal 1-3

    coeffs = np.array ([
        [m0_2, -1],
        [m1_3, -1],
    ])
    consts = np.array ([
        m0_2*corners[0][0] - corners[0][1],
        m1_3*corners[1][0] - corners[1][1],
    ])

    center = np.linalg.solve(coeffs, consts)
    return (int(center[0]), int(center[1]))


# Detector and  parameters objects
detector, params = cv2.aruco.ArucoDetector(), cv2.aruco.DetectorParameters()

# Detector parameters; these determine the accuracy of aruco detection
params.adaptiveThreshConstant = 20
params.adaptiveThreshWinSizeMin, params.adaptiveThreshWinSizeMax, params.adaptiveThreshWinSizeStep = 11, 11, 6
params.minMarkerPerimeterRate = 0.1
params.perspectiveRemovePixelPerCell = 3
params.perspectiveRemoveIgnoredMarginPerCell = 0.13
detector.setDetectorParameters (params)
detector.setDictionary (cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

# True if the process should stop at aruco detection; if False, TFs will also be published
# Useful for testing with ROSBags
no_tf = False
def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    # Pixel area threshold below which an Aruco marker will be rejected
    aruco_area_threshold = 1500

    # The camera matrix
    # Take from /camera/color/camera_info.k if needed
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0.
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = None

    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedCandidates = detector.detectMarkers (gray_img)

    if ids is None or ids.size == 0:
        return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, []

    # Filter out small markers
    ids_fil = []; corners_fil = []
    for n in range (len(ids)):
        area = calculate_rectangle_area(corners[n][0])[0]
        if area > aruco_area_threshold:
            ids_fil.append (ids[n])
            corners_fil.append (corners[n])

    # Calculate centers
    for n in range (len(ids_fil)):
        c = calculate_center (corners_fil[n][0])
        center_aruco_list.append(c)

    # Pose estimation
    angle_aruco_list, distance_from_rgb_list, objpts = cv2.aruco.estimatePoseSingleMarkers (corners_fil, size_of_aruco_m, cam_mat, dist_mat)

    if no_tf: # Some extra debug info if not publishing TFs
        for winsize in range (params.adaptiveThreshWinSizeMin, params.adaptiveThreshWinSizeMax + 1, params.adaptiveThreshWinSizeStep):
            th_img = cv2.adaptiveThreshold(gray_img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, winsize, params.adaptiveThreshConstant)
            cv2.aruco.drawDetectedMarkers (th_img, rejectedCandidates)
            cv2.imshow (f'Threshold -- WinSize {winsize}', th_img)

    # Report some useful results on the OpenCV output image
    for n in range (len(ids_fil)):
        cv2.drawFrameAxes(image, cam_mat, dist_mat, angle_aruco_list[n], distance_from_rgb_list[n], 0.5)
        cv2.circle (image, center_aruco_list[n], 4, (255, 0, 255), 2)
        cv2.putText (image, "center", (center_aruco_list[n][0] - 10, center_aruco_list[n][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.aruco.drawDetectedMarkers (image, corners, ids)

    cv2.imshow ('Aruco Detector', image)
    cv2.waitKey (1)

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids_fil


class aruco_tf(Node):
    '''
    This class is instantiated and spun as an ROS node. Handles the aruco detection process
    '''

    def __init__(self, base_tf_fmt='obj_{_id}', cam_tf_fmt='cam_{_id}'):
        super().__init__('aruco_tf_publisher')

        ############ Topic SUBSCRIPTIONS ############
        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############
        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id

        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        self.cv_image = None # Camera image in OpenCV format
        self.base_cam_tf = None # Position of the camera

        # These determine the frame names for the published TFs
        self.base_tf_fmt, self.cam_tf_fmt = base_tf_fmt, cam_tf_fmt

    def colorimagecb(self, data):
        '''
        Description: Callback for ROS Image -> CV2 image conversion
        '''
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')


    def process_image(self):
        '''
        Description: The main aruco detection routine. Calls detect_aruco and uses the result to publish TFs
        '''

        ############ Function VARIABLES ############
        # Keeping these here for now; but do we really need them?
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        if (self.cv_image is None): return

        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco (self.cv_image)
        if not ids: return
        if no_tf: return # Everything after this point is to do with TF publishing
        aruco_quat_list = [] # List of quaternions for each aruco

        #while not self.base_cam_tf:
        if not self.base_cam_tf:
            try:
                self.base_cam_tf = self.tf_buffer.lookup_transform (
                        'camera_link', f'base_link',
                        rclpy.time.Time()
                )
            except tf2_ros.LookupException as e:
                return
                print (e)
                pass

        cam_rot = R.from_quat ((
            self.base_cam_tf.transform.rotation.x,
            self.base_cam_tf.transform.rotation.y,
            self.base_cam_tf.transform.rotation.z,
            self.base_cam_tf.transform.rotation.w,
        ))

        for an in angle_aruco_list:
            # We are only interested in the yaw for each marker, since it is easy to
            # just use the yaw + an extra rotation that is common for all markers.
            yaw = R.from_rotvec(an[0]).as_euler('xyz')[2]
            marker_rot = cam_rot * R.from_euler ('zyz', [math.pi/2, math.pi/2, -yaw])
            aruco_quat_list += [ marker_rot.as_quat() ]



        for n in range(len(ids)):
            _id = ids[n][0]

            t = TransformStamped()
            t.header.frame_id = 'camera_link'
            t.child_frame_id = self.cam_tf_fmt.format (_id=_id)

            # Position
            # Coordinates obtained from OpenCV are in the marker's coordinate frame, which is not aligned with base_link
            t.transform.translation.x = distance_from_rgb_list[n][0][2]
            t.transform.translation.y = -distance_from_rgb_list[n][0][0]
            t.transform.translation.z = -distance_from_rgb_list[n][0][1]
            # Rotation
            t.transform.rotation.x = aruco_quat_list[n][0]
            t.transform.rotation.y = aruco_quat_list[n][1]
            t.transform.rotation.z = aruco_quat_list[n][2]
            t.transform.rotation.w = aruco_quat_list[n][3]

            # Transform from marker to camera
            self.br.sendTransform(t)


            try:
                # Transform from marker to base
                base_obj_tf = self.tf_buffer.lookup_transform (
                    'base_link', self.cam_tf_fmt.format (_id=_id),
                    rclpy.time.Time()
                )
                base_obj_tf.child_frame_id = self.base_tf_fmt.format (_id=_id)
                self.br.sendTransform (base_obj_tf)

            except tf2_ros.LookupException as e:
                print (e)
                pass


def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.declare_parameter ("no_tf", False)
    global no_tf
    no_tf = node.get_parameter("no_tf").get_parameter_value().bool_value
    if no_tf:
        print ("Not publishing transforms")

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    main()
