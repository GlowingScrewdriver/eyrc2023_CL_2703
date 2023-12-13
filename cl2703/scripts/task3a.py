#!/usr/bin/python3

from task1a import aruco_tf
import rclpy

if __name__ == "__main__":
    rclpy.init ()
    node = aruco_tf (base_tf_fmt = "2703_base_{_id}", cam_tf_fmt = "2703_cam_{_id}")
    rclpy.spin (node)
    rclpy.shutdown ()
